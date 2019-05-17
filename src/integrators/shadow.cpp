// integrators/shadow.cpp*
#include "shadow.h"

#include "bssrdf.h"
#include "camera.h"
#include "interaction.h"
#include "paramset.h"
#include "progressreporter.h"
#include "scene.h"
#include "stats.h"

#include <sstream>

namespace pbrt {

STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);
STAT_PERCENT("Integrator/Zero-radiance paths", zeroRadiancePaths, totalPaths);
STAT_INT_DISTRIBUTION("Integrator/Path length", pathLength);


ShadowObject::ShadowObject(std::size_t id)
    : defined(true),
      identifier(id) {}

ShadowObject::ShadowObject(const SurfaceInteraction &isect)
    : defined(true),
      identifier(isect.primitive->GetMaterial()
               ? isect.primitive->GetMaterial()->Identifier() : 0) {}

bool operator==(ShadowObject lhs, ShadowObject rhs) {
    return lhs.defined == rhs.defined
        && lhs.identifier == rhs.identifier;
}

bool operator!=(ShadowObject lhs, ShadowObject rhs) {
    return !(lhs == rhs);
}

ShadowObject::operator bool() const {
    return defined;
}

ShadowSpectrum operator*(Float x, ShadowSpectrum &&S) {
    S.LL *= x;
    S.WL *= x;
    return S;
}

ShadowSpectrum operator/(ShadowSpectrum &&S, Float x) {
    S.LL /= x;
    S.WL /= x;
    return S;
}

ShadowSpectrum operator*(Spectrum L, ShadowSpectrum &&S) {
    S.LL *= L;
    S.WL *= L;
    return S;
}

bool operator==(const ShadowLayer &lhs, const ShadowLayer &rhs) {
    return lhs.caster == rhs.caster
        && lhs.light == rhs.light
        && lhs.order == rhs.order;
}


// ShadowIntegrator Method Definitions
ShadowIntegrator::ShadowIntegrator(int maxDepth, int maxSkips, Float skipProb,
                                   std::shared_ptr<const Camera> camera,
                                   std::shared_ptr<Sampler> sampler,
                                   NamedObjects namedCasters,
                                   NamedObjects namedCatchers,
                                   NamedObjects noSelfShadow,
                                   bool singleFile,
                                   bool splitLights, bool splitDirect,
                                   const Bounds2i &pixelBounds,
                                   Float rrThreshold,
                                   const std::string &lightSampleStrategy)
    : camera(std::move(camera)),
      sampler(std::move(sampler)),
      pixelBounds(pixelBounds),
      maxDepth(maxDepth),
      maxSkips(maxSkips),
      skipProb(skipProb),
      casters(std::move(namedCasters)),
      catchers(std::move(namedCatchers)),
      noSelfShadow(std::move(noSelfShadow)),
      noneEncountered(defaultEncounterMap(casters)),
      singleFile(singleFile),
      splitLights(splitLights),
      splitDirect(splitDirect),
      rrThreshold(rrThreshold),
      lightSampleStrategy(lightSampleStrategy) {}

void ShadowIntegrator::Preprocess(const Scene &scene) {
    lightDistrib = CreateLightSampleDistribution(lightSampleStrategy, scene);

    // Put tracked lights inside the same container
    decltype(scene.lights) allLights;
    if (splitLights) {
        allLights.insert(allLights.begin(), scene.lights.begin(),
                                            scene.lights.end());
        allLights.insert(allLights.begin(), scene.infiniteLights.begin(),
                                            scene.infiniteLights.end());
    } else
        allLights.push_back(nullptr);

    auto layerName = [this](ShadowObject caster, int light, ShadowOrder order) {
        auto sep = singleFile ? '.' : '_';
        std::ostringstream name;

        if (!singleFile)
            name << sep;

        name << "Shadow" << sep << casters.at(caster);

        if (splitLights)
            name << sep << "Light" << light;

        if (splitDirect) {
            name << sep;
            switch (order) {
                case ShadowOrder::DIRECT:
                    name << "Direct";
                    break;
                case ShadowOrder::INDIRECT:
                    name << "Indirect";
                    break;
                default:
                    break;
            }
        }

        if (singleFile)
            name << sep;

        return name.str();
    };

    Layers::Map<std::string> layerNames;
    // Loop over casters * lights and add layers
    for (const auto &casterKey : casters) {
        const auto &caster = casterKey.first;
        unsigned int lightIndex = 0;

        for (const auto &light : allLights) {
            if (splitDirect) {
                layerNames.emplace(
                    ShadowLayer{caster, light.get(), ShadowOrder::DIRECT},
                    layerName(caster, lightIndex, ShadowOrder::DIRECT));
                layerNames.emplace(
                    ShadowLayer{caster, light.get(), ShadowOrder::INDIRECT},
                    layerName(caster, lightIndex, ShadowOrder::INDIRECT));
            } else {
                layerNames.emplace(
                    ShadowLayer{caster, light.get(), ShadowOrder::ALL},
                    layerName(caster, lightIndex, ShadowOrder::ALL));
            }

            lightIndex++;
        }
    }
    layerNames.emplace(mainLayer, "");
    layers = {*camera->film, std::move(layerNames)};
}

void ShadowIntegrator::Render(const Scene &scene) {
    ProfilePhase p(Prof::IntegratorRender);

    Preprocess(scene);

    // Render image tiles in parallel
    // Compute number of tiles, _nTiles_, to use for parallel rendering
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i sampleExtent = sampleBounds.Diagonal();
    const int tileSize = 16;
    Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
                   (sampleExtent.y + tileSize - 1) / tileSize);
    ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");
    {
        ParallelFor2D([&](Point2i tile) {
            // Render section of image corresponding to _tile_

            // Allocate _MemoryArena_ for tile
            MemoryArena arena;

            // Get sampler instance for tile
            // A second sampler is needed to avoid patterns appearing due to
            // correlation between pixel position and caster skipping
            int seed = tile.y * nTiles.x + tile.x;
            std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);
            std::unique_ptr<Sampler> skipSampler = sampler->Clone(seed);

            // Compute sample bounds for tile
            int x0 = sampleBounds.pMin.x + tile.x * tileSize;
            int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
            int y0 = sampleBounds.pMin.y + tile.y * tileSize;
            int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
            Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
            LOG(INFO) << "Starting image tile " << tileBounds;

            // Get a tile for each layer
            auto layeredTile = layers.GetFilmTile(tileBounds);

            // Loop over pixels in tile to render them
            for (Point2i pixel : tileBounds) {
                {
                    ProfilePhase pp(Prof::StartPixel);
                    tileSampler->StartPixel(pixel);
                    skipSampler->StartPixel(pixel);
                }

                // Do this check after the BeginPixel() call; this keeps
                // the usage of RNG values from (most) Samplers that use
                // RNGs consistent, which improves reproducibility /
                // debugging.
                if (!InsideExclusive(pixel, pixelBounds))
                    continue;

                do {
                    // Initialize _CameraSample_ for current sample
                    CameraSample cameraSample =
                        tileSampler->GetCameraSample(pixel);

                    // Generate camera ray for current sample
                    RayDifferential ray;
                    Float rayWeight =
                        camera->GenerateRayDifferential(cameraSample, &ray);
                    ray.ScaleDifferentials(
                        1 / std::sqrt((Float)tileSampler->samplesPerPixel));
                    ++nCameraRays;

                    // Evaluate radiance along camera ray
                    // Contributions to the various layers are made inside Li
                    if (rayWeight > 0)
                        Li(ray, scene, *tileSampler, *skipSampler, arena,
                           layeredTile.samples);

                    // Add the contributions made to the layers as samples
                    layeredTile.AddSample(cameraSample.pFilm, rayWeight);

                    // Free _MemoryArena_ memory from computing image sample
                    // value
                    arena.Reset();
                } while (tileSampler->StartNextSample() &&
                         skipSampler->StartNextSample());
            }
            LOG(INFO) << "Finished image tile " << tileBounds;

            // Merge image tiles into _FilmLayers_
            layers.MergeFilmTile(layeredTile);

            reporter.Update();
        }, nTiles);
        reporter.Done();
    }
    LOG(INFO) << "Rendering finished";

    // Save final image after rendering
    if (singleFile)
        layers.WriteImage();
    else
        layers.WriteSeparateImages();
}

Spectrum ShadowIntegrator::Li(const RayDifferential &r, const Scene &scene,
                              Sampler &sampler, Sampler &skipSampler,
                              MemoryArena &arena, TileSamples &samples, int)
                              const {
    ProfilePhase p(Prof::SamplerIntegratorLi);

    Spectrum beta(1);
    RayDifferential ray(r);
    bool specularBounce = false;
    int bounces;
    int skips = 0;

    // The assigner caster that will be skipped when encountered
    ShadowObject assignedCaster;

    // True if the ray can skip a caster
    bool alteredPropagation = false;

    // Keep track of the encountered casters
    auto hasEncountered = noneEncountered;

    // Radiance contribution to the different layers is handled by this lambda
    auto contribute = [&](const Light *light, Spectrum L,
                          ShadowObject tempCaster = {}) {
        ProfilePhase pp(Prof::ShadowIntegratorContribute);
        const auto *l = splitLights ? light : nullptr;

        if (assignedCaster) {
            auto order = splitDirect ? ShadowOrder::INDIRECT : ShadowOrder::ALL;
            samples.at({assignedCaster, l, order}) += L;
        } else if (tempCaster) {
            auto order = splitDirect ? ShadowOrder::DIRECT : ShadowOrder::ALL;
            samples.at({tempCaster, l, order}) += L;
        } else
            samples.at(mainLayer) += L;
    };

    // Added after book publication: etaScale tracks the accumulated effect
    // of radiance scaling due to rays passing through refractive
    // boundaries (see the derivation on p. 527 of the third edition). We
    // track this value in order to remove it from beta when we apply
    // Russian roulette; this is worthwhile, since it lets us sometimes
    // avoid terminating refracted rays that are about to be refracted back
    // out of a medium and thus have their beta value increased.
    Float etaScale = 1;


    for (bounces = 0;; ++bounces) {
        // Intersect _ray_ with scene and store intersection in _isect_
        SurfaceInteraction isect;
        bool foundIntersection = scene.Intersect(ray, &isect);


        // Possibly add emitted light at intersection
        if (bounces == 0 || specularBounce) {
            if (foundIntersection) {
                if (const auto *light = isect.primitive->GetAreaLight())
                    contribute(light, beta * isect.Le(-ray.d));
            } else {
                for (const auto &light : scene.infiniteLights)
                    contribute(light.get(), beta * light->Le(ray));
            }
        }

        // Terminate path if ray escaped or _maxDepth_ was reached
        if (!foundIntersection || bounces >= maxDepth)
            break;


        ShadowObject object(isect);
        if (alteredPropagation) {
            if (IsCaster(object)) {
                // Possibly assign the caster to the path
                if (!assignedCaster && !hasEncountered[object]) {
                    if (skipSampler.Get1D() < skipProb) {
                        assignedCaster = object;
                        beta /= skipProb;
                    } else
                        beta /= 1 - skipProb;
                }
                hasEncountered[object] = true;

                // The assigned caster was encountered, skip it
                if (object == assignedCaster) {
                    if (skips++ < maxSkips) {
                        ray = isect.SpawnRay(ray.d);
                        bounces--;
                        continue;
                    } else
                        break;
                }
            }
        } else if (IsCatcher(object)) {
            alteredPropagation = true;
            if (NoSelfShadow(object))
                hasEncountered[object] = true;
        }


        // Compute scattering functions and skip over medium boundaries
        isect.ComputeScatteringFunctions(ray, arena, true);

        if (!isect.bsdf) {
            ray = isect.SpawnRay(ray.d);
            bounces--;
            continue;
        }


        const Distribution1D *distrib = lightDistrib->Lookup(isect.p);

        if (isect.bsdf->NumComponents(BxDFType(BSDF_ALL & ~BSDF_SPECULAR)) >
            0) {
            auto S = beta * SampleLight(isect, scene, arena, sampler,
                                        alteredPropagation, assignedCaster,
                                        hasEncountered, false, distrib);

            ++totalPaths;
            if (S.LL.IsBlack() && S.WL.IsBlack())
                ++zeroRadiancePaths;
            else {
                contribute(S.light, S.LL, S.LOccluder);
                contribute(S.light, S.WL, S.WOccluder);
            }
        }

        // Sample BSDF to get new path direction
        Vector3f wo = -ray.d, wi;
        Float pdf;
        BxDFType flags;
        Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                                          BSDF_ALL, &flags);

        if (f.IsBlack() || pdf == 0.f) break;
        beta *= f * AbsDot(wi, isect.shading.n) / pdf;
        DCHECK(!std::isinf(beta.y()));
        specularBounce = (flags & BSDF_SPECULAR) != 0;
        if ((flags & BSDF_SPECULAR) && (flags & BSDF_TRANSMISSION)) {
            Float eta = isect.bsdf->eta;
            // Update the term that tracks radiance scaling for refraction
            // depending on whether the ray is entering or leaving the
            // medium.
            etaScale *= (Dot(wo, isect.n) > 0) ? (eta * eta) : 1 / (eta * eta);
        }
        ray = isect.SpawnRay(wi);

        // Possibly terminate the path with Russian roulette.
        // Factor out radiance scaling due to refraction in rrBeta.
        Spectrum rrBeta = beta * etaScale;
        if (rrBeta.MaxComponentValue() < rrThreshold && bounces > 3) {
            Float q = std::max((Float).05, 1 - rrBeta.MaxComponentValue());
            if (sampler.Get1D() < q) break;
            beta /= 1 - q;
            DCHECK(!std::isinf(beta.y()));
        }
    }

    ReportValue(pathLength, bounces);
    return samples.at(mainLayer);
}

ShadowIntegrator::Layers &ShadowIntegrator::GetLayers() {
    return layers;
}

ShadowSpectrum
ShadowIntegrator::SampleLight(const Interaction &it, const Scene &scene,
                              MemoryArena &arena, Sampler &sampler,
                              bool alteredPropagation,
                              ShadowObject assignedCaster,
                              const EncounterMap &hasEncountered,
                              bool handleMedia,
                              const Distribution1D *lightDistrib) const {
    ProfilePhase p(Prof::DirectLighting);
    // Randomly choose a single light to sample, _light_
    int nLights = int(scene.lights.size());
    if (nLights == 0) return {};
    int lightNum;
    Float lightPdf;
    if (lightDistrib) {
        lightNum = lightDistrib->SampleDiscrete(sampler.Get1D(), &lightPdf);
        if (lightPdf == 0) return {};
    } else {
        lightNum = std::min((int)(sampler.Get1D() * nLights), nLights - 1);
        lightPdf = Float(1) / nLights;
    }
    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    Point2f uLight = sampler.Get2D();
    Point2f uScattering = sampler.Get2D();
    return DirectLight(it, uScattering, *light, uLight, scene, sampler, arena,
                       alteredPropagation, assignedCaster, hasEncountered,
                       handleMedia) / lightPdf;
}

ShadowSpectrum
ShadowIntegrator::DirectLight(const Interaction &it, const Point2f &uScattering,
                              const Light &light, const Point2f &uLight,
                              const Scene &scene, Sampler &sampler,
                              MemoryArena &arena, bool alteredPropagation,
                              ShadowObject assignedCaster,
                              const EncounterMap &hasEncountered,
                              bool handleMedia) const {
    BxDFType bsdfFlags = BxDFType(BSDF_ALL & ~BSDF_SPECULAR);

    Spectrum LLd(0);
    Spectrum WLd(0);
    ShadowObject LTempCaster = assignedCaster;
    ShadowObject WTempCaster = assignedCaster;

    // Sample light source with multiple importance sampling
    Vector3f wi;
    Float lightPdf = 0, scatteringPdf = 0;
    VisibilityTester visibility;

    Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
    VLOG(2) << "EstimateDirect uLight:" << uLight << " -> Li: " << Li
            << ", wi: " << wi << ", pdf: " << lightPdf;
    if (lightPdf > 0 && !Li.IsBlack()) {
        // Compute BSDF or phase function's value for light sample
        Spectrum f;
        if (it.IsSurfaceInteraction()) {
            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction&)it;
            f = isect.bsdf->f(isect.wo, wi, bsdfFlags) *
                AbsDot(wi, isect.shading.n);
            scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, bsdfFlags);
            VLOG(2) << "  surf f*dot :" << f
                    << ", scatteringPdf: " << scatteringPdf;
        } else {
            // Evaluate phase function for light sampling strategy
            const MediumInteraction &mi = (const MediumInteraction&)it;
            Float p = mi.phase->p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;
            VLOG(2) << "  medium p: " << p;
        }
        if (!f.IsBlack()) {
            // Compute effect of visibility for light source sample
            if (handleMedia) {
                Li *= visibility.Tr(scene, sampler);
                VLOG(2) << "  after Tr, Li: " << Li;
            } else {
                auto pLight = visibility.P1();
                Ray ray = it.SpawnRayTo(pLight);
                bool hit;
                int skips = 0;
                SurfaceInteraction isect;
                ShadowObject hitObject;

                while ((hit = scene.Intersect(ray, &isect)) &&
                       skips++ < maxSkips) {
                    hitObject = ShadowObject(isect);

                    // If an object is being skipped
                    if (LTempCaster) {
                        // But another was hit
                        if (LTempCaster != hitObject)
                            break;
                    }

                    // If no object is being skipped
                    else {
                        // And the one that was hit can be skipped, skip it
                        if (alteredPropagation && IsCaster(hitObject) &&
                            !hasEncountered.at(hitObject))
                            LTempCaster = hitObject;
                        else
                            break;
                    }

                    ray = isect.SpawnRayTo(pLight);
                }

                if (hit) {
                    VLOG(2) << "  shadow ray blocked";
                    Li = {0};
                } else
                    VLOG(2) << "  shadow ray unoccluded";
            }

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack()) {
                if (IsDeltaLight(light.flags))
                    LLd += f * Li / lightPdf;
                else {
                    Float weight =
                        PowerHeuristic(1, lightPdf, 1, scatteringPdf);
                    LLd += f * Li * weight / lightPdf;
                }
            }
        }
    }

    // Sample BSDF with multiple importance sampling
    if (!IsDeltaLight(light.flags)) {
        Spectrum f;
        bool sampledSpecular = false;
        if (it.IsSurfaceInteraction()) {
            // Sample scattered direction for surface interactions
            BxDFType sampledType;
            const SurfaceInteraction &isect = (const SurfaceInteraction&)it;
            f = isect.bsdf->Sample_f(isect.wo, &wi, uScattering, &scatteringPdf,
                                     bsdfFlags, &sampledType);
            f *= AbsDot(wi, isect.shading.n);
            sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;
        } else {
            // Sample scattered direction for medium interactions
            const MediumInteraction &mi = (const MediumInteraction&)it;
            Float p = mi.phase->Sample_p(mi.wo, &wi, uScattering);
            f = Spectrum(p);
            scatteringPdf = p;
        }
        VLOG(2) << "  BSDF / phase sampling f: " << f << ", scatteringPdf: " <<
                scatteringPdf;
        if (!f.IsBlack() && scatteringPdf > 0) {
            // Account for light contributions along sampled direction _wi_
            Float weight = 1;
            if (!sampledSpecular) {
                lightPdf = light.Pdf_Li(it, wi);
                if (lightPdf == 0)
                    return {&light, LTempCaster, LLd, WTempCaster, WLd};
                weight = PowerHeuristic(1, scatteringPdf, 1, lightPdf);
            }

            // Find intersection and compute transmittance
            Spectrum Tr(1.f);
            Spectrum Li(0.f);

            Ray ray = it.SpawnRay(wi);
            bool hit;
            int skips = 0;
            SurfaceInteraction isect;
            ShadowObject hitObject;

            // Add light contribution from material sampling
            while ((hit = scene.Intersect(ray, &isect)) && skips++ < maxSkips) {
                hitObject = ShadowObject(isect);

                // If the light is intersected, contribute its _Le_ and break
                if (isect.primitive->GetAreaLight() == &light) {
                    Li = isect.Le(-wi);
                    break;
                }

                // If an object is being skipped
                if (WTempCaster) {
                    // But another was hit
                    if (WTempCaster != hitObject)
                        break;
                }

                // If no object is being skipped
                else {
                    // And the one that was hit can be skipped, skip it
                    if (alteredPropagation && IsCaster(hitObject) &&
                        !hasEncountered.at(hitObject))
                        WTempCaster = hitObject;
                    else
                        break;
                }

                ray = isect.SpawnRay(ray.d);
            }

            // If no intersection was found, contribute infinite light
            if (!hit)
                Li = light.Le(ray);

            if (!Li.IsBlack()) WLd += f * Li * Tr * weight / scatteringPdf;
        }
    }

    return {&light, LTempCaster, LLd, WTempCaster, WLd};
}

bool ShadowIntegrator::IsCaster(ShadowObject object) const {
    return casters.find(object) != casters.end();
}

bool ShadowIntegrator::IsCatcher(ShadowObject object) const {
    return catchers.empty() || catchers.find(object) != catchers.end();
}

bool ShadowIntegrator::NoSelfShadow(ShadowObject object) const {
    return noSelfShadow.find(object) != noSelfShadow.end();
}

const ShadowLayer ShadowIntegrator::mainLayer {{}, nullptr, ShadowOrder::ALL};

ShadowIntegrator::EncounterMap
ShadowIntegrator::defaultEncounterMap(const NamedObjects& casters) {
    EncounterMap defaultMap;
    for (const auto &key : casters)
        defaultMap.emplace(key.first, false);
    return defaultMap;
}


ShadowIntegrator *CreateShadowIntegrator(const ParamSet &params,
                                         std::shared_ptr<Sampler> sampler,
                                         std::shared_ptr<const Camera> camera) {
    // Check that the filename extension corresponds to an EXR image
    const auto &filename = camera->film->filename;
    auto extension = filename.substr(filename.find_last_of('.'));
    if (extension != ".exr") {
        Warning("The shadow integrator outputs an EXR file. "
                "The provided extension was %s.", extension.c_str());
    }

    // There is no such thing as russian roulette for skipping surface, so a
    // maximum number of skips must be set as for depth
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int maxSkips = params.FindOneInt("maxskips", maxDepth);

    std::hash<std::string> hasher;
    // Track the specified casters
    ShadowIntegrator::NamedObjects namedCasters;
    int castersCount = 0;
    const auto *casters = params.FindString("casters", &castersCount);
    for (int i = 0; i < castersCount; ++i)
        namedCasters.emplace(ShadowObject(hasher(casters[i])), casters[i]);

    // Track the specified catchers
    ShadowIntegrator::NamedObjects namedCatchers;
    int catchersCount = 0;
    const auto *catchers = params.FindString("catchers", &catchersCount);
    for (int i = 0; i < catchersCount; ++i)
        namedCatchers.emplace(ShadowObject(hasher(catchers[i])), catchers[i]);

    // Discard self-shadowing for the specified catchers
    // The wildcard '*' can be used to specify all casters
    ShadowIntegrator::NamedObjects noSelfShadow;
    int noSelfCount = 0;
    const auto *noSelf = params.FindString("noselfshadow", &noSelfCount);
    if (noSelfCount == 1 && *noSelf == "*")
        noSelfShadow = namedCasters;
    else {
        for (int i = 0; i < noSelfCount; ++i)
            noSelfShadow.emplace(ShadowObject(hasher(noSelf[i])), noSelf[i]);
    }

    bool singleFile = params.FindOneBool("singlefile", true);
    bool splitLights = params.FindOneBool("splitlights", false);
    bool splitDirect = params.FindOneBool("splitdirect", false);

    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i({pb[0], pb[2]}, {pb[1], pb[3]}));
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }

    Float skipProb = params.FindOneFloat("skipprob", Float(1) / 2);
    if (skipProb < 0 || skipProb > 1) {
        Error("Skip probability must be inside interval [0, 1]. "
              "A value of %f was specified.", skipProb);
    }

    Float rrThreshold = params.FindOneFloat("rrthreshold", Float(1));
    auto lightStrategy = params.FindOneString("lightsamplestrategy", "spatial");

    return new ShadowIntegrator(maxDepth, maxSkips, skipProb,
                                std::move(camera), std::move(sampler),
                                std::move(namedCasters),
                                std::move(namedCatchers),
                                std::move(noSelfShadow),
                                singleFile, splitLights, splitDirect,
                                pixelBounds, rrThreshold, lightStrategy);
}

}  // namespace pbrt
