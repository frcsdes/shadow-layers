// integrators/statistics.cpp*
#include "statistics.h"

#include "camera.h"
#include "paramset.h"
#include "progressreporter.h"

namespace pbrt {

STAT_COUNTER("Integrator/Batch size", nSamplesBatch);
STAT_COUNTER("Integrator/Batches rendered", nBatches);
STAT_PERCENT("Integrator/Unconverged pixels", nUnconvergedPixels, nPixels);
STAT_INT_DISTRIBUTION("Integrator/Samples per pixel", samplesPerPixel);
STAT_FLOAT_DISTRIBUTION("Integrator/Variance per pixel", variancePerPixel);
STAT_COUNTER("Integrator/Render time in milliseconds", nElapsedMilliseconds);
STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

IntegratorStats::IntegratorStats(const ParamSet &params, Film *const film,
                                 const Sampler &sampler)
    : mode([](std::string &&modeString) {
          return modeString == "time"     ? Mode::TIME
               : modeString == "variance" ? Mode::VARIANCE
               :                            Mode::NORMAL;
      }(params.FindOneString("mode", "normal"))),
      batchSize(params.FindOneInt("batchsize", 32)),
      minSamples(sampler.samplesPerPixel),
      maxSamples(params.FindOneInt("maxsamples", 4 * minSamples)),
      maxSeconds(params.FindOneInt("seconds", 60)),
      maxVariance(params.FindOneFloat("variance", 0.1)),
      bounds(film->croppedPixelBounds),
      statsImages([film]() -> Images {
          return {*film, {
              {Stats::SPP,  "_spp"},
              {Stats::MEAN, "_mean"},
              {Stats::M2,   "_var"},
          }};
      }()) {}

void IntegratorStats::RenderBegin() {
    startTime = Clock::now();
}

void IntegratorStats::RenderEnd() const {
    nElapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>
                           (Clock::now() - startTime).count();
    if (mode == Mode::TIME)
        nSamplesBatch = batchSize;
}

bool IntegratorStats::RenderBatch() const {
    switch (mode) {
        case Mode::TIME:
            nBatches++;
            return std::chrono::duration_cast<std::chrono::seconds>
                   (Clock::now() - startTime).count() < maxSeconds;
        default:
            return false;
    }
}

void IntegratorStats::SamplesLoop(const Point2i &pixel, Tile &tile,
                                  SamplingFunctor sampleOnce) const {
    if (!InsideExclusive(pixel, tile.at(Stats::SPP)->GetPixelBounds())) {
        sampleOnce();
        return;
    }

    auto loop = [&]() {
        UpdateStats(pixel, tile, sampleOnce());
    };

    switch (mode) {
        case Mode::NORMAL:
            for (int s = 0; s < minSamples; ++s)
                loop();
            break;

        case Mode::TIME:
            for (int s = 0; s < batchSize; ++s)
                loop();
            break;

        case Mode::VARIANCE:
            for (int s = 0; s < minSamples; ++s)
                loop();
            while (!StopSampling(pixel, tile))
                loop();
            ReportStats(pixel, tile);
            break;
    }

    tile.at(Stats::SPP)->GetPixel(pixel).filterWeightSum
        = maxSamples - minSamples;
    tile.at(Stats::MEAN)->GetPixel(pixel).filterWeightSum = 1;
    tile.at(Stats::M2)->GetPixel(pixel).filterWeightSum
        = Samples(pixel, tile) - 1;
    tile.at(Stats::SPP)->GetPixel(pixel).contribSum
        = Samples(pixel, tile) - minSamples;
}

IntegratorStats::Images &IntegratorStats::StatsImages() {
    return statsImages;
}

Float &IntegratorStats::Samples(const Point2i &pixel, Tile &tile) const {
    return tile.at(Stats::SPP)->GetPixel(pixel).contribSum[0];
}

Spectrum &IntegratorStats::Mean(const Point2i &pixel, Tile &tile) const {
    return tile.at(Stats::MEAN)->GetPixel(pixel).contribSum;
}

Spectrum &IntegratorStats::Moment2(const Point2i &pixel, Tile &tile) const {
    return tile.at(Stats::M2)->GetPixel(pixel).contribSum;
}

void IntegratorStats::UpdateStats(const Point2i &pixel, Tile &tile,
                                 const Spectrum &L) const {
    Samples(pixel, tile) += 1;
    auto delta1 = L - Mean(pixel, tile);
    Mean(pixel, tile) += delta1 / Samples(pixel, tile);
    auto delta2 = L - Mean(pixel, tile);
    Moment2(pixel, tile) += delta1 * delta2;
}

Spectrum IntegratorStats::Variance(const Point2i &pixel, Tile &tile) const {
    return Samples(pixel, tile) > 1
         ? Moment2(pixel, tile) / (Samples(pixel, tile) - 1)
         : 0;
}

bool IntegratorStats::Converged(const Point2i &pixel, Tile &tile) const {
    return Variance(pixel, tile).y() <= maxVariance;
}

bool IntegratorStats::StopSampling(const Point2i &pixel, Tile &tile) const {
    return Converged(pixel, tile) || Samples(pixel, tile) >= maxSamples;
}

void IntegratorStats::ReportStats(const Point2i &pixel, Tile &tile) const {
    nPixels++;
    if (!Converged(pixel, tile))
        nUnconvergedPixels++;
    ReportValue(samplesPerPixel, Samples(pixel, tile));
    ReportValue(variancePerPixel, Variance(pixel, tile).y());
}

// PathIntegratorStats Method Definitions
PathIntegratorStats::PathIntegratorStats(const ParamSet &params,
                                         int maxDepth,
                                         std::shared_ptr<const Camera> camera,
                                         std::shared_ptr<Sampler> sampler,
                                         const Bounds2i &pixelBounds,
                                         Float rrThreshold,
                                         const std::string &lightSampleStrategy)
    : PathIntegrator(maxDepth, camera, sampler, pixelBounds, rrThreshold,
                     lightSampleStrategy),
      IntegratorStats(params, camera->film, *sampler),
      sampler(std::move(sampler)),
      pixelBounds(pixelBounds) {}

void PathIntegratorStats::Render(const Scene &scene) {
    Preprocess(scene, *sampler);
    // Render image tiles in parallel

    // Compute number of tiles, _nTiles_, to use for parallel rendering
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i sampleExtent = sampleBounds.Diagonal();
    const int tileSize = 16;
    Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
                   (sampleExtent.y + tileSize - 1) / tileSize);

    RenderBegin();
    ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");
    {
        do {
            ParallelFor2D([&](Point2i tile) {
                // Render section of image corresponding to _tile_

                // Allocate _MemoryArena_ for tile
                MemoryArena arena;

                // Get sampler instance for tile
                int seed = tile.y * nTiles.x + tile.x;
                std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);

                // Compute sample bounds for tile
                int x0 = sampleBounds.pMin.x + tile.x * tileSize;
                int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
                int y0 = sampleBounds.pMin.y + tile.y * tileSize;
                int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
                Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
                LOG(INFO) << "Starting image tile " << tileBounds;

                // Get _FilmTile_ for tile
                std::unique_ptr<FilmTile> filmTile =
                    camera->film->GetFilmTile(tileBounds);
                auto statsTile = StatsImages().GetFilmTile(tileBounds);

                // Loop over pixels in tile to render them
                for (Point2i pixel : tileBounds) {
                    {
                        ProfilePhase pp(Prof::StartPixel);
                        tileSampler->StartPixel(pixel);
                    }

                    // Do this check after the StartPixel() call; this keeps
                    // the usage of RNG values from (most) Samplers that use
                    // RNGs consistent, which improves reproducability /
                    // debugging.
                    if (!InsideExclusive(pixel, pixelBounds))
                        continue;

                    SamplesLoop(pixel, statsTile, [&]() {
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
                        Spectrum L(0.f);
                        if (rayWeight > 0) L = Li(ray, scene, *tileSampler,
                                                  arena, 0);

                        // Issue warning if unexpected radiance value returned
                        if (L.HasNaNs()) {
                            LOG(ERROR) << StringPrintf(
                                "Not-a-number radiance value returned for "
                                "pixel (%d, %d), sample %d. Setting to black.",
                                pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        } else if (L.y() < -1e-5) {
                            LOG(ERROR) << StringPrintf(
                                "Negative luminance value, %f, returned for "
                                "pixel (%d, %d), sample %d. Setting to black.",
                                L.y(), pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        } else if (std::isinf(L.y())) {
                              LOG(ERROR) << StringPrintf(
                                "Infinite luminance value returned for "
                                "pixel (%d, %d), sample %d. Setting to black.",
                                pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        }
                        VLOG(1) << "Camera sample: " << cameraSample
                                << " -> ray: " << ray
                                << " -> L = " << L;

                        // Add camera ray's contribution to image
                        filmTile->AddSample(cameraSample.pFilm, L, rayWeight);

                        // Free _MemoryArena_ memory from computing image sample
                        // value
                        arena.Reset();
                        tileSampler->StartNextSample();

                        return L;
                    });
                }
                LOG(INFO) << "Finished image tile " << tileBounds;

                // Merge image tile into _Film_
                camera->film->MergeFilmTile(std::move(filmTile));
                StatsImages().MergeFilmTile(statsTile);
            }, nTiles);
        } while (RenderBatch());
        reporter.Done();
    }
    RenderEnd();
    LOG(INFO) << "Rendering finished";

    // Save final image after rendering
    camera->film->WriteImage();
    StatsImages().WriteSeparateImages();
}

PathIntegratorStats *CreatePathIntegratorStats(
    const ParamSet &params,
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera
) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    Float rrThreshold = params.FindOneFloat("rrthreshold", 1.);
    std::string lightStrategy =
        params.FindOneString("lightsamplestrategy", "spatial");

    return new PathIntegratorStats(params, maxDepth, camera, sampler,
                                   pixelBounds, rrThreshold, lightStrategy);
}

// ShadowIntegratorStats Method Definitions
ShadowIntegratorStats::ShadowIntegratorStats(const ParamSet &params,
                                             int maxDepth, int maxSkips,
                                             Float skipProb,
                                             std::shared_ptr<const Camera> cam,
                                             std::shared_ptr<Sampler> sampler,
                                             NamedObjects namedCasters,
                                             NamedObjects namedCatchers,
                                             NamedObjects noSelfShadow,
                                             bool singleFile,
                                             bool splitLights, bool splitDirect,
                                             const Bounds2i &pixelBounds,
                                             Float rrThreshold, const
                                             std::string &lightSampleStrategy)
    : ShadowIntegrator(maxDepth, maxSkips, skipProb, cam, sampler,
                       std::move(namedCasters), std::move(namedCatchers),
                       std::move(noSelfShadow), singleFile,
                       splitLights, splitDirect, pixelBounds, rrThreshold,
                       lightSampleStrategy),
      IntegratorStats(params, cam->film, *sampler),
      camera(std::move(cam)),
      sampler(std::move(sampler)),
      pixelBounds(pixelBounds) {}

void ShadowIntegratorStats::Render(const Scene &scene) {
    ProfilePhase p(Prof::IntegratorRender);

    Preprocess(scene);

    // Render image tiles in parallel
    // Compute number of tiles, _nTiles_, to use for parallel rendering
    Bounds2i sampleBounds = camera->film->GetSampleBounds();
    Vector2i sampleExtent = sampleBounds.Diagonal();
    const int tileSize = 16;
    Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
                   (sampleExtent.y + tileSize - 1) / tileSize);

    RenderBegin();
    ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");
    {
        do {
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
                auto layeredTile = GetLayers().GetFilmTile(tileBounds);
                auto statsTile = StatsImages().GetFilmTile(tileBounds);

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

                    SamplesLoop(pixel, statsTile, [&]() {
                        // Initialize _CameraSample_ for current sample
                        CameraSample cameraSample =
                            tileSampler->GetCameraSample(pixel);

                        // Generate camera ray for current sample
                        RayDifferential ray;
                        Float rayWeight =
                            camera->GenerateRayDifferential(cameraSample, &ray);
                        ray.ScaleDifferentials(1 /
                            std::sqrt((Float) tileSampler->samplesPerPixel));
                        ++nCameraRays;

                        // Evaluate radiance along camera ray. Contributions
                        // to the various layers are made inside Li
                        Spectrum L;
                        if (rayWeight > 0)
                            L = Li(ray, scene, *tileSampler, *skipSampler,
                                   arena, layeredTile.samples);

                        layeredTile.AddSample(cameraSample.pFilm, rayWeight);

                        // Free _MemoryArena_ memory from computing image sample
                        // value
                        arena.Reset();
                        tileSampler->StartNextSample();
                        skipSampler->StartNextSample();

                        return L;
                    });
                }
                LOG(INFO) << "Finished image tile " << tileBounds;

                // Merge image tiles into _FilmLayers_
                GetLayers().MergeFilmTile(layeredTile);
                StatsImages().MergeFilmTile(statsTile);
            }, nTiles);
        } while (RenderBatch());
        reporter.Done();
    }
    RenderEnd();
    LOG(INFO) << "Rendering finished";

    // Save final images after rendering
    GetLayers().WriteImage();
    StatsImages().WriteSeparateImages();
}

ShadowIntegratorStats *CreateShadowIntegratorStats(
    const ParamSet &params,
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera
) {
    // Check that the filename extension corresponds to an EXR image
    const auto &filename = camera->film->filename;
    auto extension = filename.substr(filename.find_last_of('.'));
    if (extension != ".exr") {
        Warning("The shadow integrator outputs a multi-layer EXR file. "
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

    return new ShadowIntegratorStats(params, maxDepth, maxSkips, skipProb,
                                     std::move(camera), std::move(sampler),
                                     std::move(namedCasters),
                                     std::move(namedCatchers),
                                     std::move(noSelfShadow),
                                     singleFile, splitLights, splitDirect,
                                     pixelBounds, rrThreshold, lightStrategy);
}

}  // namespace pbrt
