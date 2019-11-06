// integrators/statistics.cpp*
#include "statistics.h"

#include "camera.h"
#include "paramset.h"
#include "progressreporter.h"
#include "filters/box.h"

namespace pbrt {

STAT_COUNTER("Integrator/Batch size", nSamplesBatch);
STAT_COUNTER("Integrator/Batches rendered", nBatches);
STAT_PERCENT("Integrator/Unconverged pixels", nUnconvergedPixels, nPixels);
STAT_INT_DISTRIBUTION("Integrator/Samples per pixel", samplesPerPixel);
STAT_FLOAT_DISTRIBUTION("Integrator/Variance per pixel", variancePerPixel);
STAT_COUNTER("Integrator/Render time in milliseconds", nElapsedMilliseconds);
STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

Statistics::Statistics(const ParamSet &params, const Film *film,
                       const Sampler &sampler)
    : mode([](const std::string &modeString) {
              return modeString == "time"     ? Mode::TIME
                   : modeString == "variance" ? Mode::VARIANCE
                   :                            Mode::NORMAL;
          }(params.FindOneString("mode", "normal"))),
      maxSamples{sampler.samplesPerPixel},
      minSamples{params.FindOneInt("minsamples", 128)},
      maxVariance{params.FindOneFloat("maxvariance", 0.01)},
      batchSize{params.FindOneInt("batchsize", 8)},
      maxSeconds{params.FindOneInt("maxseconds", 60)},
      originalFilm(film),
      pixelBounds(film->croppedPixelBounds),
      pixels(new Pixel[pixelBounds.Area()])
      { CHECK_GE(maxSamples, minSamples); }

void Statistics::RenderBegin() {
    startTime = Clock::now();
}

void Statistics::RenderEnd() const {
    nElapsedMilliseconds = ElapsedMilliseconds();
    if (mode == Mode::TIME)
        nSamplesBatch = BatchSize();
}

void Statistics::WriteImages() const {
    CHECK(originalFilm);
    Film samplingFilm(*originalFilm, StatImagesFilter(), "_sampling");
    Film varianceFilm(*originalFilm, StatImagesFilter(), "_variance");

    for (Point2i pixel : pixelBounds) {
        Point2f floatPixel(static_cast<Float>(pixel.x),
                           static_cast<Float>(pixel.y));
        const Pixel &statsPixel = GetPixel(pixel);
        samplingFilm.AddSplat(floatPixel, Sampling(statsPixel));
        varianceFilm.AddSplat(floatPixel, Variance(statsPixel));
    }

    samplingFilm.WriteImage();
    varianceFilm.WriteImage();
}

bool Statistics::StartNextBatch(int index) {
    if (mode == Mode::TIME) {
        bool start = (index + 1) * batchSize < maxSamples
                  && ElapsedMilliseconds() < maxSeconds * 1000;
        if (start) nBatches++;
        return start;
    }

    else {
        if (batchOnce) {
            batchOnce = false;
            return true;
        } else
            return false;
    }
}

long Statistics::BatchSize() const {
    return batchSize;
}

template<class F>
void Statistics::SamplingLoop(const Point2i &pixel, F sampleOnce) {
    auto loop = [&]() { UpdateStats(pixel, sampleOnce()); };

    switch (mode) {
        case Mode::NORMAL:
            for (int i = 0; i < minSamples; ++i)
                loop();
            break;

        case Mode::TIME:
            for (int i = 0; i < batchSize; ++i)
                loop();
            break;

        case Mode::VARIANCE:
            for (int i = 0; i < minSamples; ++i)
                loop();
            while (!StopCriterion(pixel))
                loop();
            ReportStats(pixel);
            break;
    }
}

void Statistics::UpdateStats(const Point2i &pixel, const Spectrum &L) {
    if (!InsideExclusive(pixel, pixelBounds))
        return;

    Pixel &statsPixel = GetPixel(pixel);
    long &samples = statsPixel.samples;
    Spectrum &mean = statsPixel.mean;
    Spectrum &moment2 = statsPixel.moment2;

    samples++;
    Spectrum delta1 = L - mean;
    mean += delta1 / samples;
    Spectrum delta2 = L - mean;
    moment2 += delta1 * delta2;
}

Float Statistics::Sampling(const Pixel &statsPixel) const {
    return mode == Mode::VARIANCE
         ? static_cast<Float>(statsPixel.samples - minSamples) /
                   static_cast<Float>(maxSamples - minSamples)
         : static_cast<Float>(statsPixel.samples);
}

Spectrum Statistics::Variance(const Pixel &statsPixel) const {
    return statsPixel.samples > 1
         ? statsPixel.moment2 / (static_cast<Float>(statsPixel.samples) - 1)
         : 0;
}

bool Statistics::Converged(const Pixel &statsPixel) const {
    return Variance(statsPixel).y() < maxVariance;
}

bool Statistics::StopCriterion(const Point2i &pixel) const {
    if (!InsideExclusive(pixel, pixelBounds))
        return true;
    const Pixel &statsPixel = GetPixel(pixel);
    return statsPixel.samples >= maxSamples || Converged(statsPixel);
}

void Statistics::ReportStats(const Point2i &pixel) const {
    if (!InsideExclusive(pixel, pixelBounds))
        return;
    const Pixel &statsPixel = GetPixel(pixel);

    nPixels++;
    if (!Converged(statsPixel))
        nUnconvergedPixels++;
    ReportValue(samplesPerPixel, statsPixel.samples);
    ReportValue(variancePerPixel, Variance(statsPixel).y());
}

long Statistics::ElapsedMilliseconds() const {
    using namespace std::chrono;
    return duration_cast<milliseconds>(Clock::now() - startTime).count();
}

std::string Statistics::WorkTitle() const {
    std::string type = mode == Mode::TIME     ? "time"
                     : mode == Mode::VARIANCE ? "variance"
                     :                          "sampling";
    return "Rendering (equal " + type + ')';
}

long Statistics::UpdateWork() const {
    return mode == Mode::TIME ? 0 : 1;
}

Statistics::Pixel &Statistics::GetPixel(Point2i pixel) {
    const auto *that = this;
    return const_cast<Pixel &>(that->GetPixel(pixel));
}

const Statistics::Pixel &Statistics::GetPixel(Point2i pixel) const {
    CHECK(InsideExclusive(pixel, pixelBounds));
    int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
    int offset = (pixel.x - pixelBounds.pMin.x)
               + (pixel.y - pixelBounds.pMin.y) * width;
    return pixels[offset];
}

std::unique_ptr<Filter> Statistics::StatImagesFilter() {
    return std::unique_ptr<Filter>(new BoxFilter({0, 0}));
}

// PathIntegratorStats Method Definitions
PathIntegratorStats::PathIntegratorStats(Statistics stats, int maxDepth,
                                         std::shared_ptr<const Camera> camera,
                                         std::shared_ptr<Sampler> sampler,
                                         const Bounds2i &pixelBounds,
                                         Float rrThreshold,
                                         const std::string &lightSampleStrategy)
    : PathIntegrator(maxDepth, camera, sampler, pixelBounds, rrThreshold,
                     lightSampleStrategy),
      stats(std::move(stats)),
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

    stats.RenderBegin();
    ProgressReporter reporter(nTiles.x * nTiles.y, stats.WorkTitle());
    for (int batch = 0; stats.StartNextBatch(batch); ++batch) {
        ParallelFor2D([&](Point2i tile) {
            // Render section of image corresponding to _tile_

            // Allocate _MemoryArena_ for tile
            MemoryArena arena;

            // Get sampler instance for tile
            int seed = nTiles.x * nTiles.y * batch + nTiles.x * tile.y + tile.x;
            std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);

            // Compute sample bounds for tile
            int x0 = sampleBounds.pMin.x + tile.x * tileSize;
            int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
            int y0 = sampleBounds.pMin.y + tile.y * tileSize;
            int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
            Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
            LOG(INFO) << "Starting image tile " << tileBounds;

            // Get _FilmTile_ for tile
            auto filmTile = camera->film->GetFilmTile(tileBounds);

            // Loop over pixels in tile to render them
            for (Point2i pixel : tileBounds) {
                {
                    ProfilePhase pp(Prof::StartPixel);
                    tileSampler->StartPixel(pixel);
                    tileSampler->SetSampleNumber(batch * stats.BatchSize());
                }

                // Do this check after the StartPixel() call; this keeps
                // the usage of RNG values from (most) Samplers that use
                // RNGs consistent, which improves reproducability /
                // debugging.
                if (!InsideExclusive(pixel, pixelBounds))
                    continue;

                stats.SamplingLoop(pixel, [&]() {
                    // Initialize _CameraSample_ for current sample
                    CameraSample cameraSample =
                        tileSampler->GetCameraSample(pixel);

                    // Generate camera ray for current sample
                    RayDifferential ray;
                    Float rayWeight =
                        camera->GenerateRayDifferential(cameraSample, &ray);
                    ray.ScaleDifferentials(
                        1 / std::sqrt((Float) tileSampler->samplesPerPixel));
                    ++nCameraRays;

                    // Evaluate radiance along camera ray
                    Spectrum L(0.f);
                    if (rayWeight > 0)
                        L = Li(ray, scene, *tileSampler, arena, 0);

                    // Issue warning if unexpected radiance value returned
                    if (L.HasNaNs()) {
                        LOG(ERROR) << StringPrintf(
                            "Not-a-number radiance value returned for "
                            "pixel (%d, %d), sample %d. Setting to black.",
                            pixel.x, pixel.y,
                            (int) tileSampler->CurrentSampleNumber());
                        L = Spectrum(0.f);
                    } else if (L.y() < -1e-5) {
                        LOG(ERROR) << StringPrintf(
                            "Negative luminance value, %f, returned for "
                            "pixel (%d, %d), sample %d. Setting to black.",
                            L.y(), pixel.x, pixel.y,
                            (int) tileSampler->CurrentSampleNumber());
                        L = Spectrum(0.f);
                    } else if (std::isinf(L.y())) {
                        LOG(ERROR) << StringPrintf(
                            "Infinite luminance value returned for "
                            "pixel (%d, %d), sample %d. Setting to black.",
                            pixel.x, pixel.y,
                            (int) tileSampler->CurrentSampleNumber());
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
            reporter.Update(stats.UpdateWork());
        }, nTiles);
    }
    reporter.Done();
    stats.RenderEnd();
    LOG(INFO) << "Rendering finished";

    // Save final image after rendering
    camera->film->WriteImage();
    stats.WriteImages();
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

    return new PathIntegratorStats({params, camera->film, *sampler}, maxDepth,
                                   camera, sampler, pixelBounds, rrThreshold,
                                   lightStrategy);
}

// ShadowIntegratorStats Method Definitions
ShadowIntegratorStats::ShadowIntegratorStats(Statistics stats, int maxDepth,
                                             int maxSkips, Float skipProb,
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
      stats(std::move(stats)),
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

    stats.RenderBegin();
    ProgressReporter reporter(nTiles.x * nTiles.y, stats.WorkTitle());
    for (int batch = 0; stats.StartNextBatch(batch); ++batch) {
        ParallelFor2D([&](Point2i tile) {
            // Render section of image corresponding to _tile_

            // Allocate _MemoryArena_ for tile
            MemoryArena arena;

            // Get sampler instance for tile
            // A second sampler is needed to avoid patterns appearing due to
            // correlation between pixel position and caster skipping
            int seed = nTiles.x * nTiles.y * batch + nTiles.x * tile.y + tile.x;
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

            // Loop over pixels in tile to render them
            for (Point2i pixel : tileBounds) {
                {
                    ProfilePhase pp(Prof::StartPixel);
                    tileSampler->StartPixel(pixel);
                    skipSampler->StartPixel(pixel);
                    tileSampler->SetSampleNumber(batch * stats.BatchSize());
                    skipSampler->SetSampleNumber(batch * stats.BatchSize());
                }

                // Do this check after the BeginPixel() call; this keeps
                // the usage of RNG values from (most) Samplers that use
                // RNGs consistent, which improves reproducibility /
                // debugging.
                if (!InsideExclusive(pixel, pixelBounds))
                    continue;

                stats.SamplingLoop(pixel, [&]() {
                    // Initialize _CameraSample_ for current sample
                    CameraSample cameraSample =
                        tileSampler->GetCameraSample(pixel);

                    // Generate camera ray for current sample
                    RayDifferential ray;
                    Float rayWeight =
                        camera->GenerateRayDifferential(cameraSample, &ray);
                    ray.ScaleDifferentials(
                        1 / std::sqrt((Float) tileSampler->samplesPerPixel));
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
            reporter.Update(stats.UpdateWork());
        }, nTiles);
    }
    reporter.Done();
    stats.RenderEnd();
    LOG(INFO) << "Rendering finished";

    // Save final images after rendering
    GetLayers().WriteImage();
    stats.WriteImages();
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

    return new ShadowIntegratorStats({params, camera->film, *sampler},
                                     maxDepth, maxSkips, skipProb,
                                     std::move(camera), std::move(sampler),
                                     std::move(namedCasters),
                                     std::move(namedCatchers),
                                     std::move(noSelfShadow),
                                     singleFile, splitLights, splitDirect,
                                     pixelBounds, rrThreshold, lightStrategy);
}

}  // namespace pbrt
