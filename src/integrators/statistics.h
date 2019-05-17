#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_STATISTICS_H
#define PBRT_INTEGRATORS_STATISTICS_H

// integrators/statistics.h*
#include "pbrt.h"
#include "layeredfilm.h"
#include "memory.h"
#include "path.h"
#include "shadow.h"
#include "shape.h"

#include <chrono>
#include <functional>

namespace pbrt {

class IntegratorStats {
    enum class Mode {NORMAL, TIME, VARIANCE};
    enum class Stats {SPP, MEAN, M2};

    using Clock = std::chrono::steady_clock;
    using Images = LayeredFilm<Stats>;
    using Tile = LayeredFilm<Stats>::LayeredTile;
    using SamplingFunctor = std::function<Spectrum()>;

  public:
    IntegratorStats(const ParamSet &params, Film *const film,
                    const Sampler &sampler);

    void RenderBegin();
    void RenderEnd() const;
    bool RenderBatch() const;

    void SamplesLoop(const Point2i &pixel, Tile &tile,
                     SamplingFunctor sampleOnce) const;

    Images &StatsImages();

  private:
    Float &Samples(const Point2i &pixel, Tile &tile) const;
    Spectrum &Mean(const Point2i &pixel, Tile &tile) const;
    Spectrum &Moment2(const Point2i &pixel, Tile &tile) const;

    void UpdateStats(const Point2i &pixel, Tile &tile, const Spectrum &L) const;

    Spectrum Variance(const Point2i &pixel, Tile &tile) const;
    bool Converged(const Point2i &pixel, Tile &tile) const;
    bool StopSampling(const Point2i &pixel, Tile &tile) const;
    void ReportStats(const Point2i &pixel, Tile &tile) const;

    const Mode mode;
    const long batchSize;
    const long minSamples;
    const long maxSamples;
    const int maxSeconds;
    const Float maxVariance;
    const Bounds2i bounds;

    std::chrono::time_point<Clock> startTime;
    Images statsImages;
};

// PathIntegratorStats Declarations
class PathIntegratorStats : public PathIntegrator, IntegratorStats {
  public:
    // PathIntegratorStats Public Methods
    PathIntegratorStats(const ParamSet &params,
                        int maxDepth, std::shared_ptr<const Camera> camera,
                        std::shared_ptr<Sampler> sampler,
                        const Bounds2i &pixelBounds, Float rrThreshold = 1,
                        const std::string &lightSampleStrategy = "spatial");

    void Render(const Scene &scene) override;

  private:
    // PathIntegratorStats Private Data
    const std::shared_ptr<Sampler> sampler;
    const Bounds2i pixelBounds;
};

PathIntegratorStats *CreatePathIntegratorStats(
    const ParamSet &params,
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera
);

// ShadowIntegratorStats Declarations
class ShadowIntegratorStats : public ShadowIntegrator, IntegratorStats {
    using LayeredTile = LayeredFilm<ShadowLayer>::LayeredTile;

  public:
    // ShadowIntegratorStats Public Methods
    ShadowIntegratorStats(const ParamSet &params,
                          int maxDepth, int maxSkips, Float skipProb,
                          std::shared_ptr<const Camera> cam,
                          std::shared_ptr<Sampler> sampler,
                          NamedObjects namedCasters,
                          NamedObjects namedCatchers,
                          NamedObjects noSelfShadow,
                          bool singleFile, bool splitLights, bool splitDirect,
                          const Bounds2i &pixelBounds, Float rrThreshold = 1,
                          const std::string &lightSampleStrategy = "spatial");

    void Render(const Scene &scene) override;

  private:
    // ShadowIntegratorStats Private Data
    const std::shared_ptr<const Camera> camera;
    const std::shared_ptr<Sampler> sampler;
    const Bounds2i pixelBounds;
};

ShadowIntegratorStats *CreateShadowIntegratorStats(
    const ParamSet &params,
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera
);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_STATISTICS_H
