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

class Statistics {
    enum class Mode {NORMAL, TIME, VARIANCE};
    using Clock = std::chrono::steady_clock;

public:
    struct Pixel {
        long samples = 0;
        Spectrum mean = 0;
        Spectrum moment2 = 0;
    };

  public:
    Statistics(const ParamSet &params, const Film *film,
               const Sampler &sampler);

    void RenderBegin();
    void RenderEnd() const;
    void WriteImages() const;

    bool StartNextBatch(int index);
    long BatchSize() const;

    template<class F>
    void SamplingLoop(const Point2i &pixel, F sampleOnce);

    std::string WorkTitle() const;
    long UpdateWork() const;

  private:
    void UpdateStats(const Point2i &pixel, const Spectrum &L);
    void ReportStats(const Point2i &pixel) const;

    Float Sampling(const Pixel &statsPixel) const;
    Spectrum Variance(const Pixel &statsPixel) const;
    bool Converged(const Pixel &statsPixel) const;
    bool StopCriterion(const Point2i &pixel) const;
    long ElapsedMilliseconds() const;

    const Mode mode;
    const long maxSamples;
    const long minSamples;
    const Float maxVariance;
    const long batchSize;
    const int maxSeconds;

    const Film *originalFilm;
    const Bounds2i pixelBounds;
    std::unique_ptr<Pixel[]> pixels;
    Clock::time_point startTime;
    bool batchOnce = true;

    Pixel &GetPixel(Point2i pixel);
    const Pixel &GetPixel(Point2i pixel) const;
    static std::unique_ptr<Filter> StatImagesFilter();
};

// PathIntegratorStats Declarations
class PathIntegratorStats : public PathIntegrator {
  public:
    // PathIntegratorStats Public Methods
    PathIntegratorStats(Statistics stats, int maxDepth,
                        std::shared_ptr<const Camera> camera,
                        std::shared_ptr<Sampler> sampler,
                        const Bounds2i &pixelBounds, Float rrThreshold = 1,
                        const std::string &lightSampleStrategy = "spatial");

    void Render(const Scene &scene) override;

  private:
    // PathIntegratorStats Private Data
    Statistics stats;
    const std::shared_ptr<Sampler> sampler;
    const Bounds2i pixelBounds;
};

PathIntegratorStats *CreatePathIntegratorStats(
    const ParamSet &params,
    std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera
);

// ShadowIntegratorStats Declarations
class ShadowIntegratorStats : public ShadowIntegrator {
    using LayeredTile = LayeredFilm<ShadowLayer>::LayeredTile;

  public:
    // ShadowIntegratorStats Public Methods
    ShadowIntegratorStats(Statistics stats, int maxDepth,
                          int maxSkips, Float skipProb,
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
    Statistics stats;
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
