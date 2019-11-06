#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_SHADOW_H
#define PBRT_INTEGRATORS_SHADOW_H

// integrators/shadow.h*
#include "pbrt.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "shape.h"
#include "layeredfilm.h"
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace pbrt {

// _ShadowObject_ identifies an object from its surface. More precisely, it is
// combined with and _Identifier_ material associated to a unique index.
struct ShadowObject {
    ShadowObject() = default;
    explicit ShadowObject(std::size_t id);
    explicit ShadowObject(const SurfaceInteraction &isect);

    bool defined = false;
    std::size_t identifier = 0;

    friend bool operator==(ShadowObject lhs, ShadowObject rhs);
    friend bool operator!=(ShadowObject lhs, ShadowObject rhs);
    explicit operator bool() const;
};

// A _ShadowSpectrum_ represents a _Spectrum_ value that may have been occluded.
// It contains a pointer to the _Light_ that emitted radiance, and a pair of
// occluding _ShadowObject_ and _Spectrum_ for both light L and importance W.
struct ShadowSpectrum {
    const Light *light;
    ShadowObject LOccluder;
    Spectrum LL;
    ShadowObject WOccluder;
    Spectrum WL;

    friend ShadowSpectrum operator*(Float x, ShadowSpectrum &&S);
    friend ShadowSpectrum operator/(ShadowSpectrum &&S, Float x);
    friend ShadowSpectrum operator*(const Spectrum &L, ShadowSpectrum &&S);
};

// An enumeration to separate between direct or indirect shadows optionally
enum class ShadowOrder {DIRECT, INDIRECT, ALL};

// The key used to identify a given shadow layer
struct ShadowLayer {
    ShadowObject caster;
    const Light *light;
    ShadowOrder order;

    friend bool operator==(const ShadowLayer &lhs, const ShadowLayer &rhs);
};

}  // namespace pbrt


// Utility to combine multiple hashing functions
inline std::size_t HashCombine(std::size_t seed) { return seed; }

template<class T, class... Rest>
inline std::size_t HashCombine(std::size_t seed, const T &value, Rest... rest) {
    auto shift = 0x9e3779b9 + (seed << 6u) + (seed >> 2u);
    return HashCombine(seed ^ (std::hash<T>()(value) + shift), rest...);
}

// Specialize std::hash for the custom classes above
template<>
struct std::hash<pbrt::ShadowObject> {
    std::size_t operator()(pbrt::ShadowObject const& object) const {
        return object.identifier;
    };
};

template<>
struct std::hash<pbrt::ShadowLayer> {
    std::size_t operator()(const pbrt::ShadowLayer &key) const {
        return HashCombine(0, key.caster, key.light, key.order);;
    }
};


namespace pbrt {

// ShadowIntegrator Declarations
class ShadowIntegrator : public Integrator {
  public:
    using Layers = LayeredFilm<ShadowLayer>;
    using TileSamples = decltype(Layers::LayeredTile::samples);
    using EncounterMap = std::unordered_map<ShadowObject, bool>;
    using NamedObjects = std::unordered_map<ShadowObject, std::string>;

  public:
    // ShadowIntegrator Public Methods
    ShadowIntegrator(int maxDepth, int maxSkips, Float skipProb,
                     std::shared_ptr<const Camera> camera,
                     std::shared_ptr<Sampler> sampler,
                     NamedObjects namedCasters,
                     NamedObjects namedCatchers,
                     NamedObjects noSelfShadow,
                     bool singleFile, bool splitLights, bool splitDirect,
                     const Bounds2i &pixelBounds, Float rrThreshold = 1,
                     std::string lightSampleStrategy = "spatial");

    void Preprocess(const Scene &scene);
    void Render(const Scene &scene) override;
    Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, Sampler &skipSampler, MemoryArena &arena,
                TileSamples &samples, int depth = 0) const;

    Layers &GetLayers();

    // Essentially _UniformSampleOneLight_ but accounting for possible occlusion
    ShadowSpectrum SampleLight(const Interaction &it, const Scene &scene,
                               MemoryArena &arena, Sampler &sampler,
                               bool alteredPropagation,
                               ShadowObject assignedCaster,
                               const EncounterMap &hasEncountered,
                               bool handleMedia = false, const Distribution1D
                               *lightDistrib = nullptr) const;

    // Essentially _EstimateDirect_ but accounting for possible occlusion
    ShadowSpectrum DirectLight(const Interaction &it, const Point2f &uShading,
                               const Light &light, const Point2f &uLight,
                               const Scene &scene, Sampler &sampler,
                               MemoryArena &arena, bool alteredPropagation,
                               ShadowObject assignedCaster,
                               const EncounterMap &hasEncountered,
                               bool handleMedia = false) const;

  private:
    // ShadowIntegrator Private Methods
    bool IsCaster(ShadowObject object) const;
    bool IsCatcher(ShadowObject object) const;
    bool NoSelfShadow(ShadowObject object) const;

    // ShadowIntegrator Private Data
    const std::shared_ptr<const Camera> camera;
    const std::shared_ptr<Sampler> sampler;

    const Bounds2i pixelBounds;
    const int maxDepth;
    const int maxSkips;
    const Float skipProb;
    const Float rrThreshold;
    const std::string lightSampleStrategy;
    std::unique_ptr<LightDistribution> lightDistrib;

    // Keep track of the specified objects
    const NamedObjects casters;
    const NamedObjects catchers;
    const NamedObjects noSelfShadow;
    const EncounterMap noneEncountered;

    // Manage the different layers
    const bool singleFile;
    const bool splitLights;
    const bool splitDirect;
    Layers layers;

    static const ShadowLayer mainLayer;
    static EncounterMap defaultEncounterMap(const NamedObjects &casters);
};

ShadowIntegrator *CreateShadowIntegrator(const ParamSet &params,
                                         std::shared_ptr<Sampler> sampler,
                                         std::shared_ptr<const Camera> camera);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_SHADOW_H
