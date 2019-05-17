#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_MATERIALS_IDENTIFIER_H
#define PBRT_MATERIALS_IDENTIFIER_H

// materials/identifier.h*
#include "pbrt.h"
#include "material.h"

namespace pbrt {

class IdentifierMaterial : public Material {
  public:
    IdentifierMaterial(const std::string &id, std::shared_ptr<Material> mat);

    void ComputeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                    TransportMode mode,
                                    bool allowMultipleLobes) const override;
    std::size_t Identifier() const override;

  private:
    const std::size_t identifier;
    const std::shared_ptr<Material> material;
};

}  // namespace pbrt

#endif  // PBRT_MATERIALS_IDENTIFIER_H
