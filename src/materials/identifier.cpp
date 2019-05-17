// materials/identifier.cpp*
#include "identifier.h"
#include "interaction.h"

#include <functional>

namespace pbrt {

IdentifierMaterial::IdentifierMaterial(const std::string &id,
                                       std::shared_ptr<Material> mat)
    : identifier(std::hash<std::string>()(id)),
      material(std::move(mat)) {}

void
IdentifierMaterial::ComputeScatteringFunctions(SurfaceInteraction *si,
                                               MemoryArena &arena,
                                               TransportMode mode,
                                               bool allowMultipleLobes) const {
    if (material)
        material->ComputeScatteringFunctions(si, arena, mode,
                                             allowMultipleLobes);
}

std::size_t IdentifierMaterial::Identifier() const {
    return identifier;
}

}  // namespace pbrt
