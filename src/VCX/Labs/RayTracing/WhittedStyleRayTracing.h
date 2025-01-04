#pragma once

#include <numeric>
#include <spdlog/spdlog.h>

#include "Engine/Scene.h"
#include "Labs/RayTracing/Ray.h"
#include "Labs/RayTracing/RayIntersector.h"

namespace VCX::Labs::Rendering {
    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);

} // namespace VCX::Labs::Rendering
