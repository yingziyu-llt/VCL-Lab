#pragma once

#include <numeric>
#include <spdlog/spdlog.h>
#include <random>

#include "Engine/Scene.h"
#include "Labs/RayTracing/Ray.h"
#include "Labs/RayTracing/RayIntersector.h"

namespace VCX::Labs::Rendering {
    glm::vec3 PathTrace(const RayIntersector & intersector, Ray ray, float p_rr, bool enableShadow,bool enableCosweighted,bool,std::mt19937& gen);
    glm::vec3 BRDF(const glm::vec3 & normal, const glm::vec3 & viewDir, const glm::vec3 & lightDir, const glm::vec3 & albedo, float roughness, float metallic) ;
} // namespace VCX::Labs::Rendering
