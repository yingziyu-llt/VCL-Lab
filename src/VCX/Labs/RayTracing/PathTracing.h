#pragma once

#include <numeric>
#include <spdlog/spdlog.h>
#include <random>

#include "Engine/Scene.h"
#include "Labs/RayTracing/Ray.h"
#include "Labs/RayTracing/RayIntersector.h"

namespace VCX::Labs::Rendering {
    glm::vec3 PathTrace(const RayIntersector & intersector, Ray ray, float p_rr, bool enableShadow,bool enableCosweighted,bool,std::mt19937& gen,int depth);
    glm::vec3 BRDF(const glm::vec3 &normal, const glm::vec3 &viewDir, const glm::vec3 &lightDir,  const glm::vec3 &kd, const glm::vec3 &ks, float shininess);
    glm::vec3 BRDFWeightedSample(const glm::vec3 & normal, const glm::vec3 & viewDir, float shininess, std::mt19937 & gen) ;
} // namespace VCX::Labs::Rendering
