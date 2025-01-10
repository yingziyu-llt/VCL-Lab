#include "Labs/RayTracing/PathTracing.h"

namespace VCX::Labs::Rendering {

    glm::vec3 BRDF(const glm::vec3 & normal, const glm::vec3 & viewDir, const glm::vec3 & lightDir, const glm::vec3 & albedo, float roughness, float metallic) {
        glm::vec3 halfVector = glm::normalize(lightDir + viewDir);
        float     NdotL      = glm::max(glm::dot(normal, lightDir), 0.0f);
        float     NdotV      = glm::max(glm::dot(normal, viewDir), 0.0f);
        float     NdotH      = glm::max(glm::dot(normal, halfVector), 0.0f);
        float     VdotH      = glm::max(glm::dot(viewDir, halfVector), 0.0f);

        float alpha = roughness * roughness;

        // Distribution function (GGX)
        float alpha2 = alpha * alpha;
        float denom  = (NdotH * NdotH * (alpha2 - 1.0f) + 1.0f);
        float D      = alpha2 / (glm::pi<float>() * denom * denom);

        // Fresnel-Schlick approximation
        glm::vec3 F0 = glm::mix(glm::vec3(0.04f), albedo, metallic);
        glm::vec3 F  = F0 + (1.0f - F0) * glm::pow(1.0f - VdotH, 5.0f);

        // Geometry function (Smith)
        float k          = alpha / 2.0f;
        float G_SchlickV = NdotV / (NdotV * (1.0f - k) + k);
        float G_SchlickL = NdotL / (NdotL * (1.0f - k) + k);
        float G          = G_SchlickV * G_SchlickL;

        glm::vec3 numerator   = D * F * G;
        float     denominator = 4.0f * NdotV * NdotL + 0.001f; // Prevent division by zero
        glm::vec3 specular    = numerator / denominator;

        glm::vec3 kS = F;
        glm::vec3 kD = glm::vec3(1.0f) - kS;
        kD *= 1.0f - metallic;

        return (kD * albedo) + specular; // 移除除以 pi
    }
    glm::vec3 PathTrace(const RayIntersector & intersector, Ray ray, float P_RR, bool enableShadow, std::mt19937 & gen) {
        std::uniform_real_distribution<float> dist_real(0, 1);
        if (dist_real(gen) > P_RR) return glm::vec3(0); // 俄罗斯轮盘赌

        const float pi = 3.1415926;

        // 处理直接光照
        for (auto light : intersector.InternalScene->Lights) {
            if (light.Type == Engine::LightType::Point) {
                glm::vec3 toLight   = light.Position - ray.Origin;
                float     lightDist = glm::length(toLight);
                toLight             = glm::normalize(toLight);

                if (glm::dot(toLight, ray.Direction) >= 0.99f) {
                    if (enableShadow) {
                        Ray  shadowRay(ray.Origin, toLight); // 避免自交
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        if (shadowHit.IntersectState) {
                            float hitDist = glm::length(shadowHit.IntersectPosition - ray.Origin);
                            if (hitDist <= lightDist) continue; // 光线被遮挡
                        }
                    }
                    return light.Intensity / (lightDist * lightDist);
                }
            }
        }

        // 处理间接光照
        auto rayHit = intersector.IntersectRay(ray);
        if (rayHit.IntersectState) {
            const glm::vec3 pos      = rayHit.IntersectPosition;
            const glm::vec3 n        = rayHit.IntersectNormal;
            const glm::vec3 kd       = rayHit.IntersectAlbedo;
            const glm::vec3 ks       = rayHit.IntersectMetaSpec;
            const float     alpha    = rayHit.IntersectAlbedo.w;
            const float     metallic = rayHit.IntersectMetaSpec.r;

            // 使用余弦加权的重要性采样生成新的光线方向
            float     r1 = dist_real(gen), r2 = dist_real(gen);
            float     phi      = 2 * pi * r1;
            float     cosTheta = sqrt((1 - r2) / (1 + (alpha * alpha - 1) * r2));
            float     sinTheta = sqrt(1 - cosTheta * cosTheta);
            glm::vec3 dir      = glm::normalize(glm::vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta));

            // 确保新方向与法线在同一半球
            if (glm::dot(dir, n) < 0) dir = -dir;

            Ray newRay(pos + n * 0.001f, dir); // 避免自交
            return PathTrace(intersector, newRay, P_RR, enableShadow, gen) * BRDF(n, -ray.Direction, dir, kd, alpha, metallic) * pi / P_RR;
        }

        return glm::vec3(0); // 背景色
    }
} // namespace VCX::Labs::Rendering