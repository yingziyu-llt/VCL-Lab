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
    glm::vec3 PathTrace(const RayIntersector & intersector, Ray ray, float P_RR, bool enableShadow, bool enableCosweighted, bool enableLightWeighted, std::mt19937 & gen) {
        std::uniform_real_distribution<float> dist_real(0, 1);
        if (dist_real(gen) > P_RR) return glm::vec3(0); // 俄罗斯轮盘赌

        const float pi     = 3.1415926;
        auto        rayHit = intersector.IntersectRay(ray);
        if (rayHit.IntersectState) {
            const glm::vec3 pos      = rayHit.IntersectPosition;
            const glm::vec3 n        = rayHit.IntersectNormal;
            const glm::vec3 kd       = rayHit.IntersectAlbedo;
            const glm::vec3 ks       = rayHit.IntersectMetaSpec;
            const float     alpha    = rayHit.IntersectAlbedo.w;
            const float     metallic = rayHit.IntersectMetaSpec.r;
            float           factor   = 1;

            glm::vec3 dir;
            glm::vec3 L_e(0);
            if (enableLightWeighted) {
                std::uniform_int_distribution<int> dist_int(0, intersector.InternalScene->Lights.size() - 1);
                int                                lightIndex = dist_int(gen);

                auto light = intersector.InternalScene->Lights[lightIndex];
                if (light.Type == Engine::LightType::Point) {
                    // 点光源
                    dir                = glm::normalize(light.Position - pos);
                    factor             = (intersector.InternalScene->Lights.size() + 1); // 点光源的权重
                    glm::vec3 lightDir = glm::normalize(light.Position - pos);
                    glm::vec3 dis      = light.Position - pos;
                    if (enableShadow) {
                        Ray  shadowRay(pos, lightDir);
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        if (shadowHit.IntersectState) {
                            float lightDist = glm::length(dis);
                            float hitDist   = glm::length(shadowHit.IntersectPosition - pos);
                            if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f) factor = 0;
                        }
                    }
                    L_e = light.Intensity / glm::length(dis) / glm::length(dis) * BRDF(n, -ray.Direction, lightDir, kd, alpha, metallic) * glm::dot(n, -ray.Direction) * pi * factor;
                } else if (light.Type == Engine::LightType::Directional) {
                    // 方向光源
                    dir                = glm::normalize(light.Direction);
                    factor             = (intersector.InternalScene->Lights.size() + 1); // 方向光源的权重
                    glm::vec3 lightDir = glm::normalize(light.Direction);
                    if (enableShadow) {
                        Ray  shadowRay(pos, lightDir);
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        if (shadowHit.IntersectState) {
                            if (shadowHit.IntersectAlbedo.w >= 0.2f) factor = 0;
                        }
                    }
                    L_e = light.Intensity * BRDF(n, -ray.Direction, lightDir, kd, alpha, metallic) * glm::dot(n, -ray.Direction) * pi * factor;
                } else if (light.Type == Engine::LightType::Area) {
                    // 面光源
                    glm::vec3 edge1 = light.Position2 - light.Position;
                    glm::vec3 edge2 = light.Position3 - light.Position;
                    // 在面光源上均匀采样一个点
                    float u = dist_real(gen);
                    float v = dist_real(gen);
                    if (u + v > 1.0f) {
                        u = 1.0f - u;
                        v = 1.0f - v;
                    }

                    glm::vec3 lightPoint = light.Position + u * edge1 + v * edge2;
                    dir                  = glm::normalize(lightPoint - pos);
                    glm::vec3 n2         = glm::normalize(glm::cross(edge1, edge2));
                    factor               = glm::abs(glm::dot(dir, n)) * glm::abs(glm::dot(dir, n2)) / glm::dot(lightPoint - pos, lightPoint - pos) * glm::length(glm::cross(edge1,edge2)) / 2 * (intersector.InternalScene->Lights.size()); // 面光源的权重
                    if (enableShadow) {
                        Ray  shadowRay(pos, dir);
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        if (shadowHit.IntersectState) {
                            float lightDist = glm::length(lightPoint - pos);
                            float hitDist   = glm::length(shadowHit.IntersectPosition - pos);
                            if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f) factor = 0;
                        }
                    }
                    L_e = light.Intensity * glm::abs(glm::dot(dir, n)) * BRDF(n, -ray.Direction, dir, kd, alpha, metallic) * pi * factor;
                }
            }
            if (enableCosweighted) {
                float epsilon1 = dist_real(gen);
                float epsilon2 = dist_real(gen);
                float theta    = acos(sqrt(1.0f - epsilon1));
                float phi      = 2.0f * pi * epsilon2;
                dir            = glm::vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            } else {
                float theta = acos(2.0f * dist_real(gen) - 1.0f);
                float phi   = 2.0f * pi * dist_real(gen);
                dir         = glm::vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            }
            // 确保新方向与法线在同一半球
            if (glm::dot(dir, n) < 0) dir = -dir;

            Ray newRay(pos + n * 0.001f, dir); // 避免自交

            for (auto light : intersector.InternalScene->Lights) {
                if (light.Type == Engine::LightType::Point) {
                    glm::vec3 lightDir = glm::normalize(light.Position - pos);
                    glm::vec3 dis      = light.Position - pos;
                    if (glm::dot(lightDir, dir) > 0.9) {
                        if (enableShadow) {
                            Ray  shadowRay(pos, lightDir);
                            auto shadowHit = intersector.IntersectRay(shadowRay);
                            if (shadowHit.IntersectState) {
                                float lightDist = glm::length(dis);
                                float hitDist   = glm::length(shadowHit.IntersectPosition - pos);
                                if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f) continue;
                            }
                        }
                        if (enableCosweighted) return L_e + light.Intensity / glm::length(dis) / glm::length(dis) * BRDF(n, -ray.Direction, lightDir, kd, alpha, metallic) * pi / P_RR;
                        else return L_e + light.Intensity / glm::length(dis) / glm::length(dis) * BRDF(n, -ray.Direction, lightDir, kd, alpha, metallic) * glm::dot(-ray.Direction, n) * pi / P_RR;
                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    glm::vec3 lightDir = glm::normalize(light.Direction);
                    if (glm::dot(lightDir, dir) > 0.9) {
                        if (enableShadow) {
                            Ray  shadowRay(pos, lightDir);
                            auto shadowHit = intersector.IntersectRay(shadowRay);
                            if (shadowHit.IntersectState) {
                                if (shadowHit.IntersectAlbedo.w >= 0.2f) continue;
                            }
                        }
                        if (enableCosweighted) return L_e + light.Intensity * BRDF(n, -ray.Direction, lightDir, kd, alpha, metallic) * pi / P_RR;
                        else
                            return L_e + light.Intensity * BRDF(n, dir, lightDir, kd, alpha, metallic) * glm::dot(n, -ray.Direction) * pi / P_RR;
                    }
                } else if (light.Type == Engine::LightType::Area) {
                    glm::vec3    p1       = light.Position;
                    glm::vec3    p2       = light.Position2;
                    glm::vec3    p3       = light.Position3;
                    glm::vec3    lightDir = glm::normalize(glm::cross(p2 - p1, p3 - p1));
                    Intersection inter;
                    if (IntersectTriangle(inter, newRay, p1, p2, p3)) {
                        if (enableShadow) {
                            Ray  shadowRay(pos, dir);
                            auto shadowHit = intersector.IntersectRay(shadowRay);
                            if (shadowHit.IntersectState) {
                                float lightDist = glm::length(lightDir);
                                float hitDist   = glm::length(shadowHit.IntersectPosition - pos);
                                if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f) continue;
                            }
                        }
                        if (enableCosweighted)
                            return L_e + light.Intensity * glm::abs(glm::dot(lightDir, newRay.Direction)) * BRDF(n, -ray.Direction, lightDir, kd, alpha, metallic) * pi / P_RR;
                        else
                            return L_e + light.Intensity * glm::abs(glm::dot(lightDir, newRay.Direction)) * BRDF(n, -ray.Direction, lightDir, kd, alpha, metallic) * glm::dot(n, -ray.Direction) * pi / P_RR;
                    }
                }

                if (enableCosweighted)
                    return L_e + PathTrace(intersector, newRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen) * BRDF(n, -ray.Direction, dir, kd, alpha, metallic) * pi / P_RR;
                else return L_e + PathTrace(intersector, newRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen) * BRDF(n, -ray.Direction, dir, kd, alpha, metallic) * glm::dot(n, -ray.Direction) * pi / P_RR;
            }
        }
        return glm::vec3(0); // 背景色
    }
} // namespace VCX::Labs::Rendering