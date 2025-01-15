#include "Labs/RayTracing/PathTracing.h"

namespace VCX::Labs::Rendering {
    glm::vec3 BRDF(const glm::vec3 & normal, const glm::vec3 & viewDir, const glm::vec3 & lightDir, const glm::vec3 & kd, const glm::vec3 & ks, float shininess) {
        glm::vec3 h = glm::normalize(viewDir + lightDir);
        return glm::max(glm::dot(glm::normalize(lightDir), normal), 0.0f) * kd + glm::max(glm::pow(glm::max(glm::dot(h, normal), 0.0f), shininess), 0.0f) * ks;
    }
    glm::vec3 BRDFWeightedSample(const glm::vec3& normal, const glm::vec3& viewDir, float shininess, std::mt19937& gen) {
    std::uniform_real_distribution<float> dist(0, 1);
    float u = dist(gen);
    float v = dist(gen);

    // 根据 Phong BRDF 的分布生成采样方向
    float phi = 2.0f * glm::pi<float>() * u;
    float theta = acos(pow(v, 1.0f / (shininess + 1.0f)));

    // 转换为笛卡尔坐标
    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);

    // 构建局部坐标系
    glm::vec3 reflectDir = glm::reflect(-viewDir, normal);
    glm::vec3 tangent = glm::normalize(glm::cross(reflectDir, glm::vec3(0, 1, 0)));
    if (glm::length(tangent) < 1e-5f) {
        tangent = glm::normalize(glm::cross(reflectDir, glm::vec3(1, 0, 0)));
    }
    glm::vec3 bitangent = glm::cross(reflectDir, tangent);

    // 将采样方向转换到世界坐标系
    return glm::normalize(tangent * x + bitangent * y + reflectDir * z);
}

    glm::vec3 PathTrace(const RayIntersector & intersector, Ray ray, float P_RR, bool enableShadow, bool enableCosweighted, bool enableLightWeighted, std::mt19937 & gen,int depth) {
        std::uniform_real_distribution<float> dist_real(0, 1);
        float f = depth > 3 ? 1.0f : P_RR;
        if (dist_real(gen) > P_RR && depth > 3) return glm::vec3(0); // 俄罗斯轮盘赌

        const float pi     = 3.1415926;
        auto        rayHit = intersector.IntersectRay(ray);
        if (rayHit.IntersectState) {
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;
            float           factor    = 1;
            if (rayHit.IntersectMode == Engine::BlendMode::Transparent) {
                // Glass ball handling
                float     ior     = 1.5f; // Index of refraction for glass
                bool      outside = glm::dot(ray.Direction, n) < 0;
                glm::vec3 bias    = 0.001f * n;

                // Adjust normal and compute refraction ratio
                glm::vec3 adjustedN = outside ? n : -n;
                float     eta       = outside ? 1.0f / ior : ior;

                // Calculate Fresnel effect
                float cosTheta                = glm::clamp(glm::dot(-ray.Direction, adjustedN), -1.0f, 1.0f);
                float sinTheta                = sqrt(1.0f - cosTheta * cosTheta);
                bool  totalInternalReflection = eta * sinTheta > 1.0f;

                glm::vec3 reflectDir = glm::reflect(ray.Direction, adjustedN);
                glm::vec3 refractDir;

                float reflectance = 1.0f; // Default to total internal reflection

                if (! totalInternalReflection) {
                    refractDir  = glm::refract(ray.Direction, adjustedN, eta);
                    float r0    = (1 - ior) / (1 + ior);
                    r0          = r0 * r0;
                    reflectance = r0 + (1 - r0) * pow(1 - cosTheta, 5);
                }

                Ray       reflectRay(outside ? pos + bias : pos - bias, reflectDir);
                glm::vec3 reflectionColor = PathTrace(intersector, reflectRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen,depth + 1);

                glm::vec3 refractionColor(0);
                if (! totalInternalReflection) {
                    Ray refractRay(outside ? pos - bias : pos + bias, refractDir);
                    refractionColor = PathTrace(intersector, refractRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen,depth + 1);
                }
                return reflectionColor * reflectance + refractionColor * (1 - reflectance);
            }

            glm::vec3 L_direct(0);
            if (enableLightWeighted) {
                glm::vec3                          dir;
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
                            if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f && rayHit.IntersectMode == Engine::BlendMode::Opaque) factor = 0;
                        }
                    }
                    L_direct = light.Intensity / glm::length(dis) / glm::length(dis) * BRDF(n, -ray.Direction, lightDir, kd, ks, shininess) * glm::dot(n, -ray.Direction) * pi * factor;
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
                    L_direct = light.Intensity * BRDF(n, -ray.Direction, lightDir, kd, ks, shininess) * glm::dot(n, -ray.Direction) * pi * factor;
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
                    factor               = glm::abs(glm::dot(dir, n)) * glm::abs(glm::dot(dir, n2)) / glm::dot(lightPoint - pos, lightPoint - pos) * glm::length(glm::cross(edge1, edge2)) / 2; // 面光源的权重
                    if (enableShadow) {
                        Ray  shadowRay(pos, dir);
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        if (shadowHit.IntersectState) {
                            float lightDist = glm::length(lightPoint - pos);
                            float hitDist   = glm::length(shadowHit.IntersectPosition - pos);
                            if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f) factor = 0;
                        }
                    }
                    L_direct = light.Intensity * glm::abs(glm::dot(dir, n)) * BRDF(n, -ray.Direction, dir, kd, ks, shininess) * pi * factor;
                }
            }

            glm::vec3 L_indirect(0);
            glm::vec3 dir;
            if (enableCosweighted) {
                float epsilon1 = dist_real(gen);
                float epsilon2 = dist_real(gen);
                float theta    = acos(sqrt(1.0f - epsilon1));
                float phi      = 2.0f * pi * epsilon2;
                dir            = glm::vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            } else {
                dir         = BRDFWeightedSample(n, -ray.Direction, shininess, gen);
            }

            Ray newRay(pos + dir * 1e-5f, dir);
            L_indirect = PathTrace(intersector, newRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen,depth) * BRDF(n, -ray.Direction, dir, kd, ks, shininess) * glm::dot(n, dir) * pi;
            return (L_direct + L_indirect) / f;
        }
        return glm::vec3(0); // 背景色
    }
} // namespace VCX::Labs::Rendering