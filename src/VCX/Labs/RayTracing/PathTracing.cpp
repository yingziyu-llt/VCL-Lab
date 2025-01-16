#include "Labs/RayTracing/PathTracing.h"
#include<assert.h>
namespace VCX::Labs::Rendering {
    glm::vec3 BRDF(const glm::vec3 & normal, const glm::vec3 & viewDir, const glm::vec3 & lightDir, const glm::vec3 & kd, const glm::vec3 & ks, float shininess) {
        glm::vec3 h = glm::normalize(viewDir + lightDir);
        return glm::max(glm::dot(glm::normalize(lightDir), normal), 0.0f) * kd + glm::max(glm::pow(glm::max(glm::dot(h, normal), 0.0f), shininess), 0.0f) * ks;
    }
    glm::vec3 BRDFWeightedSample(const glm::vec3 & normal, const glm::vec3 & viewDir, float shininess, std::mt19937 & gen) {
        std::uniform_real_distribution<float> dist(0, 1);
        float                                 u = dist(gen);
        float                                 v = dist(gen);
        float phi   = 2.0f * glm::pi<float>() * u;
        float theta = acos(pow(v, 1.0f / (shininess + 1.0f)));
        float x = sin(theta) * cos(phi);
        float y = sin(theta) * sin(phi);
        float z = cos(theta);
        glm::vec3 reflectDir = glm::reflect(-viewDir, normal);
        glm::vec3 tangent    = glm::normalize(glm::cross(reflectDir, glm::vec3(0, 1, 0)));
        if (glm::length(tangent) < 1e-5f) {
            tangent = glm::normalize(glm::cross(reflectDir, glm::vec3(1, 0, 0)));
        }
        glm::vec3 bitangent = glm::cross(reflectDir, tangent);
        return glm::normalize(tangent * x + bitangent * y + reflectDir * z);
    }

    glm::vec3 PathTrace(const RayIntersector & intersector, Ray ray, float P_RR, bool enableShadow, bool enableCosweighted, bool enableLightWeighted, std::mt19937 & gen, int depth) {
        std::uniform_real_distribution<float> dist_real(0, 1);
        float                                 f = depth > 3 ? 1.0f : P_RR;
        if (dist_real(gen) > P_RR && depth > 3) return glm::vec3(0); // 俄罗斯轮盘赌

        const float pi     = 3.1415926;
        RayHit      rayHit = intersector.IntersectRay(ray);
        if (rayHit.IntersectState) {
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;
            float           factor    = 1;
            glm::vec3       Le        = rayHit.IntersectEmission;
            if (rayHit.IntersectMode == Engine::BlendMode::Transparent) {
                Ray   reflRay(pos + ray.Direction * 1e-5f, ray.Direction - 2.0f * glm::dot(ray.Direction, n) * n);
                bool  into = glm::dot(n, ray.Direction) < 0;
                float nc   = 1.0f;
                float nt   = 1.5f;
                float nnt  = into ? nc / nt : nt / nc;
                float ddn  = glm::dot(ray.Direction, n);
                float cos2t;
                if ((cos2t = 1.0f - nnt * nnt * (1.0f - ddn * ddn)) < 0.0f) {
                    return Le + PathTrace(intersector, reflRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen, depth + 1);
                }
                glm::vec3 tdir = glm::normalize(ray.Direction * nnt - n * ((into ? 1 : -1) * (ddn * nnt + (float) sqrt(cos2t))));
                float     a    = nt - nc;
                float     b    = nt + nc;
                float     R0   = a * a / (b * b);
                float     c    = 1 - (into ? -ddn : glm::dot(tdir, n));
                float     Re   = R0 + (1 - R0) * c * c * c * c * c;
                float     Tr   = 1 - Re;
                float     P    = 0.25f + 0.5f * Re;
                float     RP   = Re / P;
                float     TP   = Tr / (1 - P);
                if (depth > 2) {
                    if (dist_real(gen) < P)
                        return Le + PathTrace(intersector, reflRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen, depth + 1) * RP;
                    else
                        return Le + PathTrace(intersector, Ray(pos, tdir), P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen, depth + 1) * TP;
                } else
                    return Le + PathTrace(intersector, reflRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen, depth + 1) * Re + PathTrace(intersector, Ray(pos, tdir), P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen, depth + 1) * Tr;
            }

            glm::vec3 L_direct(0);
            if (enableLightWeighted) {
                for (auto light : intersector.InternalScene->Lights) {
                    glm::vec3 dir(0);
                    if (light.Type == Engine::LightType::Point) {
                        // 点光源
                        dir                = glm::normalize(light.Position - pos);
                        factor             = 1; // 点光源的权重
                        glm::vec3 dis      = light.Position - pos;
                        if (enableShadow) {
                            Ray  shadowRay(pos + dir * 1e-2f, dir);
                            auto shadowHit = intersector.IntersectRay(shadowRay);
                            if (shadowHit.IntersectState) {
                                float lightDist = glm::length(dis);
                                float hitDist   = glm::length(shadowHit.IntersectPosition - pos);
                                if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f && rayHit.IntersectMode == Engine::BlendMode::Opaque) factor = 0;
                            }
                        }
                        L_direct += light.Intensity / glm::length(dis) / glm::length(dis) * BRDF(n, -ray.Direction, dir, kd, ks, shininess) * glm::dot(n, -ray.Direction) * pi * factor;
                    } else if (light.Type == Engine::LightType::Directional) {
                        // 方向光源
                        dir                = glm::normalize(light.Direction);
                        factor             = 1; // 方向光源的权重
                        if (enableShadow) {
                            Ray  shadowRay(pos + dir * 1e-2f, dir);
                            auto shadowHit = intersector.IntersectRay(shadowRay);
                            if (shadowHit.IntersectState) {
                                if (shadowHit.IntersectAlbedo.w >= 0.2f) factor = 0;
                            }
                        }
                        L_direct += light.Intensity * BRDF(n, -ray.Direction, dir, kd, ks, shininess) * glm::dot(n, -ray.Direction) * pi * factor;
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
                            if (hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f && shadowHit.IntersectEmission == glm::vec3(0)) factor = 0;
                        }
                    }
                    L_direct = light.Intensity * glm::abs(glm::dot(dir, n)) * BRDF(n, -ray.Direction, dir, kd, ks, shininess) * pi * factor;
                }

                }
            }

            glm::vec3 L_indirect(0);
            glm::vec3 dir;
            if (enableCosweighted) {
                float theta,phi;
                theta = dist_real(gen);
                phi = dist_real(gen);
                dir = glm::vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            } else {
                dir = BRDFWeightedSample(n, -ray.Direction, shininess, gen);
            }

            Ray newRay(pos + dir * 1e-5f, dir);
            L_indirect = PathTrace(intersector, newRay, P_RR, enableShadow, enableCosweighted, enableLightWeighted, gen, depth) * BRDF(n, -ray.Direction, dir, kd, ks, shininess) * glm::abs(glm::dot(n, dir)) * pi;
            //L_indirect = glm::vec3(0);
            return Le + (L_direct + L_indirect) / f;
        }
        return glm::vec3(0); // 背景色
    }
} // namespace VCX::Labs::Rendering