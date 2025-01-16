#include "Labs/RayTracing/WhittedStyleRayTracing.h"

namespace VCX::Labs::Rendering {

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);
        float     gamma   = 2.2f;
        float     ambient = 0.05f;

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;

            glm::vec3 result(0.0f);
            result = kd * ambient;

            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        Ray shadowRay(pos,glm::normalize(l));
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        if(shadowHit.IntersectState) {
                            float lightDist = glm::length(l);
                            float hitDist = glm::length(shadowHit.IntersectPosition - pos);
                            if(hitDist <= lightDist && shadowHit.IntersectAlbedo.w >= 0.2f) continue;
                        }
                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        Ray shadowRay(pos,glm::normalize(l));
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        if(shadowHit.IntersectState) {
                            if(shadowHit.IntersectAlbedo.w >= 0.2f) continue;
                        }
                    }
                }

                glm::vec3 h         = glm::normalize(-ray.Direction + glm::normalize(l));
                result += light.Intensity * attenuation * (glm::max(glm::dot(glm::normalize(l), n), 0.0f) * kd + glm::max(glm::pow(glm::max(glm::dot(h, n), 0.0f), shininess),0.0f) * ks);
            }

            if (alpha < 0.9) {

                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;
                ray = Ray(pos + ray.Direction * 0.01f, ray.Direction);
            } else {
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos + out_dir * 0.01f, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering