#pragma once

#include "Engine/Scene.h"
#include "Labs/RayTracing/Ray.h"
#include <glm/glm.hpp>
#include <numeric>
#include <spdlog/spdlog.h>
#include <vector>
#include <algorithm>
#include <stack>

namespace VCX::Labs::Rendering {
    
    constexpr float EPS1 = 1e-2f;
    constexpr float EPS2 = 1e-8f;
    constexpr float EPS3 = 1e-4f;

    glm::vec4 GetTexture(const Engine::Texture2D<Engine::Formats::RGBA8> & texture, const glm::vec2 & uvCoord);
    glm::vec4 GetAlbedo(const Engine::Material & material, const glm::vec2 & uvCoord);

    struct Intersection {
        float t, u, v;
    };

    bool IntersectTriangle(Intersection & output, const Ray & ray, const glm::vec3 & p1, const glm::vec3 & p2, const glm::vec3 & p3);

    struct RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec4         IntersectAlbedo;
        glm::vec4         IntersectMetaSpec;
    };

    class Face {
    public:
        glm::vec3 Vertices[3];
        int       Index    = -1;
        int       modelIdx = -1;
        glm::vec3 Center() const;
    };

    class AABB {
    public:
        glm::vec3 Min_, Max_;
        AABB();
        AABB(const glm::vec3 & min, const glm::vec3 & max);
        AABB(const Face & triangle);
        AABB      Merge(const AABB & other) const;
        bool      Intersect(const Ray & ray, float & tmin, float & tmax) const;
        glm::vec3 Center() const;
        bool      Contains(const glm::vec3 & point) const;
    };

    class BVHNode {
    public:
        AABB BoundingBox_;
        std::shared_ptr<BVHNode> Left_ = nullptr;
        std::shared_ptr<BVHNode> Right_ = nullptr;
        std::vector<Face> Faces_;
        BVHNode(int depth);
        BVHNode() : BoundingBox_() {}
        BVHNode(const std::vector<Face> & faces, int depth);
        bool is_leaf() const;
    };

    class BVHTree{
    public:
        std::shared_ptr<BVHNode> Root_;
        BVHTree();
        std::shared_ptr<BVHNode> BuildBVH(std::vector<Face> & faces, std::shared_ptr<BVHNode> node, int depth);
        ~BVHTree();
    };

    class BVGRayIntersector {
    public:
        BVHTree *SceneBVH_;
        const Engine::Scene *InternalScene;
        BVGRayIntersector() {};
        void InitScene(const Engine::Scene * scene);
        ~BVGRayIntersector();
        RayHit IntersectRay(const Ray & ray) const;
    };
    using RayIntersector = BVGRayIntersector;
    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);

} // namespace VCX::Labs::Rendering
