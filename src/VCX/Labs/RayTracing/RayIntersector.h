#pragma once

#include "Engine/Scene.h"
#include "Labs/RayTracing/Ray.h"
#include <glm/glm.hpp>
#include <numeric>
#include <spdlog/spdlog.h>
#include <vector>

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

    class Triangle {
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
        AABB(const Triangle & triangle);
        AABB      Merge(const AABB & other) const;
        bool      Intersect(const Ray & ray, float & tmin, float & tmax) const;
        glm::vec3 Center() const;
        bool      Contains(const glm::vec3 & point) const;
        bool      Contains(const Triangle & triangle) const;
    };

    class OctTreeNode {
    public:
        std::vector<Triangle> Triangles_;
        AABB                  BoundingBox_;
        OctTreeNode *         Children_[8];
        int                   Depth_;
        OctTreeNode(int depth);
        OctTreeNode(const std::vector<Triangle> & triangles, int depth);
        bool is_leaf() const;
    };

    class OctTree {
    public:
        OctTreeNode * Root_;
        OctTree();
        OctTree(const std::vector<Triangle> & triangles, AABB bounding_box, int depth);
        ~OctTree();
    };

    class OctTreeRayIntersector {
    public:
        Engine::Scene const * InternalScene = nullptr;
        OctTree *             SceneOctTree  = nullptr;
        OctTreeRayIntersector();
        ~OctTreeRayIntersector();
        void   InitScene(Engine::Scene const * scene);
        RayHit IntersectRay(const Ray & ray) const;

    private:
        void IntersectOctTree(OctTreeNode * node, const Ray & ray, float & tmin, Intersection & its, int & modelIdx, int & meshIdx, float & u, float & v) const;
    };

    using RayIntersector = OctTreeRayIntersector;

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);

} // namespace VCX::Labs::Rendering
