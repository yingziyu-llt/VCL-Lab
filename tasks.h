#pragma once

#include <numeric>
#include <spdlog/spdlog.h>

#include "Engine/Scene.h"
#include "Labs/3-Rendering/Ray.h"

namespace VCX::Labs::Rendering {

    constexpr float EPS1 = 1e-2f; // distance to prevent self-intersection
    constexpr float EPS2 = 1e-8f; // angle for parallel judgement
    constexpr float EPS3 = 1e-4f; // relative distance to enlarge kdtree

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord);

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord);

    struct Intersection {
        float t, u, v; // ray parameter t, barycentric coordinates (u, v)
    };

    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3);

    struct RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec4         IntersectAlbedo;   // [Albedo   (vec3), Alpha     (float)]
        glm::vec4         IntersectMetaSpec; // [Specular (vec3), Shininess (float)]
    };

    struct TrivialRayIntersector {
        Engine::Scene const * InternalScene = nullptr;

        TrivialRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
        }

        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    if (! IntersectTriangle(its, ray, p1, p2, p3)) continue;
                    if (its.t < EPS1 || its.t > tmin) continue;
                    tmin = its.t, umin = its.u, vmin = its.v, modelIdx = i, meshIdx = j;
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);

            return result;
        }
    };

    /* Optional: write your own accelerated intersector here */

    class SimpleMesh {
    public:
        glm::vec3 Position[3];
        int       index;
        glm::vec3 center() const {
            return (Position[0] + Position[1] + Position[2]) / 3.0f;
        }
    };
    class AABB {
    public:
        glm::vec3 Min, Max;
        AABB() {
            Min = glm::vec3(1e7);
            Max = glm::vec3(-1e7);
        }
        AABB(glm::vec3 min, glm::vec3 max) {
            Min = min;
            Max = max;
        }
        AABB merge(const AABB & other) {
            return AABB(glm::min(Min, other.Min), glm::max(Max, other.Max));
        }
        AABB(const SimpleMesh & triangulation) {
            Min = glm::vec3(1e7);
            Max = glm::vec3(-1e7);
            for (auto & pos : triangulation.Position) {
                Min = glm::min(Min, pos);
                Max = glm::max(Max, pos);
            }
        }
        bool intersect(const Ray & ray, float & tmin, float & tmax) const {
            glm::vec3 invDir = glm::vec3(
                ray.Direction.x != 0 ? 1.0f / ray.Direction.x : FLT_MAX,
                ray.Direction.y != 0 ? 1.0f / ray.Direction.y : FLT_MAX,
                ray.Direction.z != 0 ? 1.0f / ray.Direction.z : FLT_MAX);

            glm::vec3 t0      = (Min - ray.Origin) * invDir;
            glm::vec3 t1      = (Max - ray.Origin) * invDir;
            glm::vec3 t_min_v = glm::min(t0, t1);
            glm::vec3 t_max_v = glm::max(t0, t1);
            tmin              = glm::max(glm::max(t_min_v.x, t_min_v.y), t_min_v.z);
            tmax              = glm::min(glm::min(t_max_v.x, t_max_v.y), t_max_v.z);
            return tmax >= tmin;
        }
        glm::vec3 center() const {
            return (Min + Max) / 2.0f;
        }
    };

    class KDNode {
    public:
        AABB                    Box;             // 包围盒
        KDNode *                Left  = nullptr; // 左子节点
        KDNode *                Right = nullptr; // 右子节点
        std::vector<SimpleMesh> Meshes;
        int                     PrimitiveIndex = -1; // 当前节点存储的原始对象索引 (-1 表示非叶节点)

        KDNode() = default;
        ~KDNode() {
            if (Left != nullptr) {
                delete Left;
                Left = nullptr;
            }
            if (Right != nullptr) {
                delete Right;
                Right = nullptr;
            }
        }

        bool isLeaf() const {
            return Left == nullptr && Right == nullptr;
        }
    };

    class KDTree {
    private:
        KDNode * Root;

        KDNode * build(std::vector<SimpleMesh> meshes, int depth) {
            if (meshes.empty()) return nullptr;

            KDNode * node = new KDNode();
            for (auto & mesh : meshes) {
                AABB triBox(mesh);
                node->Box = node->Box.merge(triBox);
            }
            if (meshes.size() <= 10) {
                node->Meshes = meshes;
                return node;
            }

            int axis = depth % 3;
            std::sort(meshes.begin(), meshes.end(), [axis](const SimpleMesh & a, const SimpleMesh & b) {
                glm::vec3 centerA = a.center();
                glm::vec3 centerB = b.center();
                return centerA[axis] < centerB[axis];
            });
            int mid     = meshes.size() / 2;
            node->Left  = build(std::vector<SimpleMesh>(meshes.begin(), meshes.begin() + mid), depth + 1);
            node->Right = build(std::vector<SimpleMesh>(meshes.begin() + mid, meshes.end()), depth + 1);
            return node;
        }
        bool intersectNode(const KDNode * node, const Ray & ray, Intersection & closestIts, float closestT, int & faceIdX) const {
            if (! node || ! node->Box.intersect(ray, closestIts.t, closestT)) return false;

            bool hit = false;

            if (node->isLeaf()) {
                for (const auto & tri : node->Meshes) {
                    Intersection its;
                    if (IntersectTriangle(its, ray, tri.Position[0], tri.Position[1], tri.Position[2]) && its.t < closestT) {
                        closestT   = its.t;
                        closestIts = its;
                        faceIdX    = tri.index;
                        hit        = true;
                    }
                }
                return hit;
            }

            // 检查子节点
            Intersection itsLeft, itsRight;
            int          faceIdLeft, faceIdRight;
            bool         hitLeft  = intersectNode(node->Left, ray, itsLeft, closestT, faceIdLeft);
            bool         hitRight = intersectNode(node->Right, ray, itsRight, closestT, faceIdRight);
            if (hitLeft && hitRight) {
                closestIts = itsLeft.t < itsRight.t ? itsLeft : itsRight;
                faceIdX    = itsLeft.t < itsRight.t ? faceIdLeft : faceIdRight;
            } else if (hitLeft) {
                closestIts = itsLeft;
                faceIdX    = faceIdLeft;
            } else if (hitRight) {
                closestIts = itsRight;
                faceIdX    = faceIdRight;
            }
            return hitLeft || hitRight;
        }

    public:
        KDTree(VCX::Engine::SurfaceMesh Meshes) {
            std::vector<SimpleMesh> meshes;
            for (int i = 0; i < Meshes.Indices.size(); i += 3) {
                SimpleMesh mesh;
                mesh.Position[0] = Meshes.Positions[Meshes.Indices[i]];
                mesh.Position[1] = Meshes.Positions[Meshes.Indices[i + 1]];
                mesh.Position[2] = Meshes.Positions[Meshes.Indices[i + 2]];
                mesh.index       = i / 3;
                meshes.push_back(mesh);
            }
            Root = build(meshes, 0);
        }
        ~KDTree() {
            if (Root != nullptr) {
                delete Root;
                Root = nullptr;
            }
        }
        bool intersect(const Ray & ray, Intersection & hit, float closestT, int & indexIdX) const {
            return intersectNode(Root, ray, hit, closestT, indexIdX);
        }
    };

    struct KDTreeRayIntersector {
        Engine::Scene const * InternalScene = nullptr;
        std::vector<KDTree>   kdTrees;

        KDTreeRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
            int maxmodel  = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                KDTree       tree(model.Mesh);
                kdTrees.push_back(tree);
            }
        }

        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                int          meshId;
                if (kdTrees[i].intersect(ray, its, tmin, meshId)) {
                    if (its.t < tmin && its.t > EPS1) {
                        tmin     = its.t;
                        umin     = its.u;
                        vmin     = its.v;
                        modelIdx = i;
                        meshIdx  = meshId;
                    }
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);

            return result;
        }
    };

    using RayIntersector = TrivialRayIntersector;

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);

} // namespace VCX::Labs::Rendering
