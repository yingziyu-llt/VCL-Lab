#include "Labs/RayTracing/RayIntersector.h"

namespace VCX::Labs::Rendering {

    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        glm::vec3 normal = glm::cross(p1 - p2, p1 - p3);
        if (abs(glm::dot(ray.Direction, normal)) < 1e-6)
            return false;
        output.t = glm::dot(p1 - ray.Origin, normal) / glm::dot(ray.Direction, normal);
        if (output.t <= 1e-2)
            return false;
        output.u = glm::dot(p3 - p1, glm::cross(ray.Direction, ray.Origin - p1)) / glm::dot(ray.Direction, normal);
        output.v = glm::dot(-p2 + p1, glm::cross(ray.Direction, ray.Origin - p1)) / glm::dot(ray.Direction, normal);
        if (output.u < 0 || output.v > 1 || output.v < 0 || output.v > 1 || output.u + output.v > 1)
            return false;
        return true;
    }

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const &texture, glm::vec2 const &uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        
        glm::vec2 uv = glm::fract(uvCoord);
        uv.x = uv.x * texture.GetSizeX() - 0.5f;
        uv.y = uv.y * texture.GetSizeY() - 0.5f;

        std::size_t xmin = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax = (ymin + 1) % texture.GetSizeY();

        float xfrac = glm::fract(uv.x);
        float yfrac = glm::fract(uv.y);

        return glm::mix(
            glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac),
            glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac),
            xfrac
        );
    }

    glm::vec4 GetAlbedo(Engine::Material const &material, glm::vec2 const &uvCoord) {
        glm::vec4 albedo = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2f)), albedo.w);
    }

    // Triangle Methods
    glm::vec3 Triangle::Center() const {
        return (Vertices[0] + Vertices[1] + Vertices[2]) / 3.0f;
    }

    // AABB Methods
    AABB::AABB() : Min_(FLT_MAX), Max_(-FLT_MAX) {}

    AABB::AABB(const glm::vec3 &min, const glm::vec3 &max) : Min_(min), Max_(max) {}

    AABB AABB::Merge(const AABB &other) const {
        return AABB(glm::min(Min_, other.Min_), glm::max(Max_, other.Max_));
    }

    AABB::AABB(const Triangle &triangle) : Min_(FLT_MAX), Max_(-FLT_MAX) {
        for (const auto &vertex : triangle.Vertices) {
            Min_ = glm::min(Min_, vertex - glm::vec3(EPS3));
            Max_ = glm::max(Max_, vertex + glm::vec3(EPS3));
        }
    }

    bool AABB::Intersect(const Ray &ray, float &tmin, float &tmax) const {
        tmin = 0.0f;
        tmax = std::numeric_limits<float>::max();

        for (int i = 0; i < 3; ++i) {
            if (fabs(ray.Direction[i]) < EPS2) {
                if (ray.Origin[i] < Min_[i] || ray.Origin[i] > Max_[i]) {
                    return false;
                }
            } else {
                float invD = 1.0f / ray.Direction[i];
                float t0 = (Min_[i] - ray.Origin[i]) * invD;
                float t1 = (Max_[i] - ray.Origin[i]) * invD;

                if (invD < 0.0f) std::swap(t0, t1);

                tmin = std::max(t0, tmin);
                tmax = std::min(t1, tmax);

                if (tmax <= tmin) return false;
            }
        }

        return true;
    }

    glm::vec3 AABB::Center() const {
        return (Min_ + Max_) * 0.5f;
    }

    bool AABB::Contains(const glm::vec3 &point) const {
        return point.x >= Min_.x && point.x <= Max_.x &&
               point.y >= Min_.y && point.y <= Max_.y &&
               point.z >= Min_.z && point.z <= Max_.z;
    }

    bool AABB::Contains(const Triangle &triangle) const {
        for (const auto &vertex : triangle.Vertices) {
            if (Contains(vertex)) return true;
        }
        return false;
    }

    // OctTreeNode Methods
    OctTreeNode::OctTreeNode(int depth) : Depth_(depth) {
        std::fill(std::begin(Children_), std::end(Children_), nullptr);
    }

    bool OctTreeNode::is_leaf() const {
        return std::all_of(std::begin(Children_), std::end(Children_), [](OctTreeNode *child) { return child == nullptr; });
    }

    OctTreeNode::OctTreeNode(const std::vector<Triangle> &triangles, int depth) :
        Triangles_(triangles), Depth_(depth) {
        std::fill(std::begin(Children_), std::end(Children_), nullptr);

        AABB aabb;
        for (const auto &triangle : triangles) {
            aabb = aabb.Merge(AABB(triangle));
        }
        BoundingBox_ = aabb;
    }

    // OctTree Methods
    OctTree::OctTree() = default;

    OctTree::OctTree(const std::vector<Triangle> &triangles, AABB bounding_box, int depth) {
        if (depth > 10 || triangles.size() <= 10) {
            Root_ = new OctTreeNode(triangles, depth);
            return;
        }

        Root_ = new OctTreeNode(depth);
        Root_->BoundingBox_ = bounding_box;

        for (int i = 0; i < 8; ++i) {
            glm::vec3 min, max;
            glm::vec3 center = Root_->BoundingBox_.Center();

            min.x = (i & 1) ? center.x : Root_->BoundingBox_.Min_.x;
            max.x = (i & 1) ? Root_->BoundingBox_.Max_.x : center.x;
            min.y = (i & 2) ? center.y : Root_->BoundingBox_.Min_.y;
            max.y = (i & 2) ? Root_->BoundingBox_.Max_.y : center.y;
            min.z = (i & 4) ? center.z : Root_->BoundingBox_.Min_.z;
            max.z = (i & 4) ? Root_->BoundingBox_.Max_.z : center.z;

            AABB aabb(min, max);
            std::vector<Triangle> child_triangles;

            for (const auto &triangle : triangles) {
                if (aabb.Contains(triangle)) {
                    child_triangles.push_back(triangle);
                }
            }

            if (!child_triangles.empty()) {
                Root_->Children_[i] = (new OctTree(child_triangles, aabb, depth + 1))->Root_;
            }
        }
    }

    OctTree::~OctTree() {
        delete Root_;
    }

    // OctTreeRayIntersector Methods
    OctTreeRayIntersector::OctTreeRayIntersector() = default;

    void OctTreeRayIntersector::InitScene(Engine::Scene const *scene) {
        InternalScene = scene;

        std::vector<Triangle> triangles;
        for (const auto &model : scene->Models) {
            for (size_t j = 0; j < model.Mesh.Indices.size(); j += 3) {
                std::uint32_t const *face = model.Mesh.Indices.data() + j;
                Triangle triangle{
                    {model.Mesh.Positions[face[0]], model.Mesh.Positions[face[1]], model.Mesh.Positions[face[2]]},
                    j,
                    &model - &scene->Models[0]
                };
                triangles.push_back(triangle);
            }
        }

        AABB sceneAABB;
        for (const auto &triangle : triangles) {
            sceneAABB = sceneAABB.Merge(AABB(triangle));
        }

        SceneOctTree = new OctTree(triangles, sceneAABB, 0);
    }

    OctTreeRayIntersector::~OctTreeRayIntersector() {
        delete SceneOctTree;
    }

    RayHit OctTreeRayIntersector::IntersectRay(Ray const &ray) const {
        RayHit result;
        if (!InternalScene || !SceneOctTree) {
            spdlog::warn("Uninitialized intersector.");
            result.IntersectState = false;
            return result;
        }

        float tmin = 1e7f;
        Intersection its;
        int modelIdx = -1, meshIdx = -1;
        float u = 1.0f, v = 1.0f;
        IntersectOctTree(SceneOctTree->Root_, ray, tmin, its, modelIdx, meshIdx, u, v);

        if (tmin == 1e7f) {
            result.IntersectState = false;
            return result;
        }

        const auto &model = InternalScene->Models[modelIdx];
        const auto &normals = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
        const auto &texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();

        std::uint32_t const *face = model.Mesh.Indices.data() + meshIdx;
        const glm::vec3 &p1 = model.Mesh.Positions[face[0]];
        const glm::vec3 &p2 = model.Mesh.Positions[face[1]];
        const glm::vec3 &p3 = model.Mesh.Positions[face[2]];
        const glm::vec3 &n1 = normals[face[0]];
        const glm::vec3 &n2 = normals[face[1]];
        const glm::vec3 &n3 = normals[face[2]];
        const glm::vec2 &uv1 = texcoords[face[0]];
        const glm::vec2 &uv2 = texcoords[face[1]];
        const glm::vec2 &uv3 = texcoords[face[2]];

        result.IntersectState = true;
        const auto &material = InternalScene->Materials[model.MaterialIndex];
        result.IntersectMode = material.Blend;
        result.IntersectPosition = (1.0f - u - v) * p1 + u * p2 + v * p3;
        result.IntersectNormal = (1.0f - u - v) * n1 + u * n2 + v * n3;

        glm::vec2 uvCoord = (1.0f - u - v) * uv1 + u * uv2 + v * uv3;
        result.IntersectAlbedo = GetAlbedo(material, uvCoord);
        result.IntersectMetaSpec = GetTexture(material.MetaSpec, uvCoord);

        return result;
    }

    void OctTreeRayIntersector::IntersectOctTree(
        OctTreeNode *node, const Ray &ray, float &tmin,
        Intersection &its, int &modelIdx, int &meshIdx, float &u, float &v) const {
        if (!node) return;

        float t0, t1;
        if (!node->BoundingBox_.Intersect(ray, t0, t1)) return;

        if (node->is_leaf()) {
            for (const auto &triangle : node->Triangles_) {
                if (IntersectTriangle(its, ray, triangle.Vertices[0], triangle.Vertices[1], triangle.Vertices[2])) {
                    if (its.t < tmin && its.t > EPS1) {
                        tmin = its.t;
                        modelIdx = triangle.modelIdx;
                        meshIdx = triangle.Index;
                        u = its.u;
                        v = its.v;
                    }
                }
            }
            return;
        }

        for (int i = 0; i < 8; ++i) {
            IntersectOctTree(node->Children_[i], ray, tmin, its, modelIdx, meshIdx, u, v);
        }
    }

} // namespace VCX::Labs::Rendering
