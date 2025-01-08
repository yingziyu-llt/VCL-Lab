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

    

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);

        glm::vec2 uv = glm::fract(uvCoord);
        uv.x         = uv.x * texture.GetSizeX() - 0.5f;
        uv.y         = uv.y * texture.GetSizeY() - 0.5f;

        std::size_t xmin = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax = (ymin + 1) % texture.GetSizeY();

        float xfrac = glm::fract(uv.x);
        float yfrac = glm::fract(uv.y);

        return glm::mix(
            glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac),
            glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac),
            xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2f)), albedo.w);
    }

    // Triangle Methods
    glm::vec3 Face::Center() const {
        return (Vertices[0] + Vertices[1] + Vertices[2]) / 3.0f;
    }
    // AABB Methods
    AABB::AABB():
        Min_(FLT_MAX), Max_(-FLT_MAX) {}

    AABB::AABB(const glm::vec3 & min, const glm::vec3 & max):
        Min_(min), Max_(max) {}

    AABB AABB::Merge(const AABB & other) const {
        AABB result;
        for (int i = 0; i < 3; ++i) {
            result.Min_[i] = std::min(Min_[i], other.Min_[i]);
            result.Max_[i] = std::max(Max_[i], other.Max_[i]);
        }
        return result;
    }

    AABB::AABB(const Face & faces):
        Min_(FLT_MAX), Max_(-FLT_MAX) {
        //printf("Building AABB!\n");
        //printf("%f %f %f\n", faces.Vertices[0].x, faces.Vertices[0].y, faces.Vertices[0].z);
        //printf("%f %f %f\n", faces.Vertices[1].x, faces.Vertices[1].y, faces.Vertices[1].z);
        //printf("%f %f %f\n", faces.Vertices[2].x, faces.Vertices[2].y, faces.Vertices[2].z);
        for (const auto & vertex : faces.Vertices) {
            for (int i = 0; i < 3; ++i) {
                Min_[i] = std::min(Min_[i], vertex[i] - EPS3);
                Max_[i] = std::max(Max_[i], vertex[i] + EPS3);
            }
        }
    }

    bool AABB::Intersect(const Ray & ray, float & tmin, float & tmax) const {
        tmin = 0.0f;
        tmax = std::numeric_limits<float>::max();

        for (int i = 0; i < 3; ++i) {
            if(fabs(ray.Direction[i]) < EPS1) {
                if (ray.Origin[i] < Min_[i] || ray.Origin[i] > Max_[i]) return false;
                else continue;
            }
            float invD = 1.0f / ray.Direction[i];
            float t0   = (Min_[i] - ray.Origin[i]) * invD;
            float t1   = (Max_[i] - ray.Origin[i]) * invD;

            if (invD < 0.0f) std::swap(t0, t1);

            tmin = std::max(t0, tmin);
            tmax = std::min(t1, tmax);

            if (tmax < tmin) return false;
        }

        return true;
    }

    glm::vec3 AABB::Center() const {
        return (Min_ + Max_) * 0.5f;
    }

    bool AABB::Contains(const glm::vec3 & point) const {
        return point.x >= Min_.x && point.x <= Max_.x && point.y >= Min_.y && point.y <= Max_.y && point.z >= Min_.z && point.z <= Max_.z;
    }

    // BVHNode Methods
    BVHNode::BVHNode(int depth):
        BoundingBox_(), Left_(nullptr), Right_(nullptr), Faces_() {}

    BVHNode::BVHNode(const std::vector<Face> & faces, int depth):
        BoundingBox_(), Left_(nullptr), Right_(nullptr), Faces_(faces) {
        for (const auto & triangle : Faces_) {
            BoundingBox_ = BoundingBox_.Merge(AABB(triangle));
        }
    }

    bool BVHNode::is_leaf() const {
        return Left_ == nullptr && Right_ == nullptr;
    }

    // BVHTree Methods
    BVHTree::BVHTree():
        Root_(nullptr) {}
    BVHTree::~BVHTree() {}
    std::shared_ptr<BVHNode> BVHTree::BuildBVH(std::vector<Face> & faces, std::shared_ptr<BVHNode> node, int depth) {
        if (faces.size() <= 5 || depth > 15) {
            //printf("Build Leaf! faces: %d, depth: %d\n", faces.size(), depth);
            node->Faces_ = faces;
            for (const auto & triangle : faces) {
                node->BoundingBox_ = node->BoundingBox_.Merge(AABB(triangle));
            }
            return node;
        }
        for (const auto & triangle : faces) {
            node->BoundingBox_ = node->BoundingBox_.Merge(AABB(triangle));
        }
        std::vector<float> x, y, z;
        for (const auto & triangle : faces) {
            glm::vec3 center = triangle.Center();
            x.push_back(center.x);
            y.push_back(center.y);
            z.push_back(center.z);
        }
        auto median = [](std::vector<float> & arr) {
            std::nth_element(arr.begin(), arr.begin() + arr.size() / 2, arr.end());
            return arr[arr.size() / 2];
        };
        float x_range = *std::max_element(x.begin(), x.end()) - *std::min_element(x.begin(), x.end());
        float y_range = *std::max_element(y.begin(), y.end()) - *std::min_element(y.begin(), y.end());
        float z_range = *std::max_element(z.begin(), z.end()) - *std::min_element(z.begin(), z.end());

        int axis = 0;
        if (y_range > x_range && y_range > z_range) {
            axis = 1;
        } else if (z_range > x_range && z_range > y_range) {
            axis = 2;
        }

        float split = median(axis == 0 ? x : axis == 1 ? y
                                                       : z);

        std::vector<Face> left, right;
        for (const auto & triangle : faces) {
            if (AABB(triangle).Center()[axis] < split) {
                left.push_back(triangle);
            } else {
                right.push_back(triangle);
            }
        }
        node->Left_  = BuildBVH(left, std::make_shared<BVHNode>(depth + 1), depth + 1);
        node->Right_ = BuildBVH(right, std::make_shared<BVHNode>(depth + 1), depth + 1);
        return node;
    }
    void BVGRayIntersector::InitScene(const Engine::Scene * scene) {
        SceneBVH_ = new BVHTree(), InternalScene = scene;
        std::vector<Face> faces;
        for (int i = 0; i < scene->Models.size(); ++i) {
            auto & model = scene->Models[i];
            for (int j = 0; j < model.Mesh.Indices.size(); j += 3) {
                Face face;
                for (int k = 0; k < 3; ++k) {
                    face.Vertices[k] = model.Mesh.Positions[model.Mesh.Indices[j + k]];
                }
                face.Index    = j;
                face.modelIdx = i;
                faces.push_back(face);
            }
        }
        SceneBVH_->Root_ = SceneBVH_->BuildBVH(faces, std::make_shared<BVHNode>(), 0);
    }

    BVGRayIntersector::~BVGRayIntersector() {
        delete SceneBVH_;
    }

    RayHit BVGRayIntersector::IntersectRay(const Ray & ray) const {
        RayHit result;

        std::stack<std::shared_ptr<BVHNode>> stack;
        stack.push(SceneBVH_->Root_);
        float        tmin     = 1e7, umin, vmin;
        int          maxmodel = InternalScene->Models.size();
        int          modelIdx = -1;
        int          meshIdx  = -1;
        Intersection its;

        while (! stack.empty()) {
            auto node = stack.top();
            stack.pop();
            float t1 = tmin, t2 = 0;
            //printf("Intersecting Node!\n");
            if (! node->BoundingBox_.Intersect(ray, t1, t2)) {
                //printf("No Intersection!\n");
                //printf("t1: %f, t2: %f\n", t1, t2);
                //printf("min:(%f,%f,%f) max:(%f,%f,%f)", node->BoundingBox_.Min_.x, node->BoundingBox_.Min_.y, node->BoundingBox_.Min_.z, node->BoundingBox_.Max_.x, node->BoundingBox_.Max_.y, node->BoundingBox_.Max_.z);
                continue;
            }
            if (node->is_leaf()) {
                for (const auto & triangle : node->Faces_) {
                    Intersection output;
                    if (IntersectTriangle(output, ray, triangle.Vertices[0], triangle.Vertices[1], triangle.Vertices[2])) {
                        if (output.t < tmin) {
                            tmin     = std::min(tmin, output.t);
                            umin     = output.u;
                            vmin     = output.v;
                            modelIdx = triangle.modelIdx;
                            meshIdx  = triangle.Index;
                        }
                    }
                }
            } else {
                stack.push(node->Left_);
                stack.push(node->Right_);
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
} // namespace VCX::Labs::Rendering
