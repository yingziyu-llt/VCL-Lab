#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v         = G.Vertex(i);
                auto neighbors = v->Neighbors();
                // your code here:
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U);
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        bool             visit[output.Positions.size()] = { false };
        std::vector<int> boundary;

        // Set boundary UVs for boundary vertices.
        for (std::size_t i = 0; i < input.Positions.size(); ++i) {
            DCEL::VertexProxy const * v = G.Vertex(i);
            if (v->OnBoundary()) {
                while (true) {
                    auto next = v->BoundaryNeighbors();
                    if (! visit[next.first]) {
                        boundary.push_back(next.first);
                        visit[next.first] = true;
                        v                 = G.Vertex(next.first);
                    } else if (! visit[next.second]) {
                        boundary.push_back(next.second);
                        visit[next.second] = true;
                        v                  = G.Vertex(next.second);
                    } else {
                        break;
                    }
                }
                break;
            }
        }

        int    total_len = boundary.size();
        double step      = 3.1415926 * 2.0 / total_len;

        for (int i = 0; i < total_len; ++i) {
            output.TexCoords[boundary[i]] = glm::vec2 { std::cos(step * i), std::sin(step * i) };
        }

        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            for (int i = 0; i < output.Positions.size(); ++i) {
                if (G.Vertex(i)->OnBoundary()) continue;
                glm::vec2 new_t = glm::vec2(0, 0);
                for (auto j : G.Vertex(i)->Neighbors()) {
                    new_t += output.TexCoords[j];
                }
                new_t /= G.Vertex(i)->Neighbors().size(); // lambda = 1/n,result is terrible ,but works hhh
                output.TexCoords[i] = new_t;
            }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output](DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Kp;
                glm::mat3 vec = { output.Positions[f->VertexIndex(0)], output.Positions[f->VertexIndex(1)], output.Positions[f->VertexIndex(2)] };
                vec           = glm::transpose(vec);
                glm::vec3 par(-1.0f, -1.0f, -1.0f);
                par           = glm::inverse(vec) * par;
                float     dot = sqrt(glm::dot(par, par));
                glm::vec4 q   = glm::vec4(par, 1.0f);
                if (dot > 10000.0f) {
                    return glm::mat4(0);
                }
                q *= 1.0f / dot;
                Kp = glm::mat4(
                    q.x * q.x, q.x * q.y, q.x * q.z, q.x * q.w, 
                    q.y * q.x, q.y * q.y, q.y * q.z, q.y * q.w, 
                    q.z * q.x, q.z * q.y, q.z * q.z, q.z * q.w, 
                    q.w * q.x, q.w * q.y, q.w * q.z, q.w * q.w);
                return Kp;
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;           // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition; // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;           // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [](DCEL::HalfEdge const * edge,
               glm::vec3 const &      p1,
               glm::vec3 const &      p2,
               glm::mat4 const &      Q) -> ContractionPair {
                ContractionPair result;
                result.edge  = edge;
                glm::mat4 Qq = {
                    Q[0][0],Q[1][0],Q[2][0],0,
                    Q[0][1],Q[1][1],Q[2][1],0,
                    Q[0][2],Q[1][2],Q[2][2],0,
                    Q[0][3],Q[1][3],Q[2][3],1,
                };
                if (glm::determinant(Qq) > 0.001f) {
                    glm::vec4 targetPosition = glm::inverse(Qq) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
                    result.targetPosition    = targetPosition;
                    result.cost              = glm::dot(targetPosition, Q * targetPosition);
                } else {
                    glm::vec4 from = glm::vec4(p1, 1.0f);
                    glm::vec4 to   = glm::vec4(p2, 1.0f);
                    glm::vec4 mid  = 0.5f * (from + to);
                    auto      cost = [&](glm::vec4 const & v) -> float {
                        return glm::dot(v, Q * v);
                    };
                    if (cost(from) < cost(to)) {
                        result.targetPosition = from;
                        result.cost           = cost(from);
                    } else {
                        result.targetPosition = to;
                        result.cost           = cost(to);
                    }
                    if (cost(mid) < result.cost) {
                        result.targetPosition = mid;
                        result.cost           = cost(mid);
                    }
                }
                return result;
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map;
        std::vector<ContractionPair>                   pairs;
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Kf(G.NumOfFaces(), glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)] = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (! ~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (! ~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto              v1     = top.edge->From();
            auto              v2     = top.edge->To();
            auto              result = G.Contract(top.edge);
            auto              ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Kf$.
                auto new_Kp = UpdateQ(e->Face());
                Qv[e->From()] += new_Kp - Kf[G.IndexOf(e->Face())];
                Qv[e->To()] += new_Kp - Kf[G.IndexOf(e->Face())];
                Qv[v1] += new_Kp;
                Kf[G.IndexOf(e->Face())] = new_Kp;
            }

            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            for (auto e : ring) {
                auto v2    = e->To();
                auto ring2 = G.Vertex(v2)->Ring();
                for (auto e2 : ring2) {
                    if (! G.IsContractable(e2->NextEdge())) {
                        pairs[pair_map[G.IndexOf(e2->NextEdge())]].edge = nullptr;
                    } else {
                        auto vf                                    = e2->To();
                        auto pair2                                 = MakePair(e2->NextEdge(), input.Positions[v2], input.Positions[vf], Qv[v2] + Qv[vf]);
                        pairs[pair_map[G.IndexOf(e2->NextEdge())]] = pair2;
                    }
                }
            }
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/

    auto UniformLaplacian(Engine::SurfaceMesh const & input) {
        DCEL G(input); // 初始化

        std::vector<glm::vec3> laplacian_values(input.Positions.size(), glm::vec3(0.0f));

        for (int i = 0; i < input.Positions.size(); ++i) {
            DCEL::VertexProxy const * v          = G.Vertex(i);
            glm::vec3                 center_pos = input.Positions[i];

            std::vector<glm::vec3> neighbor_positions;
            for (auto neighbor_idx : v->Neighbors()) {
                neighbor_positions.push_back(input.Positions[neighbor_idx]);
            }

            glm::vec3 laplacian(0.0f);
            for (auto & neighbor_pos : neighbor_positions) {
                laplacian += (neighbor_pos - center_pos);
            }
            int degree = neighbor_positions.size();
            if (degree > 0) {
                laplacian /= degree;
            }

            laplacian_values[i] = laplacian;
        }
        return laplacian_values;
    }

    auto CotangentLaplacian(Engine::SurfaceMesh const & input) {
        DCEL G(input);

        static constexpr auto cot {
            [](glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                float dot   = glm::dot(v1 - vAngle, v2 - vAngle);
                float cross = glm::length(glm::cross(v1 - vAngle, v2 - vAngle));
                //return dot / cross;
                return glm::clamp(dot / cross,1000.0f,-1000.0f);
            }
        };

        std::vector<glm::vec3> laplacian_values(input.Positions.size(), glm::vec3(0.0f));

        for (int i = 0; i < input.Positions.size(); ++i) {
            float                     w          = 0.0f;
            DCEL::VertexProxy const * v          = G.Vertex(i);
            glm::vec3                 center_pos = input.Positions[i];

            std::vector<glm::vec3> neighbor_positions;
            for (auto neighbor_idx : v->Neighbors()) {
                neighbor_positions.push_back(input.Positions[neighbor_idx]);
            }
            glm::vec3 laplacian(0.0f);
            float     wi           = 0.0f;
            glm::vec3 previous_pos = neighbor_positions[neighbor_positions.size() - 1];

            for (auto & neighbor_pos : neighbor_positions) {
                wi = cot(center_pos, previous_pos, neighbor_pos);
                laplacian += wi * (neighbor_pos - center_pos);
                w += wi;
                previous_pos = neighbor_pos;
            }
            if(w < 0.001 && w > 0) w = 0.001;
            if(w > -0.001 && w < 0) w = -0.001;
            laplacian /= w;

            laplacian_values[i] = laplacian;
        }
        return laplacian_values;
    }

    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [](glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                float dot   = glm::dot(v1 - vAngle, v2 - vAngle);
                float cross = glm::length(glm::cross(v1 - vAngle, v2 - vAngle));
                return dot / cross;
                //return glm::clamp(dot / cross,1000.0f,-1000.0f);
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }
        Engine::SurfaceMesh prev_mesh;
        prev_mesh = input;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh    curr_mesh = prev_mesh;
            std::vector<glm::vec3> laplacian = useUniformWeight ? UniformLaplacian(curr_mesh) : CotangentLaplacian(curr_mesh);
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                curr_mesh.Positions[i] = prev_mesh.Positions[i] + lambda * laplacian[i];
                //curr_mesh.Positions[i] = lambda * prev_mesh.Positions[i] + (1 - lambda) * curr_mesh.Positions[i];
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/

    glm::vec3 linearInterpolation(float w1, float w2, glm::vec3 v1, glm::vec3 v2) {
        return (w2 * v1 - w1 * v2) / (v2 - v1);
    }

    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        std::vector<std::vector<std::vector<std::vector<int>>>> edge_point;
        edge_point.resize(n + 1);
        for (int i = 0; i <= n; i++) {
            edge_point[i].resize(n + 1);
            for (int j = 0; j <= n; j++) {
                edge_point[i][j].resize(n + 1);
                for (int k = 0; k <= n; k++) {
                    edge_point[i][j][k].resize(3);
                    for (int l = 0; l < 3; l++)
                        edge_point[i][j][k][l] = -1;
                }
            }
        }
        // printf("successfully initialized");
        for (int x = 0; x < n; x++) {
            //    printf("i=%d\n",i);
            for (int y = 0; y < n; y++) {
                //        printf("j=%d\n",j);
                for (int z = 0; z < n; z++) {
                    //            printf("i=%d,j=%d,k=%d\n",i,j,k);
                    glm::vec3 core_point(x, y, z);
                    core_point *= dx;
                    core_point += grid_min;
                    int state = 0;

                    for (int point_count = 0; point_count < 8; point_count++) {
                        glm::vec3 position(point_count & 1, (point_count >> 1) & 1, (point_count >> 2));
                        position *= dx;
                        position += core_point;

                        if (sdf(position) > 0) {
                            state |= 1 << point_count;
                        }
                    }
                    //            printf("successfully state calculate\n");

                    int e_state = c_EdgeStateTable[state];

                    constexpr std::array<glm::vec3, 3> unit { glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f) };

                    int edge_num = 0;

                    for (int edge_cnt = 0; edge_cnt < 12; edge_cnt++) {
                        if ((e_state & (1 << edge_cnt)) == 0) continue;
                        int det_x = 0, det_y = 0, det_z = 0;
                        edge_num++;
                        if (edge_cnt == 6 || edge_cnt == 7 || edge_cnt == 9 || edge_cnt == 11) det_x = 1;
                        if (edge_cnt == 1 || edge_cnt == 3 || edge_cnt == 10 || edge_cnt == 11) det_y = 1;
                        if (edge_cnt == 2 || edge_cnt == 3 || edge_cnt == 5 || edge_cnt == 7) det_z = 1;
                        glm::vec3 from_position = core_point
                            + dx * (edge_cnt & 1) * unit[((edge_cnt >> 2) + 1) % 3]
                            + dx * ((edge_cnt >> 1) & 1) * unit[((edge_cnt >> 2) + 2) % 3];
                        glm::vec3 to_position = from_position + unit[edge_cnt >> 2] * dx;
                        float     to_dis = sdf(to_position), from_dis = sdf(from_position);
                        if (edge_point[x + det_x][y + det_y][z + det_z][edge_cnt / 4] == -1) {
                            edge_point[x + det_x][y + det_y][z + det_z][edge_cnt / 4] = output.Positions.size();
                            output.Positions.push_back((from_position * to_dis - to_position * from_dis) / (to_dis - from_dis));
                        }
                    }
                    //            printf("successfully point calculate\n");

                    for (int l = 0; l < edge_num - 2; l++) {
                        for (int m = 0; m < 3; m++) {
                            int c = c_EdgeOrdsTable[state][3 * l + 2 - m];
                            if (c == -1) break;
                            int det_x = 0, det_y = 0, det_z = 0;
                            if (c == 6 || c == 7 || c == 9 || c == 11) det_x = 1;
                            if (c == 1 || c == 3 || c == 10 || c == 11) det_y = 1;
                            if (c == 2 || c == 3 || c == 5 || c == 7) det_z = 1;
                            output.Indices.push_back(edge_point[x + det_x][y + det_y][z + det_z][c / 4]);
                        }
                    }
                    //            printf("successfully edge added\n");
                }
            }
        }
        printf("%d\n", output.Positions.size());
        printf("%d\n", output.Indices.size());
    }
} // namespace VCX::Labs::GeometryProcessing
