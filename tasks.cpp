#include "Labs/4-Animation/tasks.h"
#include "CustomFunc.inl"
#include "IKSystem.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <spdlog/spdlog.h>

namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }

        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + ik.JointGlobalRotation[i - 1] * ik.JointLocalOffset[i];
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        int time = 0;
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            for (int i = ik.NumJoints() - 2; i >= 0; i--) {
                glm::vec3 to_target       = glm::normalize(EndPosition - ik.JointGlobalPosition[i]);
                glm::vec3 to_end          = glm::normalize(ik.EndEffectorPosition() - ik.JointGlobalPosition[i]);
                glm::quat rot             = glm::rotation(to_end, to_target);
                ik.JointGlobalRotation[i] = rot * ik.JointGlobalRotation[i];
                if (i == 0) {
                    ik.JointLocalRotation[i] = ik.JointGlobalRotation[i];
                } else {
                    ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
                }
                ForwardKinematics(ik, i);
            }
            time++;
        }
        printf("CCD Iteration: %d\n", time);
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int                    nJoints = ik.NumJoints();
        int time = 0;
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                backward_positions[i] = backward_positions[i + 1] + glm::normalize(ik.JointGlobalPosition[i] - backward_positions[i + 1]) * glm::length(ik.JointLocalOffset[i + 1]);
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0]   = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints; i++) {
                forward_positions[i + 1] = forward_positions[i] + (backward_positions[i + 1] - forward_positions[i]) * glm::length(ik.JointLocalOffset[i + 1]) / glm::length(backward_positions[i + 1] - forward_positions[i]);
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
            time++;
        }
        printf("FABRIK Iteration: %d\n", time);

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(0));
        int index = 0;
        auto calculate_point = [](float t) {
            float x_val = 1.5e-3f * custom_x(t);
            float y_val = 1.5e-3f * custom_y(t);
            return glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        };
        for (int i = 0; i < nums; i++) {
            float delta =  glm::pi<float>() * 92.0f / nums;
            float previous_theta = 92 * glm::pi<float>() * (i - 1) / nums;
            auto end_point = calculate_point(92 * glm::pi<float>() * i / nums);
            if (std::abs(1.6 - end_point.x) < 1e-3 || std::abs(end_point.z + 0.2) < 1e-3) continue;
            if(glm::length(end_point - calculate_point(previous_theta)) < 0.05f)
            {
                (*custom).push_back(end_point);
                index++;
                continue;
            }
            while(glm::length(end_point - calculate_point(previous_theta)) > 0.05f && delta > 1e-4f) {
                delta /= 2.0f;
                end_point = calculate_point(previous_theta + delta);
            }
            float curr_theta = previous_theta + delta;
            while(92 * glm::pi<float>() * i / nums - curr_theta > 1e-4f) {
                auto epoint = calculate_point(curr_theta);
                index++;
                curr_theta += delta;
                if (std::abs(1.6 - epoint.x) < 1e-3 || std::abs(epoint.z + 0.2) < 1e-3) continue;
                (*custom).push_back(calculate_point(curr_theta));
            }
        }
        return custom;
    }

    /*IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        int nums      = 50;
        using Vec3Arr = std::vector<glm::vec3>;

        glm::vec3                shift = glm::vec3(1, 0, 0);
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(0));
        for (int i = 0; i < nums; i++) {
            float x = 0;
            float y = -1.0f / nums * i + 0.5f;
            (*custom).push_back(glm::vec3(x, 0, y) + shift); 
        }
        for (int i = 0; i < nums; i++) {
            float x = -0.4 / nums * i;
            float y = -0.5;
            (*custom).push_back(glm::vec3(x, 0, y) + shift); 
        }
        shift = glm::vec3(0, 0, 0);
        for (int i = 0; i < nums; i++) {
            float x = 0;
            float y = -1.0f / nums * i + 0.5f;
            (*custom).push_back(glm::vec3(x, 0, y) + shift); 
        }
        for (int i = 0; i < nums; i++) {
            float x = -0.4 / nums * i;
            float y = -0.5;
            (*custom).push_back(glm::vec3(x, 0, y) + shift); 
        }
        shift = glm::vec3(-1, 0, 0);
        for (int i = 0; i < nums; i++) {
            float x = -1.0 / nums * i + 0.5;
            float y = 0.5;
            (*custom).push_back(glm::vec3(x, 0, y) + shift);
        }
        for (int i = 0; i < nums; i++) {
            float x = 0;
            float y = -1.0f / nums * i + 0.5f;
            (*custom).push_back(glm::vec3(x, 0, y) + shift);
        }
        shift  = glm::vec3(0, 0, 0);
        for (int i = 0; i < nums; i++) {
            float t = 2.0f * glm::pi<float>() * i / (nums - 1); // 参数 t
            float x = 0.5 * std::pow(std::sin(t), 3);
            float y = 0.5 * (13 * std::cos(t) - 5 * std::cos(2 * t) - 2 * std::cos(3 * t) - std::cos(4 * t)) / 16.0f;
            (*custom).push_back(glm::vec3(x,0,y) + shift); // 存储点
        }
        shift = glm::vec3(-0.8, 0, 0);
        nums = 100;
        for (int i = 0; i < nums; i++) {
            float x = 0;
            float y = 1.0f / nums * i - 0.5f;
            (*custom).push_back(glm::vec3(x,0,y) + shift); // 存储点
        }
        for (int i = 0; i < nums; i++) {
            float x = -0.4 / nums * i;
            float y = 0.5;
            (*custom).push_back(glm::vec3(x,0,y) + shift); // 存储点
        }
        for (int i = 0; i < nums; i++) {
            float x = -0.3 / nums * i;
            float y = 0;
            (*custom).push_back(glm::vec3(x,0,y) + shift); // 存储点
        }

        return custom;
    }*/

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size()));
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const &            b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        std::vector<Eigen::Triplet<float>> triplets;
        Eigen::VectorXf                    xk = glm2eigen(system.Positions);

        // Calculate yk
        Eigen::VectorXf yk = glm2eigen(system.Positions) + dt * glm2eigen(system.Velocities);
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            if (! system.Fixed[i]) {
                yk.segment<3>(3 * i) += dt * dt * Eigen::Vector3f(0, -system.Gravity, 0);
            }
        }
        std::vector<Eigen::Triplet<float>> mass;

        // Add gravity to the right-hand side vector
        mass.reserve(system.Positions.size() * 3);

        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            mass.emplace_back(3 * i, 3 * i, system.Mass);
            mass.emplace_back(3 * i + 1, 3 * i + 1, system.Mass);
            mass.emplace_back(3 * i + 2, 3 * i + 2, system.Mass);
        }

        Eigen::SparseMatrix<float> M(system.Positions.size() * 3, system.Positions.size() * 3);

        M.setFromTriplets(mass.begin(), mass.end());

        Eigen::VectorXf grad_g = Eigen::VectorXf::Zero(system.Positions.size() * 3);
        

        triplets.reserve(system.Springs.size() * 12);
        // Construct the system matrix and the right-hand side vector
        for (auto const & spring : system.Springs) {
            auto const      p0  = spring.AdjIdx.first;
            auto const      p1  = spring.AdjIdx.second;
            glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
            float const     l   = glm::length(x01);
            glm::vec3 const e01 = x01 / l;

            float const k = system.Stiffness;

            // Construct the stiffness matrix for the spring
            Eigen::Matrix3f K;
            Eigen::Vector3f e01_eigen(e01.x, e01.y, e01.z);
            K = k * (e01_eigen * e01_eigen.transpose());

            // Construct the force vector for the spring
            glm::vec3 f = k * (l - spring.RestLength) * e01;

            // Convert glm::vec3 to Eigen::Vector3f
            Eigen::Vector3f f_eigen(f.x, f.y, f.z);
            
            // Add contributions to the system matrix and the right-hand side vector
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    triplets.emplace_back(3 * p0 + i, 3 * p0 + j, K(i, j));
                    triplets.emplace_back(3 * p0 + i, 3 * p1 + j, -K(i, j));
                    triplets.emplace_back(3 * p1 + i, 3 * p0 + j, -K(i, j));
                    triplets.emplace_back(3 * p1 + i, 3 * p1 + j, K(i, j));
                }
            }
            grad_g.segment<3>(3 * p0) -= f_eigen;
            grad_g.segment<3>(3 * p1) += f_eigen;
        }
        grad_g += M / (dt * dt) * (xk - yk);
        Eigen::SparseMatrix<float> H(system.Positions.size() * 3, system.Positions.size() * 3);
        H.setFromTriplets(triplets.begin(), triplets.end());
        H += 1.0 / (dt * dt) * M;
        Eigen::VectorXf dX = ComputeSimplicialLLT(H, -grad_g);
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            if (! system.Fixed[i]) {
                system.Positions[i] += glm::vec3(dX(3 * i), dX(3 * i + 1), dX(3 * i + 2));
                system.Velocities[i] = (system.Positions[i] - glm::vec3(xk(3 * i), xk(3 * i + 1), xk(3 * i + 2))) / dt;
            }
        }
    }


} // namespace VCX::Labs::Animation
