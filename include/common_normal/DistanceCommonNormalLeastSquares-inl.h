/** BSD 3-Clause License

* Copyright (c) 2022, National University of Singapore
* All rights reserved.

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \author Sipu Ruan */

#pragma once

#include "DistanceCommonNormalLeastSquares.h"

namespace collision_common_normal {

template <typename G1, typename G2>
CostCommonNormalLeastSquares<G1, G2>::CostCommonNormalLeastSquares(
    const G1* s1, const G2* s2, const Eigen::Matrix3d& M1,
    const Eigen::Matrix3d& M2, const Eigen::Vector3d& s1_center,
    const Eigen::Vector3d& s2_center)
    : s1_(s1),
      s2_(s2),
      M1_(M1),
      M2_(M2),
      s1_center_(s1_center),
      s2_center_(s2_center) {}

template <typename G1, typename G2>
CostCommonNormalLeastSquares<G1, G2>::~CostCommonNormalLeastSquares() {}

template <typename G1, typename G2>
template <typename T>
bool CostCommonNormalLeastSquares<G1, G2>::operator()(const T* const psi,
                                                      T* cost) const {
    Eigen::Matrix<T, 2, 1> psi1 = {psi[0], psi[1]};
    Eigen::Matrix<T, 2, 1> psi2 = {psi[2], psi[3]};

    // Use common normal concept to compute \psi_2
    Eigen::Matrix<T, 3, 1> n1_local =
        s1_->getGradientFromSpherical(psi1).normalized();
    Eigen::Matrix<T, 3, 1> n2_local =
        s2_->getGradientFromSpherical(psi2).normalized();

    Eigen::Matrix<T, 3, 1> x1 =
        M1_ * s1_->getBoundaryFromNormal(n1_local) + s1_center_;
    Eigen::Matrix<T, 3, 1> x2 =
        M2_ * s2_->getBoundaryFromNormal(n2_local) + s2_center_;

    // Cost for antiparallelism
    Eigen::Matrix<T, 3, 1> n1 =
        (M1_.transpose().inverse() * n1_local).normalized();
    Eigen::Matrix<T, 3, 1> n2 =
        (M2_.transpose().inverse() * n2_local).normalized();

    Eigen::Matrix<T, 3, 1> normal_point_parallel = n1.cross(x2 - x1);
    Eigen::Matrix<T, 3, 1> normal_normal_parallel = n1.cross(n2);

    cost[0] = normal_point_parallel(0);
    cost[1] = normal_point_parallel(1);
    cost[2] = normal_point_parallel(2);
    cost[3] = normal_normal_parallel(0);
    cost[4] = normal_normal_parallel(1);
    cost[5] = normal_normal_parallel(2);

    return true;
}

template <typename G1, typename G2>
DistanceCommonNormalLeastSquares<G1, G2>::DistanceCommonNormalLeastSquares(
    const cfc::Shape3D& s1_shape, const cfc::Shape3D& s2_shape)
    : DistanceCommonNormal<G1, G2>::DistanceCommonNormal(s1_shape, s2_shape) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceCommonNormalLeastSquares<G1, G2>::DistanceCommonNormalLeastSquares(
    const G1* s1, const G2* s2)
    : DistanceCommonNormal<G1, G2>::DistanceCommonNormal(s1, s2) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceCommonNormalLeastSquares<G1, G2>::~DistanceCommonNormalLeastSquares() {}

template <typename G1, typename G2>
bool DistanceCommonNormalLeastSquares<G1, G2>::normalOptimization() {
    bool status = false;

    // Retrieve spherical coordinate from initial gradient
    Eigen::Vector3d n1_local = this->M1_.transpose() * this->normal_;
    Eigen::Vector3d n2_local = -this->M2_.transpose() * this->normal_;

    this->psi_[0] = std::atan2(
        n1_local[2], sqrt(pow(n1_local[0], 2.0) + pow(n1_local[1], 2.0)));
    this->psi_[1] = std::atan2(n1_local[1], n1_local[0]);
    this->psi_[2] = std::atan2(
        n2_local[2], sqrt(pow(n2_local[0], 2.0) + pow(n2_local[1], 2.0)));
    this->psi_[3] = std::atan2(n2_local[1], n2_local[0]);

    // Set up the only cost function (also known as residual). This
    // uses auto-differentiation to obtain the derivative (jacobian)
    cost_ =
        new ceres::AutoDiffCostFunction<CostCommonNormalLeastSquares<G1, G2>, 6,
                                        4>(
            new CostCommonNormalLeastSquares<G1, G2>(
                this->s1_, this->s2_, this->M1_, this->M2_, this->s1_center_,
                this->s2_center_));
    problem_.AddResidualBlock(cost_, nullptr, this->psi_);

    // Get solutions
    ceres::Solve(options_, &problem_, &summary_);

    Eigen::Vector2d Psi = {this->psi_[0], this->psi_[1]};
    Eigen::Vector3d gradient =
        this->M1_ * this->s1_->getGradientFromSpherical(Psi);
    this->setVariables(gradient.normalized());

    if (summary_.termination_type != ceres::NO_CONVERGENCE) {
        status = true;
    }

    this->res_.num_iteration = summary_.num_successful_steps;

    return status;
}

template <typename G1, typename G2>
void DistanceCommonNormalLeastSquares<G1, G2>::setupSolver() {
    // Solver options
    options_.minimizer_progress_to_stdout = false;
    options_.logging_type = ceres::SILENT;

    options_.max_num_iterations = 100;
    options_.function_tolerance = 1e-16;
    options_.gradient_tolerance = 1e-16;

    options_.minimizer_type = ceres::TRUST_REGION;
    options_.trust_region_strategy_type = ceres::DOGLEG;
}

}  // namespace collision_common_normal
