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

#include "DistanceLeastSquares.h"

namespace collision_cfc {

template <typename G1, typename G2>
CostDistanceLeastSquares<G1, G2>::CostDistanceLeastSquares(
    const G1* s1, const G2* s2, const Eigen::Matrix3d& M1,
    const Eigen::Matrix3d& M2, const Eigen::Vector3d& p0)
    : s1_(s1), s2_(s2), M1_(M1), M2_(M2), p0_(p0) {}

template <typename G1, typename G2>
CostDistanceLeastSquares<G1, G2>::~CostDistanceLeastSquares() {}

template <typename G1, typename G2>
template <typename T>
bool CostDistanceLeastSquares<G1, G2>::operator()(const T* const psi,
                                                  T* cost) const {
    // Compute normal vector
    Eigen::Matrix<T, 2, 1> Psi = {psi[0], psi[1]};
    Eigen::Matrix<T, 3, 1> gradient = s1_->getGradientFromSpherical(Psi);

    // Compute contact space point based on normal vector
    ClosedFormContactSpace<T, G1, G2> mink(s1_, s2_, M1_, M2_);
    Eigen::Matrix<T, 3, 1> x_mink = mink.getContactSpaceFromGradient(gradient);

    // Cost function for distance
    Eigen::Matrix<T, 3, 1> point_distance = p0_ - x_mink;

    cost[0] = point_distance(0);
    cost[1] = point_distance(1);
    cost[2] = point_distance(2);

    return true;
}

template <typename G1, typename G2>
DistanceLeastSquares<G1, G2>::DistanceLeastSquares(const Shape3D& s1_shape,
                                                   const Shape3D& s2_shape)
    : DistanceFixedPoint<G1, G2>::DistanceFixedPoint(s1_shape, s2_shape) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceLeastSquares<G1, G2>::DistanceLeastSquares(const G1* s1, const G2* s2)
    : DistanceFixedPoint<G1, G2>::DistanceFixedPoint(s1, s2) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceLeastSquares<G1, G2>::~DistanceLeastSquares() {}

template <typename G1, typename G2>
bool DistanceLeastSquares<G1, G2>::distancePointToCFC() {
    bool status = false;

    // Retrieve spherical coordinate from initial gradient
    this->psi_[0] = std::atan2(
        this->gradient_[2],
        sqrt(pow(this->gradient_[0], 2.0) + pow(this->gradient_[1], 2.0)));
    this->psi_[1] = std::atan2(this->gradient_[1], this->gradient_[0]);

    // Set up the only cost function (also known as residual). This
    // uses auto-differentiation to obtain the derivative (jacobian)
    setCostFunction();
    problem_.AddResidualBlock(cost_, nullptr, this->psi_);

    // Get solutions
    ceres::Solve(options_, &problem_, &summary_);

    Eigen::Vector2d Psi = {this->psi_[0], this->psi_[1]};
    this->setGradient(this->M1_.transpose().inverse() *
                      this->s1_->getGradientFromSpherical(Psi));
    this->normal_ = this->gradient_.normalized();

    if (summary_.termination_type != ceres::NO_CONVERGENCE) {
        status = true;
    }

    this->iter_ = summary_.num_successful_steps;

    return status;
}

template <typename G1, typename G2>
void DistanceLeastSquares<G1, G2>::setCostFunction() {
    cost_ =
        new ceres::AutoDiffCostFunction<CostDistanceLeastSquares<G1, G2>, 3, 2>(
            new CostDistanceLeastSquares<G1, G2>(
                this->s1_, this->s2_, this->M1_, this->M2_, this->s1_to_s2_));
}

template <typename G1, typename G2>
void DistanceLeastSquares<G1, G2>::setupSolver() {
    // Solver options
    options_.minimizer_progress_to_stdout = false;
    options_.logging_type = ceres::SILENT;

    options_.max_num_iterations = 100;
    options_.function_tolerance = 1e-16;
    options_.gradient_tolerance = 1e-16;

    options_.minimizer_type = ceres::TRUST_REGION;
    options_.trust_region_strategy_type = ceres::DOGLEG;
}

}  // namespace collision_cfc
