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

#include "ContinuousDistanceLeastSquares.h"

namespace collision_cfc {

template <typename G1, typename G2>
CostContinuousLeastSquares<G1, G2>::CostContinuousLeastSquares(
    ClosedFormContactSpace<double, G1, G2>* cfc_obj,
    const Eigen::Affine3d* g1_start, const Eigen::Affine3d* g1_goal,
    const Eigen::Affine3d* g2_start, const Eigen::Affine3d* g2_goal,
    const CCD_MOTION_TYPE& motion_type)
    : cfc_obj_(cfc_obj),
      g1_start_(g1_start),
      g2_start_(g2_start),
      g1_goal_(g1_goal),
      g2_goal_(g2_goal),
      motion_type_(motion_type) {}

template <typename G1, typename G2>
CostContinuousLeastSquares<G1, G2>::~CostContinuousLeastSquares() {}

template <typename G1, typename G2>
bool CostContinuousLeastSquares<G1, G2>::operator()(const double* psi,
                                                    const double* t,
                                                    double* cost) const {
    // Update the current poses
    const Eigen::Affine3d g1_current =
        updatePoseAtTime<double>(t[0], g1_start_, g1_goal_, motion_type_);
    const Eigen::Affine3d g2_current =
        updatePoseAtTime<double>(t[0], g2_start_, g2_goal_, motion_type_);

    cfc_obj_->setLinearTransformS1(g1_current.linear());
    cfc_obj_->setLinearTransformS2(g2_current.linear());

    Eigen::Vector3d x_mink = cfc_obj_->getContactSpaceFromSpherical(
        Eigen::Vector2d({psi[0], psi[1]}));
    Eigen::Vector3d p0 = g2_current.translation() - g1_current.translation();

    // Cost function for distance
    cost[0] = p0(0) - x_mink(0);
    cost[1] = p0(1) - x_mink(1);
    cost[2] = p0(2) - x_mink(2);

    return true;
}

template <typename G1, typename G2>
ContinuousDistanceLeastSquares<G1, G2>::ContinuousDistanceLeastSquares(
    const Shape3D& s1_shape, const Eigen::Affine3d& g1_start,
    const Eigen::Affine3d& g1_goal, const Shape3D& s2_shape,
    const Eigen::Affine3d& g2_start, const Eigen::Affine3d& g2_goal,
    const ContinuousDistanceRequest& req)
    : ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(
          s1_shape, g1_start, g1_goal, s2_shape, g2_start, g2_goal, req) {
    setupSolver();
}

template <typename G1, typename G2>
ContinuousDistanceLeastSquares<G1, G2>::ContinuousDistanceLeastSquares(
    const G1* s1, const Eigen::Affine3d& g1_start,
    const Eigen::Affine3d& g1_goal, const G1* s2,
    const Eigen::Affine3d& g2_start, const Eigen::Affine3d& g2_goal,
    const ContinuousDistanceRequest& req)
    : ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(
          s1, g1_start, g1_goal, s2, g2_start, g2_goal, req) {
    setupSolver();
}

template <typename G1, typename G2>
ContinuousDistanceLeastSquares<G1, G2>::ContinuousDistanceLeastSquares(
    const Shape3D& s1_shape, const Eigen::Affine3d& g1_goal,
    const Shape3D& s2_shape, const Eigen::Affine3d& g2_goal,
    const ContinuousDistanceRequest& req)
    : ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(
          s1_shape, g1_goal, s2_shape, g2_goal, req) {
    setupSolver();
}

template <typename G1, typename G2>
ContinuousDistanceLeastSquares<G1, G2>::ContinuousDistanceLeastSquares(
    const G1* s1, const Eigen::Affine3d& g1_goal, const G1* s2,
    const Eigen::Affine3d& g2_goal, const ContinuousDistanceRequest& req)
    : ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(s1, g1_goal, s2,
                                                           g2_goal, req) {
    setupSolver();
}

template <typename G1, typename G2>
ContinuousDistanceLeastSquares<G1, G2>::~ContinuousDistanceLeastSquares() {}

template <typename G1, typename G2>
bool ContinuousDistanceLeastSquares<G1, G2>::distancePointToCFC() {
    bool status = false;

    // Set up the only cost function (also known as residual). This
    // uses numeric-differentiation to obtain the derivative (jacobian)
    cost_ =
        new ceres::NumericDiffCostFunction<CostContinuousLeastSquares<G1, G2>,
                                           ceres::CENTRAL, 3, 2, 1>(
            new CostContinuousLeastSquares<G1, G2>(
                this->cfc_obj_, &this->g1_start_, &this->g1_goal_,
                &this->g2_start_, &this->g2_goal_, this->req_.motion_type));

    double psi[2]{this->spherical_(0), this->spherical_(1)};
    problem_.AddResidualBlock(cost_, nullptr, psi, &this->t_);

    // Get solutions
    ceres::Solve(options_, &problem_, &summary_);

    this->spherical_ = {psi[0], psi[1]};
    Eigen::Vector3d m = this->s1_->getGradientFromSpherical(this->spherical_);
    this->setVariables(m, this->t_);

    if (summary_.termination_type != ceres::NO_CONVERGENCE) {
        status = true;
    }

    this->iter_ = summary_.num_successful_steps;

    return status;
}

template <typename G1, typename G2>
void ContinuousDistanceLeastSquares<G1, G2>::computeOptimalCFC() {
    // Compute optimal witness point on s1
    this->res_.closest_point_s1 =
        this->g1_opt_.linear() *
            this->s1_->getBoundaryFromGradient(this->gradient_) +
        this->g1_opt_.translation();

    // Compute optimal point on contact space
    this->cfc_obj_->setLinearTransformS1(this->g1_opt_.linear());
    this->cfc_obj_->setLinearTransformS2(this->g2_opt_.linear());

    this->res_.point_on_contact_space =
        this->cfc_obj_->getContactSpaceFromGradient(this->gradient_) +
        this->g1_opt_.translation();
}
template <typename G1, typename G2>
void ContinuousDistanceLeastSquares<G1, G2>::setupSolver() {
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
