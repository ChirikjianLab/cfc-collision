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

#include "ContinuousDistanceCFC.h"

namespace collision_cfc {

template <typename G1, typename G2>
ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(
    G1* s1, const Eigen::Affine3d& g1_start, const Eigen::Affine3d& g1_goal,
    G1* s2, const Eigen::Affine3d& g2_start, const Eigen::Affine3d& g2_goal,
    const ContinuousDistanceRequest& req)
    : s1_(s1),
      s2_(s2),
      g1_start_(g1_start),
      g2_start_(g2_start),
      g1_goal_(g1_goal),
      g2_goal_(g2_goal),
      req_(req) {
    setup();
}

template <typename G1, typename G2>
ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(
    const Shape3D& s1_shape, const Eigen::Affine3d& g1_start,
    const Eigen::Affine3d& g1_goal, const Shape3D& s2_shape,
    const Eigen::Affine3d& g2_start, const Eigen::Affine3d& g2_goal,
    const ContinuousDistanceRequest& req)
    : s1_(new G1(s1_shape)),
      s2_(new G2(s2_shape)),
      g1_start_(g1_start),
      g2_start_(g2_start),
      g1_goal_(g1_goal),
      g2_goal_(g2_goal),
      req_(req) {
    setup();
}

template <typename G1, typename G2>
ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(
    G1* s1, const Eigen::Affine3d& g1_goal, G2* s2,
    const Eigen::Affine3d& g2_goal, const ContinuousDistanceRequest& req)
    : s1_(s1), s2_(s2), g1_goal_(g1_goal), g2_goal_(g2_goal), req_(req) {
    setStartPoses();
    setup();
}

template <typename G1, typename G2>
ContinuousDistanceCFC<G1, G2>::ContinuousDistanceCFC(
    const Shape3D& s1_shape, const Eigen::Affine3d& g1_goal,
    const Shape3D& s2_shape, const Eigen::Affine3d& g2_goal,
    const ContinuousDistanceRequest& req)
    : s1_(new G1(s1_shape)),
      s2_(new G2(s2_shape)),
      g1_goal_(g1_goal),
      g2_goal_(g2_goal),
      req_(req) {
    setStartPoses();
    setup();
}

template <typename G1, typename G2>
ContinuousDistanceCFC<G1, G2>::~ContinuousDistanceCFC() {}

template <typename G1, typename G2>
void ContinuousDistanceCFC<G1, G2>::query() {
    // Initial guess
    initialization();

    // Solve for the point-to-MinkSum optimization problem
    distancePointToCFC();

    // Update optimal poses of s1 and s2
    g1_opt_ = updatePoseAtTime(t_, &g1_start_, &g1_goal_, req_.motion_type);
    g2_opt_ = updatePoseAtTime(t_, &g2_start_, &g2_goal_, req_.motion_type);

    // Get results and optimality condition
    // Obtain the solved optimal points on s1 and MinkSum boundary
    computeOptimalCFC();
    results();
    necessaryCondition(res_.optimal_normal);
}

template <typename G1, typename G2>
void ContinuousDistanceCFC<G1, G2>::setup() {
    s2_center_in_s1_ = g1_start_.linear().transpose() *
                       (g2_start_.translation() - g1_start_.translation());

    cfc_obj_ = new ClosedFormContactSpace<double, G1, G2>(
        s1_, s2_, g1_start_.linear(), g2_start_.linear());
}

template <typename G1, typename G2>
void ContinuousDistanceCFC<G1, G2>::setStartPoses() {
    g1_start_.linear() = s1_->quaternion().toRotationMatrix();
    g2_start_.linear() = s2_->quaternion().toRotationMatrix();
    g1_start_.translation() =
        Eigen::Vector3d({s1_->pos()[0], s1_->pos()[1], s1_->pos()[2]});
    g2_start_.translation() =
        Eigen::Vector3d({s2_->pos()[0], s2_->pos()[1], s2_->pos()[2]});
}

template <typename G1, typename G2>
void ContinuousDistanceCFC<G1, G2>::initialization() {
    // Initial guess for spherical coordinates: s2 center as view in s1
    spherical_(0) = std::atan2(s2_center_in_s1_[2],
                               std::sqrt(std::pow(s2_center_in_s1_[0], 2.0) +
                                         std::pow(s2_center_in_s1_[1], 2.0)));
    spherical_(1) = std::atan2(s2_center_in_s1_[1], s2_center_in_s1_[0]);

    // Initial guess for gradient/normal: along direction of s2 center as
    // view in s1, in local coordinates
    setVariables(s1_->getGradientFromSpherical(spherical_), 0.0);
}

template <typename G1, typename G2>
void ContinuousDistanceCFC<G1, G2>::results() {
    // Optimal time and normal vector
    res_.optimal_time = t_;
    res_.optimal_normal = g1_opt_.linear().transpose().inverse() * normal_;

    // Collision status, distance and closes points
    res_.closest_point_s2 =
        res_.closest_point_s1 +
        (g2_opt_.translation() - res_.point_on_contact_space);
    res_.distance = (res_.closest_point_s2 - res_.closest_point_s1).norm();

    if (res_.optimal_normal.transpose() *
                (res_.closest_point_s2 - res_.closest_point_s1) >
            0.0 &&
        res_.distance > 1e-3) {
        res_.is_collision = false;
    } else {
        res_.is_collision = true;
        res_.distance = -res_.distance;
    }

    // Number of iterations
    res_.num_iteration = iter_;
}

template <typename G1, typename G2>
void ContinuousDistanceCFC<G1, G2>::necessaryCondition(
    const Eigen::Vector3d& normal) {
    res_.necessary_condition =
        (normal.cross(res_.closest_point_s2 - res_.closest_point_s1)).norm();
}

}  // namespace collision_cfc
