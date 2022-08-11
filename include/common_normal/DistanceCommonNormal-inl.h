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

#include "DistanceCommonNormal.h"

namespace collision_common_normal {

template <typename G1, typename G2>
DistanceCommonNormal<G1, G2>::DistanceCommonNormal(const cfc::Shape3D& s1_shape,
                                                   const cfc::Shape3D& s2_shape)
    : cfc::DistanceSolver<G1, G2>::DistanceSolver(s1_shape, s2_shape) {}

template <typename G1, typename G2>
DistanceCommonNormal<G1, G2>::DistanceCommonNormal(const G1* s1, const G2* s2)
    : cfc::DistanceSolver<G1, G2>::DistanceSolver(s1, s2) {}

template <typename G1, typename G2>
DistanceCommonNormal<G1, G2>::~DistanceCommonNormal() {}

template <typename G1, typename G2>
void DistanceCommonNormal<G1, G2>::query() {
    // Initial condition for normal vector
    initialization();

    // Optimize normal vectors to be anti-parallel
    normalOptimization();

    // Store results
    this->res_.closest_point_s1 =
        this->M1_ * this->s1_->getBoundaryFromNormal(Eigen::Vector3d(
                        this->M1_.transpose() * this->normal_)) +
        this->s1_center_;
    this->res_.closest_point_s2 =
        this->M2_ * this->s2_->getBoundaryFromNormal(Eigen::Vector3d(
                        -this->M2_.transpose() * this->normal_)) +
        this->s2_center_;
    this->res_.distance =
        (this->res_.closest_point_s2 - this->res_.closest_point_s1).norm();

    if (this->normal_.transpose() *
            (this->res_.closest_point_s2 - this->res_.closest_point_s1) >
        0.0) {
        this->res_.is_collision = false;
    } else {
        this->res_.is_collision = true;
        this->res_.distance = -this->res_.distance;
    }

    // Necessary condition for minimum distance query: normal at closest
    // point on s1 should be colinear with the line connecting two closest
    // points Gradient at closest point
    this->res_.necessary_condition =
        (this->normal_.cross(this->res_.closest_point_s2 -
                             this->res_.closest_point_s1))
            .norm();

    this->res_.optimal_normal = this->normal_;
}

template <typename G1, typename G2>
void DistanceCommonNormal<G1, G2>::initialization() {
    this->normal_ = this->s1_to_s2_.normalized();
}

}  // namespace collision_common_normal
