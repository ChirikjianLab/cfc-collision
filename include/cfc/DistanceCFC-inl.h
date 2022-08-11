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

#include "DistanceCFC.h"

namespace collision_cfc {

template <typename G1, typename G2>
DistanceCFC<G1, G2>::DistanceCFC(const Shape3D& s1_shape,
                                 const Shape3D& s2_shape)
    : DistanceSolver<G1, G2>::DistanceSolver(s1_shape, s2_shape) {
    // Setup CFC object
    setCFC();

    // Initial guess
    initialization();
}

template <typename G1, typename G2>
DistanceCFC<G1, G2>::DistanceCFC(const G1* s1, const G2* s2)
    : DistanceSolver<G1, G2>::DistanceSolver(s1, s2) {
    // Setup CFC object
    setCFC();

    // Initial guess
    initialization();
}

template <typename G1, typename G2>
DistanceCFC<G1, G2>::~DistanceCFC() {}

template <typename G1, typename G2>
void DistanceCFC<G1, G2>::query() {
    // Solve for the point-to-MinkSum optimization problem
    distancePointToCFC();

    // Get results and optimality condition
    // Obtain the solved optimal points on s1 and MinkSum boundary
    computeOptimalCFC();
    results();
    necessaryCondition(this->normal_);
}

template <typename G1, typename G2>
void DistanceCFC<G1, G2>::setCFC() {
    cfc_obj_ = new ClosedFormContactSpace<double, G1, G2>(this->s1_, this->s2_,
                                                          this->M1_, this->M2_);
}

template <typename G1, typename G2>
void DistanceCFC<G1, G2>::initialization() {
    // Initial guess for spherical coordinates: s2 center as view in s1
    psi_[0] = std::atan2(this->s2_center_in_s1_[2],
                         sqrt(pow(this->s2_center_in_s1_[0], 2.0) +
                              pow(this->s2_center_in_s1_[1], 2.0)));
    psi_[1] = std::atan2(this->s2_center_in_s1_[1], this->s2_center_in_s1_[0]);

    // Initial guess for gradient/normal: along direction of s2 center as
    // view in s1, in global coordinates
    this->gradient_ = this->s1_->getGradientFromSpherical(
        Eigen::Vector2d({psi_[0], psi_[1]}));
    this->normal_ = this->gradient_.normalized();
}

template <typename G1, typename G2>
void DistanceCFC<G1, G2>::results() {
    // Optimal normal vector
    this->res_.optimal_normal = this->normal_;

    // Collision status, distance and closes points
    this->res_.closest_point_s2 =
        this->res_.closest_point_s1 +
        (this->s2_center_ - this->res_.point_on_contact_space);
    this->res_.distance =
        (this->res_.closest_point_s2 - this->res_.closest_point_s1).norm();

    if ((this->normal_).transpose() *
            (this->res_.closest_point_s2 - this->res_.closest_point_s1) >
        0.0) {
        this->res_.is_collision = false;
    } else {
        this->res_.is_collision = true;
        this->res_.distance = -this->res_.distance;
    }

    // Number of iterations
    this->res_.num_iteration = this->iter_;
}

template <typename G1, typename G2>
void DistanceCFC<G1, G2>::computeOptimalCFC() {
    this->res_.closest_point_s1 =
        this->M1_ * this->s1_->getBoundaryFromGradient(Eigen::Vector3d(
                        this->M1_.transpose() * this->gradient_)) +
        this->s1_center_;
    this->res_.point_on_contact_space =
        this->cfc_obj_->getContactSpaceFromGradient(
            Eigen::Vector3d(this->M1_.transpose() * this->gradient_)) +
        this->s1_center_;
}

template <typename G1, typename G2>
void DistanceCFC<G1, G2>::necessaryCondition(const Eigen::Vector3d& normal) {
    this->res_.necessary_condition = (normal.cross(this->res_.closest_point_s2 -
                                                   this->res_.closest_point_s1))
                                         .norm();
}

}  // namespace collision_cfc
