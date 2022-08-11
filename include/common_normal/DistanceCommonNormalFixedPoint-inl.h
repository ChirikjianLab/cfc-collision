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

#include "DistanceCommonNormalFixedPoint.h"

namespace collision_common_normal {

template <typename G1, typename G2>
DistanceCommonNormalFixedPoint<G1, G2>::DistanceCommonNormalFixedPoint(
    const cfc::Shape3D& s1_shape, const cfc::Shape3D& s2_shape)
    : DistanceCommonNormal<G1, G2>::DistanceCommonNormal(s1_shape, s2_shape) {}

template <typename G1, typename G2>
DistanceCommonNormalFixedPoint<G1, G2>::DistanceCommonNormalFixedPoint(
    const G1* s1, const G2* s2)
    : DistanceCommonNormal<G1, G2>::DistanceCommonNormal(s1, s2) {}

template <typename G1, typename G2>
DistanceCommonNormalFixedPoint<G1, G2>::~DistanceCommonNormalFixedPoint() {}

template <typename G1, typename G2>
bool DistanceCommonNormalFixedPoint<G1, G2>::normalOptimization() {
    const int max_iter = 100;
    const double tol = 1e-12;
    bool flag = false;
    int iter = 0;

    this->normal_ = solve(this->normal_, max_iter, tol, &iter, &flag);

    this->res_.num_iteration = iter;

    return flag;
}

template <typename G1, typename G2>
Eigen::Vector3d DistanceCommonNormalFixedPoint<G1, G2>::solve(
    const Eigen::Vector3d& n_init, const int max_iter, const double tol,
    int* iter, bool* flag) {
    // Compute initial step using two normal vectors
    Eigen::Vector3d n_01 = n_init;
    Eigen::Vector3d r_PQ =
        (this->M1_ * this->s2_->getBoundaryFromNormal(
                         Eigen::Vector3d(-this->M1_.transpose() * n_01)) +
         this->s2_center_) -
        (this->M2_ * this->s1_->getBoundaryFromNormal(
                         Eigen::Vector3d(this->M1_.transpose() * n_01)) +
         this->s1_center_);
    Eigen::Vector3d n_02 = (n_01 + r_PQ.normalized()).normalized();
    Eigen::Vector3d n_new = n_02;

    // Iteratively update normal vector using fixed-point iteration
    for (*iter = 0; *iter < max_iter; ++*iter) {
        n_new = fixedPointStep(n_01, n_02);

        // Stop when difference with previous step lower than tolerance
        if ((n_new - n_02).norm() < tol) {
            *flag = true;
            return n_new;
        }

        n_01 = n_02;
        n_02 = n_new;
    }

    return n_new;
}

template <typename G1, typename G2>
Eigen::Vector3d DistanceCommonNormalFixedPoint<G1, G2>::fixedPointStep(
    const Eigen::Vector3d& n_01, const Eigen::Vector3d& n_02) {
    Eigen::Vector3d r_MP_01 =
        this->M1_ * this->s1_->getBoundaryFromNormal(
                        Eigen::Vector3d(this->M1_.transpose() * n_01));
    Eigen::Vector3d r_MP_02 =
        this->M1_ * this->s1_->getBoundaryFromNormal(
                        Eigen::Vector3d(this->M1_.transpose() * n_02));
    Eigen::Vector3d r_SQ_01 =
        this->M2_ * this->s2_->getBoundaryFromNormal(
                        Eigen::Vector3d(-this->M2_.transpose() * n_01));
    Eigen::Vector3d r_SQ_02 =
        this->M2_ * this->s2_->getBoundaryFromNormal(
                        Eigen::Vector3d(-this->M2_.transpose() * n_02));

    Eigen::Matrix<double, 1, 1> mag_P_1 =
        ((n_02 - (n_02.transpose() * n_01) * n_01).transpose() *
         (r_MP_02 - r_MP_01)) /
        ((n_02.cross(n_01)).transpose() * n_02.cross(n_01));
    Eigen::Vector3d r_P_1 = -mag_P_1(0) * n_02;
    Eigen::Matrix<double, 1, 1> mag_Q_1 =
        ((n_02 - (n_02.transpose() * n_01) * n_01).transpose() *
         (r_SQ_02 - r_SQ_01)) /
        ((n_02.cross(n_01)).transpose() * n_02.cross(n_01));
    Eigen::Vector3d r_Q_1 = -mag_Q_1(0) * n_02;

    Eigen::Vector3d n_1 = ((this->s2_center_ - this->s1_center_) -
                           (r_MP_02 + r_P_1) + (r_SQ_02 + r_Q_1))
                              .normalized();

    return n_1;
}

}  // namespace collision_common_normal
