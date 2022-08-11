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

#include "DistanceFixedPoint.h"

namespace collision_cfc {

template <typename G1, typename G2>
DistanceFixedPoint<G1, G2>::DistanceFixedPoint(const Shape3D& s1_shape,
                                               const Shape3D& s2_shape)
    : DistanceCFC<G1, G2>::DistanceCFC(s1_shape, s2_shape) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceFixedPoint<G1, G2>::DistanceFixedPoint(const G1* s1, const G2* s2)
    : DistanceCFC<G1, G2>::DistanceCFC(s1, s2) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceFixedPoint<G1, G2>::~DistanceFixedPoint() {}

template <typename G1, typename G2>
bool DistanceFixedPoint<G1, G2>::distancePointToCFC() {
    this->setGradient(this->M1_.transpose().inverse() * solve(this->gradient_));
    this->normal_ = this->gradient_.normalized();

    return flag_;
}

template <typename G1, typename G2>
void DistanceFixedPoint<G1, G2>::setupSolver() {
    max_iter_ = 100;
    tol_ = 1e-12;
}

template <typename G1, typename G2>
Eigen::Vector3d DistanceFixedPoint<G1, G2>::solve(
    const Eigen::Vector3d& m_init) {
    // Compute initial step using two normal vectors
    Eigen::Vector3d m_01 = m_init;
    Eigen::Vector3d x_mink =
        this->cfc_obj_->getContactSpaceFromGradient(m_01) + this->s1_center_;
    Eigen::Vector3d m_02 = this->s1_->getGradientFromDirection(Eigen::Vector3d(
        m_01 + this->M1_.transpose() *
                   (this->s2_center_ - x_mink).normalized() * m_01.norm()));

    // Iteratively update normal vector using fixed-point iteration
    Eigen::Vector3d m_new;
    for (this->iter_ = 0; this->iter_ < max_iter_; ++this->iter_) {
        m_new = fixedPointStep(m_01, m_02);

        // Stop when difference with previous step lower than tolerance
        if ((m_new - m_02).norm() < tol_) {
            flag_ = true;
            return m_new;
        }

        m_01 = m_02;
        m_02 = m_new;
    }

    return m_new;
}

template <typename G1, typename G2>
Eigen::Vector3d DistanceFixedPoint<G1, G2>::fixedPointStep(
    const Eigen::Vector3d& m_01, const Eigen::Vector3d& m_02) {
    Eigen::Vector3d x_mink_1 =
        this->M1_.transpose() *
        this->cfc_obj_->getContactSpaceFromGradient(m_01);
    Eigen::Vector3d x_mink_2 =
        this->M1_.transpose() *
        this->cfc_obj_->getContactSpaceFromGradient(m_02);

    Eigen::Vector3d n_01 = m_01.normalized();
    Eigen::Vector3d n_02 = m_02.normalized();

    Eigen::Matrix<double, 1, 1> mag =
        ((n_02 - (n_02.transpose() * n_01) * n_01).transpose() *
         (x_mink_2 - x_mink_1)) /
        ((n_02.cross(n_01)).transpose() * n_02.cross(n_01));
    Eigen::Vector3d r_1 = -mag(0) * n_02;

    Eigen::Vector3d m_1 = this->s1_->getGradientFromDirection(Eigen::Vector3d(
        this->M1_.transpose() * (this->s2_center_ - this->s1_center_) -
        (x_mink_2 + r_1)));

    return m_1;
}

}  // namespace collision_cfc
