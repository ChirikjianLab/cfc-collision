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

#include "ClosedFormContactSpace.h"

template <typename T, typename G1, typename G2>
ClosedFormContactSpace<T, G1, G2>::ClosedFormContactSpace(
    const Shape3D& s1_shape, const Shape3D& s2_shape, const Eigen::Matrix3d& M1,
    const Eigen::Matrix3d& M2)
    : M1_(M1), M2_(M2) {
    s1_ = new G1(s1_shape);
    s2_ = new G2(s2_shape);
}

template <typename T, typename G1, typename G2>
ClosedFormContactSpace<T, G1, G2>::ClosedFormContactSpace(
    const G1* s1, const G2* s2, const Eigen::Matrix3d& M1,
    const Eigen::Matrix3d& M2)
    : s1_(s1), s2_(s2), M1_(M1), M2_(M2) {}

template <typename T, typename G1, typename G2>
ClosedFormContactSpace<T, G1, G2>::~ClosedFormContactSpace() {}

template <typename T, typename G1, typename G2>
Eigen::Matrix<T, 3, 1>
ClosedFormContactSpace<T, G1, G2>::computeContactSpaceFromGradient(
    const Eigen::Matrix<T, 3, 1>& m1) {
    // Compute f_i(m_1)
    Eigen::Matrix<T, 3, 1> f_m1 = s1_->getBoundaryFromGradient(m1);

    Eigen::Matrix<T, 3, 1> m1_t =
        M2_.transpose() * (M1_.transpose().inverse() * m1);
    T phi_m1_t = Phi(-m1_t);

    Eigen::Matrix<T, 3, 1> f_m2 = s2_->getBoundaryFromGradient(
        Eigen::Matrix<T, 3, 1>(-phi_m1_t * m1_t.normalized()));

    return M1_ * f_m1 - M2_ * f_m2;
}

template <typename T, typename G1, typename G2>
T ClosedFormContactSpace<T, G1, G2>::Phi(const Eigen::Matrix<T, 3, 1>& m1) {
    Eigen::Matrix<T, 3, 1> g2_m1_inv = s2_->getHypersphereFromGradient(m1);
    Eigen::Matrix<T, 3, 1> m2 =
        s2_->getGradientFromHypersphere(g2_m1_inv.normalized());

    return m2.norm();
}

template <typename T, typename G1, typename G2>
Eigen::Matrix<T, 3, 1>
ClosedFormContactSpace<T, G1, G2>::computeContactSpaceFromNormal(
    const Eigen::Matrix<T, 3, 1>& n1) {
    // Compute f_i(n_1)
    Eigen::Matrix<T, 3, 1> f_n1 =
        s1_->getBoundaryFromNormal(Eigen::Matrix<T, 3, 1>(n1));

    Eigen::Matrix<T, 3, 1> n1_t =
        M2_.transpose() * (M1_.transpose().inverse() * n1);
    Eigen::Matrix<T, 3, 1> f_n2 =
        s2_->getBoundaryFromNormal(Eigen::Matrix<T, 3, 1>(-n1_t));

    return M1_ * f_n1 - M2_ * f_n2;
}
