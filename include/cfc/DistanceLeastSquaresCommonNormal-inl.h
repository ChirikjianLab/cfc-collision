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

#include "DistanceLeastSquaresCommonNormal.h"

namespace collision_cfc {

template <typename G1, typename G2>
CostLeastSquaresCommonNormal<G1, G2>::CostLeastSquaresCommonNormal(
    const G1* s1, const G2* s2, const Eigen::Matrix3d& M1,
    const Eigen::Matrix3d& M2, const Eigen::Vector3d& p0)
    : s1_(s1), s2_(s2), M1_(M1), M2_(M2), p0_(p0) {}

template <typename G1, typename G2>
CostLeastSquaresCommonNormal<G1, G2>::~CostLeastSquaresCommonNormal() {}

template <typename G1, typename G2>
template <typename T>
bool CostLeastSquaresCommonNormal<G1, G2>::operator()(const T* const psi,
                                                      T* cost) const {
    // Compute normal vector
    Eigen::Matrix<T, 2, 1> Psi = {psi[0], psi[1]};
    Eigen::Matrix<T, 3, 1> gradient = s1_->getGradientFromSpherical(Psi);

    // Compute contact space point based on normal vector
    ClosedFormContactSpace<T, G1, G2> mink(s1_, s2_, M1_, M2_);
    Eigen::Matrix<T, 3, 1> x_mink = mink.getContactSpaceFromGradient(gradient);

    // Cost function for distance
    Eigen::Matrix<T, 3, 1> gradient_point_parallel =
        (M1_.transpose().inverse() * gradient).cross(p0_ - x_mink);

    cost[0] = gradient_point_parallel(0);
    cost[1] = gradient_point_parallel(1);
    cost[2] = gradient_point_parallel(2);

    return true;
}

template <typename G1, typename G2>
DistanceLeastSquaresCommonNormal<G1, G2>::DistanceLeastSquaresCommonNormal(
    const Shape3D& s1_shape, const Shape3D& s2_shape)
    : DistanceLeastSquares<G1, G2>::DistanceLeastSquares(s1_shape, s2_shape) {}

template <typename G1, typename G2>
DistanceLeastSquaresCommonNormal<G1, G2>::DistanceLeastSquaresCommonNormal(
    const G1* s1, const G2* s2)
    : DistanceLeastSquares<G1, G2>::DistanceLeastSquares(s1, s2) {}

template <typename G1, typename G2>
DistanceLeastSquaresCommonNormal<G1, G2>::~DistanceLeastSquaresCommonNormal() {}

template <typename G1, typename G2>
void DistanceLeastSquaresCommonNormal<G1, G2>::setCostFunction() {
    this->cost_ =
        new ceres::AutoDiffCostFunction<CostLeastSquaresCommonNormal<G1, G2>, 3,
                                        2>(
            new CostLeastSquaresCommonNormal<G1, G2>(
                this->s1_, this->s2_, this->M1_, this->M2_, this->s1_to_s2_));
}

}  // namespace collision_cfc
