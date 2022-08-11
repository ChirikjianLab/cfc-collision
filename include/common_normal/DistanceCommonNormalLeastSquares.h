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

#include "ceres/ceres.h"
#include "glog/logging.h"

namespace collision_common_normal {

/** \class CostCommonNormalLeastSquares
 * \brief Computing common normal cost function for distance query between two
 * convex bodies with normal-param surfaces
 *
 * Reference: Gonçalves, A.A., Bernardino, A., Jorge, J. and Lopes, D.S.,
 * 2017. A benchmark study on accuracy-controlled distance calculation between
 * superellipsoid and superovoid contact geometries. Mechanism and Machine
 * Theory, 115, pp.77-96. */
template <typename G1, typename G2>
class CostCommonNormalLeastSquares {
  public:
    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object
     * \param M1 Linear transformation of s1
     * \param M2 Linear transformation of s2
     * \param s1_center Center of s1
     * \param s2_center Center of s2 */
    CostCommonNormalLeastSquares(const G1* s1, const G2* s2,
                                 const Eigen::Matrix3d& M1,
                                 const Eigen::Matrix3d& M2,
                                 const Eigen::Vector3d& s1_center,
                                 const Eigen::Vector3d& s2_center);

    ~CostCommonNormalLeastSquares();

    /** \brief Cost function */
    template <typename T>
    bool operator()(const T* const psi, T* cost) const;

  private:
    const G1* s1_;
    const G2* s2_;

    Eigen::Matrix3d M1_;
    Eigen::Matrix3d M2_;
    Eigen::Vector3d s1_center_;
    Eigen::Vector3d s2_center_;
};

/** \class DistanceCommonNormalLeastSquares
 * \brief Narrow phase proximity queries between two 3D convex bodies with
 * smooth boundaries using normal parameterization. Formulated as least-squares
 * optimization */
template <typename G1, typename G2>
class DistanceCommonNormalLeastSquares : public DistanceCommonNormal<G1, G2> {
  public:
    /** \brief Constructor
     * \param s1_shape First object (cfc::Shape3D structure)
     * \param s2_shape Second object (cfc::Shape3D structure) */
    DistanceCommonNormalLeastSquares(const cfc::Shape3D& s1_shape,
                                     const cfc::Shape3D& s2_shape);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object */
    DistanceCommonNormalLeastSquares(const G1* s1, const G2* s2);

    virtual ~DistanceCommonNormalLeastSquares();

  protected:
    bool normalOptimization() override;

  private:
    /** \brief Setup optimization solver */
    void setupSolver();

    ceres::Problem problem_;
    ceres::CostFunction* cost_;
    ceres::Solver::Options options_;
    ceres::Solver::Summary summary_;

    double psi_[4];
};

}  // namespace collision_common_normal

#include "DistanceCommonNormalLeastSquares-inl.h"
