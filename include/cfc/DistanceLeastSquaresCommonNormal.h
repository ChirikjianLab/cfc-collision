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

namespace cfc {

namespace collision_cfc {

/** \class CostLeastSquaresCommonNormal
 * \brief Computing closed-form contact space and cost function for distance
 * query between two convex bodies with gradient-param surfaces, using common
 * normal */
template <typename G1, typename G2>
class CostLeastSquaresCommonNormal {
  public:
    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object
     * \param M1 Linear transformation of s1
     * \param M2 Linear transformation of s2
     * \param p0 Vector pointing from s1 center to s2 center */
    CostLeastSquaresCommonNormal(const G1* s1, const G2* s2,
                                 const Eigen::Matrix3d& M1,
                                 const Eigen::Matrix3d& M2,
                                 const Eigen::Vector3d& p0);

    ~CostLeastSquaresCommonNormal();

  public:
    /** \brief Cost function */
    template <typename T>
    bool operator()(const T* const psi, T* cost) const;

  private:
    const G1* s1_;
    const G2* s2_;

    const Eigen::Matrix3d& M1_;
    const Eigen::Matrix3d& M2_;
    const Eigen::Vector3d& p0_;
};

/** \class DistanceLeastSquaresCommonNormal
 * \brief Narrow phase proximity queries between two 3D convex bodies with
 * smooth boundaries using gradient parameterization and common normal concept.
 * Formulated as least-squares optimization. */
template <typename G1, typename G2>
class DistanceLeastSquaresCommonNormal : public DistanceLeastSquares<G1, G2> {
  public:
    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure)
     * \param s2_shape Second object (Shape3D structure) */
    DistanceLeastSquaresCommonNormal(const Shape3D& s1_shape,
                                     const Shape3D& s2_shape);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object */
    DistanceLeastSquaresCommonNormal(const G1* s1, const G2* s2);

    virtual ~DistanceLeastSquaresCommonNormal();

  protected:
    virtual void setCostFunction() override;
};

}  // namespace collision_cfc

#include "DistanceLeastSquaresCommonNormal-inl.h"

}  // namespace cfc
