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

#include "geometry/GeometryInfo.h"

namespace cfc {

/** \class ClosedFormContactSpace
 * \brief Exact closed-form contact space for general convex bodies
 * parameterized by outward gradient/normal */
template <typename T, typename G1, typename G2>
class ClosedFormContactSpace {
  public:
    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure)
     * \param s2_shape Second object (Shape3D structure)
     * \param M1 Linear transformation of s1
     * \param M2 Linear transformation of s2 */
    ClosedFormContactSpace(const Shape3D& s1_shape, const Shape3D& s2_shape,
                           const Eigen::Matrix3d& M1,
                           const Eigen::Matrix3d& M2);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object
     * \param M1 Linear transformation of s1
     * \param M2 Linear transformation of s2 */
    ClosedFormContactSpace(const G1* s1, const G2* s2,
                           const Eigen::Matrix3d& M1,
                           const Eigen::Matrix3d& M2);

    ~ClosedFormContactSpace();

    /** \brief Set linear transformation for s1
     * \param M1 Linear transformation of s1 */
    void setLinearTransformS1(const Eigen::Matrix3d& M1) { M1_ = M1; }

    /** \brief Set linear transformation for s2
     * \param M2 Linear transformation of s2 */
    void setLinearTransformS2(const Eigen::Matrix3d& M2) { M2_ = M2; }

    /** \brief Get geometric model of s1
     * \return Geometric model */
    G1* getS1() const { return s1_; }

    /** \brief Get geometric model of s2
     * \return Geometric model */
    G2* getS2() const { return s2_; }

    /** \brief Get contact space from gradient parameterization
     * \param m1 Gradient at surface point on s1
     * \return Surface point on the contact space */
    Eigen::Matrix<T, 3, 1> getContactSpaceFromGradient(
        const Eigen::Matrix<T, 3, 1>& m1) {
        return computeContactSpaceFromGradient(m1);
    }

    /** \brief Get contact space from normal parameterization
     * \param n1 Normal vector at surface point on s1
     * \return Point on contact space */
    Eigen::Matrix<T, 3, 1> getContactSpaceFromNormal(
        const Eigen::Matrix<T, 3, 1>& n1) {
        return computeContactSpaceFromNormal(n1.normalized());
    }

    /** \brief Compute contact space from angular parameterization
     * \param Psi Angular parameter for the surface point on s1
     * \return Point on contact space */
    Eigen::Matrix<T, 3, 1> getContactSpaceFromSpherical(
        const Eigen::Matrix<T, 2, 1>& Psi) {
        Eigen::Matrix<T, 3, 1> m1 = s1_->getGradientFromSpherical(Psi);
        return computeContactSpaceFromGradient(m1);
    }

  private:
    /** \brief Compute contact space from gradient parameterization
     * \param m1 Gradient at surface point on s1
     * \return Surface point on the contact space */
    Eigen::Matrix<T, 3, 1> computeContactSpaceFromGradient(
        const Eigen::Matrix<T, 3, 1>& m1);

    /** \brief Compute |m2| = Phi(m1)
     * \param m1 Gradient at surface point on s1
     * \return Norm of surface point at s2 as a function of gradient at the same
     * contact point but viewed from s1 */
    T Phi(const Eigen::Matrix<T, 3, 1>& m1);

    /** \brief Compute contact space from normal parameterization
     * \param n1 Normal vector at surface point on s1
     * \return Point on contact space */
    Eigen::Matrix<T, 3, 1> computeContactSpaceFromNormal(
        const Eigen::Matrix<T, 3, 1>& n1);

    /** \brief Geometric model of s1 */
    const G1* s1_;

    /** \brief Geometric model of s2 */
    const G2* s2_;

    /** \brief Linear transformation of s1 */
    Eigen::Matrix3d M1_;

    /** \brief Linear transformation of s2 */
    Eigen::Matrix3d M2_;
};

#include "ClosedFormContactSpace-inl.h"

}  // namespace cfc
