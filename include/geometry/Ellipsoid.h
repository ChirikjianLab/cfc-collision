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

#include "SuperQuadrics.h"

namespace cfc {

/** \class Ellipsoid
 * \brief Definitions of shape and operations for ellipsoidal surface */
class Ellipsoid : public SuperQuadrics {
  public:
    /** \brief Constructor
     * \param shape Shape of the ellipsoid model */
    Ellipsoid(const Shape3D& shape) : SuperQuadrics(shape) {
        // Make sure the exponents equal 1
        shape_.eps[0] = 1.0;
        shape_.eps[1] = 1.0;

        A_.diagonal() << shape_.a[0], shape_.a[1], shape_.a[2];
    }

    ~Ellipsoid() {}

  public:
    /** \brief Compute implicit function value
     *  \param x 3x1 vector for a point in Euclidean space
     * \return Scalar value for implicit function */
    template <typename T>
    T getImplicitFunction(const Eigen::Matrix<T, 3, 1>& x) const {
        T implicit = x.transpose() *
                         (A_.toDenseMatrix() * A_.toDenseMatrix()).inverse() *
                         x -
                     1;

        return implicit;
    }

    /** \brief Get boundary point from angles
     *  \param psi 2x1 vector for angular parameter
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromSpherical(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 3, 1> u = getHypersphereFromSpherical(psi);
        Eigen::Matrix<T, 3, 1> x = A_.toDenseMatrix() * u;

        return x;
    }

    /** \brief Get boundary point from gradient vector
     *  \param m 3x1 vector of surface gradient
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromGradient(
        const Eigen::Matrix<T, 3, 1>& m) const {
        Eigen::Matrix<T, 3, 1> x =
            0.5 * (A_.toDenseMatrix() * A_.toDenseMatrix()) * m;

        return x;
    }

    /** \brief Get boundary point from normal vector
     *  \param n 3x1 vector of surface normal
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromNormal(
        const Eigen::Matrix<T, 3, 1>& n) const {
        Eigen::Matrix<T, 3, 1> x = (A_.toDenseMatrix() * A_.toDenseMatrix()) *
                                   n / (A_.toDenseMatrix() * n).norm();

        return x;
    }

    /** \brief Get surface gradient from spherical coordinates
     *  \param psi 2x1 vector for angular parameter
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromSpherical(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 3, 1> u = getHypersphereFromSpherical(psi);
        Eigen::Matrix<T, 3, 1> m = getGradientFromHypersphere(u);

        return m;
    }

    /** \brief Get surface gradient from surface point
     *  \param x 3x1 vector of surface point
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromCartesian(
        const Eigen::Matrix<T, 3, 1>& x) const {
        Eigen::Matrix<T, 3, 1> m =
            2.0 * (A_.toDenseMatrix() * A_.toDenseMatrix()).inverse() * x;

        return m;
    }

    /** \brief Get surface gradient from hypersphere
     *  \param u 3x1 vector of hypersphere
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromHypersphere(
        const Eigen::Matrix<T, 3, 1>& u) const {
        Eigen::Matrix<T, 3, 1> m = 2.0 * A_.toDenseMatrix().inverse() * u;

        return m;
    }

    /** \brief Get surface gradient from directional vector
     *  \param d 3x1 vector indicating direction, normalization not required
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromDirection(
        const Eigen::Matrix<T, 3, 1>& d) const {
        Eigen::Matrix<T, 3, 1> m = 2.0 * d / (A_.toDenseMatrix() * d).norm();

        return m;
    }

    /** \brief Get hypersphere surface point from spherical coordinate
     *  \param psi 2x1 vector of spherical coordinate
     * \return 3x1 vector of hypersphere */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getHypersphereFromSpherical(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 3, 1> u = {cos(psi[0]) * cos(psi[1]),
                                    cos(psi[0]) * sin(psi[1]), sin(psi[0])};

        return u;
    }

    /** \brief Get hypersphere surface point from surface gradient
     *  \param m 3x1 vector of surface gradient
     * \return 3x1 vector of hypersphere */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getHypersphereFromGradient(
        const Eigen::Matrix<T, 3, 1>& m) const {
        Eigen::Matrix<T, 3, 1> u = 0.5 * A_.toDenseMatrix() * m;

        return u;
    }

  private:
    /** Shape matrix of ellipsoid */
    Eigen::DiagonalMatrix<double, 3> A_;
};

}  // namespace cfc
