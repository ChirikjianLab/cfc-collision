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

#include "GeometryInfo.h"
#include "util/MathUtils.h"

namespace cfc {

/** \class SuperQuadrics
 * \brief Definition of the shape and operations for superquadric surface */
class SuperQuadrics {
  public:
    /** \brief Constructor
     * \param shape Shape of superquadric model */
    SuperQuadrics(const Shape3D& shape) : shape_(shape) {}

    ~SuperQuadrics() {}

  public:
    /** \brief Set the semi-axes
     * \param a0 Semi-axis along x-axis
     * \param a1 Semi-axis along y-axis
     * \param a2 Semi-axis along z-axis */
    void setSemiAxes(const double a0, const double a1, const double a2) {
        shape_.a[0] = a0;
        shape_.a[1] = a1;
        shape_.a[2] = a2;
    }

    /** \brief Set power of exponents
     * \param eps0 First exponent
     * \param eps1 Second exponent */
    void setExponents(const double eps0, const double eps1) {
        shape_.eps[0] = eps0;
        shape_.eps[1] = eps1;
    }

    /** \brief Set the position
     * \brief pos0 x-coordinate
     * \brief pos1 y-coordinate
     * \brief pos2 z-coordinate */
    void setPosition(const double pos0, const double pos1, const double pos2) {
        shape_.pos[0] = pos0;
        shape_.pos[1] = pos1;
        shape_.pos[2] = pos2;
    }

    /** \brief Set orientation from quaternion
     * \param q0 w-element
     * \param q1 x-element
     * \param q2 y-element
     * \param q3 z-element */
    void setOrientation(const double q0, const double q1, const double q2,
                        const double q3) {
        shape_.q[0] = q0;
        shape_.q[1] = q1;
        shape_.q[2] = q2;
        shape_.q[3] = q3;
    }

    /** \brief Set orientation from Eigen::Quaterniond
     * \param quat Quaternion for rotation as Eigen::Quaterniond */
    void setOrientation(const Eigen::Quaterniond quat) {
        shape_.q[0] = quat.w();
        shape_.q[1] = quat.x();
        shape_.q[2] = quat.y();
        shape_.q[3] = quat.z();
    }

    /** \brief Get the shape
     * \return Shape3D structure for the shape */
    const Shape3D getShape() const { return shape_; }

    /** \brief Get semi-axes
     * \return List of semi-axis lengths */
    const double* a() const { return shape_.a; }

    /** \brief Get power of exponents
     * \return List of exponents */
    const double* eps() const { return shape_.eps; }

    /** \brief Get quaternion for orientation
     * \return List of quaternion, order: [w, x, y, z] */
    const double* q() const { return shape_.q; }

    /** \brief Get Eigen::Quaterniond
     * \return Quaternion for orientation as Eigen::Quaterniond */
    const Eigen::Quaterniond quaternion() const {
        return Eigen::Quaterniond(shape_.q[0], shape_.q[1], shape_.q[2],
                                  shape_.q[3]);
    }

    /** \brief Get position
     * \return List of position [x, y, z] */
    const double* pos() const { return shape_.pos; }

    /** \brief Get parameters for the uniform surface samples
     * \param psi Angular parameter
     * \return An array of surface points as Eigen::Matrix */
    template <typename T>
    Eigen::Matrix<T, 2, 1> getUniformParam(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 2, 1> psiNew;
        psiNew[0] = std::atan(eps_func(std::tan(psi[0]), 1.0 / shape_.eps[0]));
        psiNew[1] = std::atan(eps_func(std::tan(psi[1]), 1.0 / shape_.eps[1]));

        psiNew[1] = psiNew[1] > pi / 2.0 ? psiNew[1] + pi : psiNew[1];
        psiNew[1] = psiNew[1] < -pi / 2.0 ? psiNew[1] - pi : psiNew[1];

        return psiNew;
    }

    /** \brief Compute implicit function value
     *  \param x 3x1 vector for a point in Euclidean space
     * \return Scalar value for implicit function */
    template <typename T>
    T getImplicitFunction(const Eigen::Matrix<T, 3, 1>& x) const {
        T implicit;
        implicit =
            eps_func(
                eps_func(pow(x[0] / shape_.a[0], 2.0), 1.0 / shape_.eps[1]) +
                    eps_func(pow(x[1] / shape_.a[1], 2.0), 1.0 / shape_.eps[1]),
                shape_.eps[1] / shape_.eps[0]) +
            eps_func(pow(x[2] / shape_.a[2], 2.0), 1.0 / shape_.eps[0]) - 1.0;

        return implicit;
    }

    /** \brief Get boundary point from angles
     *  \param psi 2x1 vector for angular parameter
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromSpherical(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 3, 1> x = {
            shape_.a[0] * eps_func(cos(psi[0]), shape_.eps[0]) *
                eps_func(cos(psi[1]), shape_.eps[1]),
            shape_.a[1] * eps_func(cos(psi[0]), shape_.eps[0]) *
                eps_func(sin(psi[1]), shape_.eps[1]),
            shape_.a[2] * eps_func(sin(psi[0]), shape_.eps[0])};

        return x;
    }

    /** \brief Get boundary point from gradient vector
     *  \param m 3x1 vector of surface gradient
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromGradient(
        const Eigen::Matrix<T, 3, 1>& m) const {
        Eigen::Matrix<T, 3, 1> x = {
            shape_.a[0] *
                eps_func(shape_.a[0] * shape_.eps[0] / 2.0 * m[0],
                         shape_.eps[1] / (2.0 - shape_.eps[1])) *
                eps_func(
                    1.0 - eps_func(pow(shape_.a[2] * shape_.eps[0] / 2.0 * m[2],
                                       2.0),
                                   1.0 / (2.0 - shape_.eps[0])),
                    (shape_.eps[0] - shape_.eps[1]) / (2.0 - shape_.eps[1])),
            shape_.a[1] *
                eps_func(shape_.a[1] * shape_.eps[0] / 2.0 * m[1],
                         shape_.eps[1] / (2.0 - shape_.eps[1])) *
                eps_func(
                    1.0 - eps_func(pow(shape_.a[2] * shape_.eps[0] / 2.0 * m[2],
                                       2.0),
                                   1.0 / (2.0 - shape_.eps[0])),
                    (shape_.eps[0] - shape_.eps[1]) / (2.0 - shape_.eps[1])),
            shape_.a[2] * eps_func(shape_.a[2] * shape_.eps[0] / 2.0 * m[2],
                                   shape_.eps[0] / (2.0 - shape_.eps[0]))};

        return x;
    }

    /** \brief Get boundary point from normal vector
     *  \param n 3x1 vector of surface normal
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromNormal(
        const Eigen::Matrix<T, 3, 1>& n) const {
        Eigen::Matrix<T, 3, 1> m = getGradientFromDirection(n);
        return getBoundaryFromGradient(m);
    }

    /** \brief Get surface gradient from spherical coordinates
     *  \param psi 2x1 vector for angular parameter
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromSpherical(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 3, 1> m = {
            2.0 / (shape_.a[0] * shape_.eps[0]) *
                eps_func(cos(psi[0]), 2.0 - shape_.eps[0]) *
                eps_func(cos(psi[1]), 2.0 - shape_.eps[1]),
            2.0 / (shape_.a[1] * shape_.eps[0]) *
                eps_func(cos(psi[0]), 2.0 - shape_.eps[0]) *
                eps_func(sin(psi[1]), 2.0 - shape_.eps[1]),
            2.0 / (shape_.a[2] * shape_.eps[0]) *
                eps_func(sin(psi[0]), 2.0 - shape_.eps[0])};

        return m;
    }

    /** \brief Get surface gradient from surface point
     *  \param x 3x1 vector of surface point
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromCartesian(
        const Eigen::Matrix<T, 3, 1>& x) const {
        Eigen::Matrix<T, 3, 1> m;
        T g_x1x2 =
            eps_func(std::pow(x[0] / shape_.a[0], 2.0), 1.0 / shape_.eps[1]) +
            eps_func(std::pow(x[1] / shape_.a[1], 2.0), 1.0 / shape_.eps[1]);

        m(0, 0) = 2.0 / (shape_.a[0] * shape_.eps[0]) *
                  eps_func(g_x1x2, shape_.eps[1] / shape_.eps[0] - 1.0) *
                  eps_func(x[0] / shape_.a[0], 2.0 / shape_.eps[1] - 1.0);
        m(1, 0) = 2.0 / (shape_.a[1] * shape_.eps[0]) *
                  eps_func(g_x1x2, shape_.eps[1] / shape_.eps[0] - 1.0) *
                  eps_func(x[1] / shape_.a[1], 2.0 / shape_.eps[1] - 1.0);
        m(2, 0) = 2.0 / (shape_.a[2] * shape_.eps[0]) *
                  eps_func(x[2] / shape_.a[2], 2.0 / shape_.eps[0] - 1.0);

        return m;
    }

    /** \brief Get surface gradient from hypersphere
     *  \param u 3x1 vector of hypersphere
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromHypersphere(
        const Eigen::Matrix<T, 3, 1>& u) const {
        Eigen::Matrix<T, 3, 1> m = {
            2.0 / (shape_.a[0] * shape_.eps[0]) *
                eps_func(u[0], 2.0 - shape_.eps[1]) *
                eps_func(pow(u[0], 2.0) + pow(u[1], 2.0),
                         (shape_.eps[1] - shape_.eps[0]) / 2.0),
            2.0 / (shape_.a[1] * shape_.eps[0]) *
                eps_func(u[1], 2.0 - shape_.eps[1]) *
                eps_func(pow(u[0], 2.0) + pow(u[1], 2.0),
                         (shape_.eps[1] - shape_.eps[0]) / 2.0),
            2.0 / (shape_.a[2] * shape_.eps[0]) *
                eps_func(u[2], 2.0 - shape_.eps[0])};

        return m;
    }

    /** \brief Get surface gradient from directional vector
     *  \param d 3x1 vector indicating direction, normalization not required
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromDirection(
        const Eigen::Matrix<T, 3, 1>& d) const {
        // Implicit function value of dual superquadric
        Shape3D dualShape = shape_;
        dualShape.a[0] = 2.0 / (shape_.a[0] * shape_.eps[0]);
        dualShape.a[1] = 2.0 / (shape_.a[1] * shape_.eps[0]);
        dualShape.a[2] = 2.0 / (shape_.a[2] * shape_.eps[0]);
        dualShape.eps[0] = 2.0 - shape_.eps[0];
        dualShape.eps[1] = 2.0 - shape_.eps[1];

        T dualImplicit =
            eps_func(eps_func(pow(d[0] / dualShape.a[0], 2.0),
                              1.0 / dualShape.eps[1]) +
                         eps_func(pow(d[1] / dualShape.a[1], 2.0),
                                  1.0 / dualShape.eps[1]),
                     dualShape.eps[1] / dualShape.eps[0]) +
            eps_func(pow(d[2] / dualShape.a[2], 2.0), 1.0 / dualShape.eps[0]);

        // Gradient vector on surface point
        Eigen::Matrix<T, 3, 1> m =
            d / eps_func(dualImplicit, dualShape.eps[0] / 2.0);

        return m;
    }

    /** \brief Get hypersphere surface point from surface gradient
     *  \param m 3x1 vector of surface gradient
     * \return 3x1 vector of hypersphere */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getHypersphereFromGradient(
        const Eigen::Matrix<T, 3, 1>& m) const {
        Eigen::Matrix<T, 3, 1> u = {
            eps_func((shape_.a[0] * shape_.eps[0]) / 2.0 * m[0],
                     1.0 / (2.0 - shape_.eps[1])) *
                eps_func(
                    eps_func(pow(shape_.a[0] * shape_.eps[0] / 2.0 * m[0], 2.0),
                             1.0 / (2.0 - shape_.eps[1])) +
                        eps_func(
                            pow(shape_.a[1] * shape_.eps[0] / 2.0 * m[1], 2.0),
                            1.0 / (2.0 - shape_.eps[1])),
                    (shape_.eps[0] - shape_.eps[1]) /
                        (4.0 - 2.0 * shape_.eps[0])),
            eps_func((shape_.a[1] * shape_.eps[0]) / 2.0 * m[1],
                     1.0 / (2.0 - shape_.eps[1])) *
                eps_func(
                    eps_func(pow(shape_.a[0] * shape_.eps[0] / 2.0 * m[0], 2.0),
                             1.0 / (2.0 - shape_.eps[1])) +
                        eps_func(
                            pow(shape_.a[1] * shape_.eps[0] / 2.0 * m[1], 2.0),
                            1.0 / (2.0 - shape_.eps[1])),
                    (shape_.eps[0] - shape_.eps[1]) /
                        (4.0 - 2.0 * shape_.eps[0])),
            eps_func((shape_.a[2] * shape_.eps[0]) / 2.0 * m[2],
                     1.0 / (2.0 - shape_.eps[0]))};

        return u;
    }

    /** \brief Get jacobian of gradient with respect to surface point
     *  \param x 3x1 vector of surface point
     * \return 3x3 jacobian matrix */
    template <typename T>
    Eigen::Matrix<T, 3, 3> getJacobianGradientFromCartesian(
        const Eigen::Matrix<T, 3, 1>& x) const {
        Eigen::Matrix<T, 3, 3> JacGradCart;
        JacGradCart.setZero();

        T g_x1x2 =
            eps_func(std::pow(x[0] / shape_.a[0], 2.0), 1.0 / shape_.eps[1]) +
            eps_func(std::pow(x[1] / shape_.a[1], 2.0), 1.0 / shape_.eps[1]);

        // Non-zero entries of the Jacobian matrix
        JacGradCart(0, 0) =
            2.0 / (std::pow(shape_.a[0], 2.0) * shape_.eps[0]) *
            ((2.0 / shape_.eps[1]) * (shape_.eps[1] / shape_.eps[0] - 1.0) *
                 eps_func(g_x1x2, shape_.eps[1] / shape_.eps[0] - 2.0) *
                 eps_func(std::pow(x[0] / shape_.a[0], 2.0),
                          2.0 / shape_.eps[1] - 1.0) +
             (2.0 / shape_.eps[1] - 1.0) *
                 eps_func(g_x1x2, shape_.eps[1] / shape_.eps[0] - 1.0) *
                 eps_func(std::pow(x[0] / shape_.a[0], 2.0),
                          1.0 / shape_.eps[1] - 1.0));

        JacGradCart(0, 1) =
            (2.0 / (shape_.a[0] * shape_.a[1] * shape_.eps[0]) *
             (2.0 / shape_.eps[1]) * (shape_.eps[1] / shape_.eps[0] - 1.0)) *
            eps_func(g_x1x2, shape_.eps[1] / shape_.eps[0] - 2.0) *
            eps_func((x[0] / shape_.a[0]), 2.0 / shape_.eps[1] - 1.0) *
            eps_func((x[1] / shape_.a[1]), 2.0 / shape_.eps[1] - 1.0);

        JacGradCart(1, 0) = JacGradCart(0, 1);

        JacGradCart(1, 1) =
            2.0 / (std::pow(shape_.a[1], 2.0) * shape_.eps[0]) *
            ((2.0 / shape_.eps[1]) * (shape_.eps[1] / shape_.eps[0] - 1.0) *
                 eps_func(g_x1x2, shape_.eps[1] / shape_.eps[0] - 2.0) *
                 eps_func(std::pow(x[1] / shape_.a[1], 2.0),
                          2.0 / shape_.eps[1] - 1.0) +
             (2.0 / shape_.eps[1] - 1.0) *
                 eps_func(g_x1x2, shape_.eps[1] / shape_.eps[0] - 1.0) *
                 eps_func(std::pow(x[1] / shape_.a[1], 2.0),
                          1.0 / shape_.eps[1] - 1.0));

        JacGradCart(2, 2) =
            (2.0 / (std::pow(shape_.a[2], 2.0) * shape_.eps[0]) *
             (2.0 / shape_.eps[0] - 1.0)) *
            eps_func(std::pow(x[2] / shape_.a[2], 2.0),
                     1.0 / shape_.eps[0] - 1.0);

        return JacGradCart;
    }

    /** \brief Get jacobian of gradient with respect to spherical coordinates
     *  \param psi 2x1 vector for angular parameter
     * \return 3x2 jacobian matrix */
    template <typename T>
    Eigen::Matrix<T, 3, 2> getJacobianCartesianFromSpherical(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 3, 2> JacCartSph;
        JacCartSph.setZero();

        JacCartSph(0, 0) =
            (shape_.a[0] * shape_.eps[0]) *
            eps_func(std::abs(cos(psi[0])), shape_.eps[0] - 1.0) *
            eps_func(cos(psi[1]), shape_.eps[1]) * (-sin(psi[0]));

        JacCartSph(0, 1) =
            (shape_.a[0] * shape_.eps[1]) *
            eps_func(cos(psi[0]), shape_.eps[0]) *
            eps_func(std::abs(cos(psi[1])), shape_.eps[1] - 1.0) *
            (-sin(psi[1]));

        JacCartSph(1, 0) =
            (shape_.a[1] * shape_.eps[0]) *
            eps_func(std::abs(cos(psi[0])), shape_.eps[0] - 1.0) *
            eps_func(sin(psi[1]), shape_.eps[1]) * (-sin(psi[0]));

        JacCartSph(1, 1) =
            (shape_.a[0] * shape_.eps[1]) *
            eps_func(cos(psi[0]), shape_.eps[0]) *
            eps_func(std::abs(sin(psi[1])), shape_.eps[1] - 1.0) * cos(psi[1]);

        JacCartSph(2, 0) =
            (shape_.a[2] * shape_.eps[0]) *
            eps_func(std::abs(sin(psi[0])), shape_.eps[0] - 1.0) * cos(psi[0]);

        return JacCartSph;
    }

  protected:
    /** \brief Shape of the superquadric model */
    Shape3D shape_;
};

}  // namespace cfc
