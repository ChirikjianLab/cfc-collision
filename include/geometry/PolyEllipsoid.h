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

/** \class PolyEllipsoid
 * \brief Definitions of shape and operations for poly-ellipsoidal surface */
class PolyEllipsoid {
  public:
    /** \brief Constructor
     * \param shape Shape of the poly-ellipsoid model */
    PolyEllipsoid(const Shape3D& shape) : shape_(shape) {}

    ~PolyEllipsoid() {}

  public:
    /** \brief Set semi-axes
     * \param a00 Semi-axis along x-positive direction
     * \param a01 Semi-axis along y-positive direction
     * \param a10 Semi-axis along z-positive direction
     * \param a11 Semi-axis along x-negative direction
     * \param a20 Semi-axis along y-negative direction
     * \param a21 Semi-axis along z-negative direction */
    void setSemiAxes(const double a00, const double a01, const double a10,
                     const double a11, const double a20, const double a21) {
        shape_.a[0] = a00;
        shape_.a[1] = a10;
        shape_.a[2] = a20;
        shape_.b[0] = a01;
        shape_.b[1] = a11;
        shape_.b[2] = a21;
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
     * \return List of semi-axis lengths along positive direction */
    const double* a() const { return shape_.a; }

    /** \brief Get semi-axes
     * \return List of semi-axis lengths along negative direction */
    const double* b() const { return shape_.b; }

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

    /** \brief Compute implicit function value
     *  \param x 3x1 vector for a point in Euclidean space
     * \return scalar value for implicit function */
    template <typename T>
    T getImplicitFunction(const Eigen::Matrix<T, 3, 1>& x) const {
        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(x);
        T implicit = x.transpose() * (A * A).inverse() * x - 1;

        return implicit;
    }

    /** \brief Get boundary point from angles
     *  \param psi 2x1 vector for angular parameter
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromSpherical(
        const Eigen::Matrix<T, 2, 1>& psi) const {
        Eigen::Matrix<T, 3, 1> u = getHypersphereFromSpherical(psi);

        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(u);
        Eigen::Matrix<T, 3, 1> x = A * u;

        return x;
    }

    /** \brief Get boundary point from gradient vector
     *  \param m 3x1 vector of surface gradient
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromGradient(
        const Eigen::Matrix<T, 3, 1>& m) const {
        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(m);
        Eigen::Matrix<T, 3, 1> x = 0.5 * (A * A) * m;

        return x;
    }

    /** \brief Get boundary point from normal vector
     *  \param n 3x1 vector of surface normal
     * \return 3x1 vector of surface point */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getBoundaryFromNormal(
        const Eigen::Matrix<T, 3, 1>& n) const {
        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(n);
        Eigen::Matrix<T, 3, 1> x = (A * A) * n / (A * n).norm();

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
        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(x);
        Eigen::Matrix<T, 3, 1> m = 2.0 * (A * A).inverse() * x;

        return m;
    }

    /** \brief Get surface gradient from hypersphere
     *  \param u 3x1 vector of hypersphere
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromHypersphere(
        const Eigen::Matrix<T, 3, 1>& u) const {
        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(u);
        Eigen::Matrix<T, 3, 1> m = 2.0 * A.inverse() * u;

        return m;
    }

    /** \brief Get surface gradient from directional vector
     *  \param d 3x1 vector indicating direction, normalization not required
     * \return 3x1 vector of surface gradient */
    template <typename T>
    Eigen::Matrix<T, 3, 1> getGradientFromDirection(
        const Eigen::Matrix<T, 3, 1>& d) const {
        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(d);
        Eigen::Matrix<T, 3, 1> m = 2.0 * d / (A * d).norm();

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
        Eigen::Matrix<T, 3, 3> A = getShapeParamOctant(m);
        Eigen::Matrix<T, 3, 1> u = 0.5 * A * m;

        return u;
    }

    /** \brief Get the shape matrix according to octant the point locates
     *  \param point 3x1 vector of point
     * \return 3x3 diagonal matrix indicating ellipsoid shape at certain
     * octant */
    template <typename T>
    Eigen::Matrix<T, 3, 3> getShapeParamOctant(
        const Eigen::Matrix<T, 3, 1> point) const {
        Eigen::Matrix<T, 3, 3> A;
        A.setIdentity();

        A(0, 0) = point[0] > 0.0 ? T(shape_.a[0]) : T(shape_.b[0]);
        A(1, 1) = point[1] > 0.0 ? T(shape_.a[1]) : T(shape_.b[1]);
        A(2, 2) = point[2] > 0.0 ? T(shape_.a[2]) : T(shape_.b[2]);

        return A;
    }

  protected:
    /** \brief Shape of the superquadric model */
    Shape3D shape_;
};

}  // namespace cfc
