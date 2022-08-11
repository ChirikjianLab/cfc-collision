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

/** \class DistanceSolver
 * \brief Superclass for distance solvers in static case */
template <typename G1, typename G2>
class DistanceSolver {
  public:
    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure)
     * \param s2_shape Second object (Shape3D structure) */
    DistanceSolver(const Shape3D& s1_shape, const Shape3D& s2_shape);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object */
    DistanceSolver(const G1* s1, const G2* s2);

    virtual ~DistanceSolver();

    /** \brief Set linear transformation matrix for s1
     * \param M1 3x3 linear transformation matrix */
    void setLinearTransformS1(const Eigen::Matrix3d& M1) { M1_ = M1; }

    /** \brief Set linear transformation matrix for s2
     * \param M2 3x3 linear transformation matrix */
    void setLinearTransformS2(const Eigen::Matrix3d& M2) { M2_ = M2; }

    /** \brief Set gradient vector
     * \param gradient Gradient vector */
    void setGradient(const Eigen::Vector3d& gradient) { gradient_ = gradient; }

    /** \brief Get shape of s1
     * \return Shape3D structure for s1 object */
    Shape3D getS1Shape() const { return s1_->getShape(); }

    /** \brief Get shape of s2
     * \return Shape3D structure for s2 object */
    Shape3D getS2Shape() const { return s2_->getShape(); }

    /** \brief Get distance information
     * \return Results as DistanceResult structure */
    DistanceResult getDistanceInfo() const { return res_; }

    /** \brief Get gradient vector
     * \return 3D vector for gradient vector */
    Eigen::Vector3d getGradient() const { return gradient_; }

    /** \brief Get normal vector
     * \return 3D vector of normal vector */
    Eigen::Vector3d getNormal() const { return normal_; }

    /** \brief Get center of s2 viewed in s1
     * \return 3D vector of s2 center viewed in local frame of s1 */
    Eigen::Vector3d getS2CenterInS1() const { return s2_center_in_s1_; }

    /** \brief Distance query, main routine */
    virtual void query() = 0;

  protected:
    /** \brief Setup the distance solver */
    virtual void setup();

    /** \brief First object */
    const G1* s1_;

    /** \brief Second object */
    const G2* s2_;

    /** \brief Linear transformation of s1 */
    Eigen::Matrix3d M1_;

    /** \brief Linear transformation of s2 */
    Eigen::Matrix3d M2_;

    /** \brief Gradient at a point on s1 surface */
    Eigen::Vector3d gradient_;

    /** \brief Normal at a point on s1 surface */
    Eigen::Vector3d normal_;

    /** \brief Center of s1 */
    Eigen::Vector3d s1_center_;

    /** \brief Center of s2 */
    Eigen::Vector3d s2_center_;

    /** \brief Center of s2 as viewed in s1 local frame */
    Eigen::Vector3d s2_center_in_s1_;

    /** \brief Vector pointing from s1 center to s2 center */
    Eigen::Vector3d s1_to_s2_;

    /** \brief Number of iterations for the optimization */
    int iter_ = 0;

    /** \brief Results of the solved distance information */
    DistanceResult res_;
};

#include "DistanceSolver-inl.h"

}  // namespace cfc
