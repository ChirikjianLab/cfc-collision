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

#include "geometry/ClosedFormContactSpace.h"
#include "util/MathUtils.h"

namespace cfc {

namespace collision_cfc {

/** \class ContinuousDistanceCFC
 * \brief Narrow phase continuous proximity queries between two 3D convex bodies
 * with smooth boundaries. Boundary parameterized by outward gradient/normal.
 * Optimization methods to be implemented by subclasses */
template <typename G1, typename G2>
class ContinuousDistanceCFC {
  public:
    /** \brief Constructor
     * \param s1 First object
     * \param g1_start Start pose of s1
     * \param g1_goal Goal pose of s1
     * \param s2 Second object
     * \param g2_start Start pose of s2
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceCFC(G1* s1, const Eigen::Affine3d& g1_start,
                          const Eigen::Affine3d& g1_goal, G1* s2,
                          const Eigen::Affine3d& g2_start,
                          const Eigen::Affine3d& g2_goal,
                          const ContinuousDistanceRequest& req);

    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure)
     * \param g1_start Start pose of s1
     * \param g1_goal Goal pose of s1
     * \param s2_shape Second object (Shape3D structure)
     * \param g2_start Start pose of s2
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceCFC(const Shape3D& s1_shape,
                          const Eigen::Affine3d& g1_start,
                          const Eigen::Affine3d& g1_goal,
                          const Shape3D& s2_shape,
                          const Eigen::Affine3d& g2_start,
                          const Eigen::Affine3d& g2_goal,
                          const ContinuousDistanceRequest& req);

    /** \brief Constructor
     * \param s1 First object, current pose as start
     * \param g1_goal Goal pose of s1
     * \param s2 Second object, current pose as start
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceCFC(G1* s1, const Eigen::Affine3d& g1_goal, G2* s2,
                          const Eigen::Affine3d& g2_goal,
                          const ContinuousDistanceRequest& req);

    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure), current pose as start
     * \param g1_goal Goal pose of s1
     * \param s2_shape Second object (Shape3D structure), current pose as start
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceCFC(const Shape3D& s1_shape,
                          const Eigen::Affine3d& g1_goal,
                          const Shape3D& s2_shape,
                          const Eigen::Affine3d& g2_goal,
                          const ContinuousDistanceRequest& req);

    virtual ~ContinuousDistanceCFC();

    /** \brief Set start pose of s1
     * \param g1_start Start pose */
    void setStartPoseS1(const Eigen::Affine3d& g1_start) {
        g1_start_ = g1_start;
    }

    /** \brief Set start pose of s2
     * \param g2_start Start pose */
    void setStartPoseS2(const Eigen::Affine3d& g2_start) {
        g2_start_ = g2_start;
    }

    /** \brief Set goal pose of s1
     * \param g1_goal Goal pose */
    void setGoalPoseS1(const Eigen::Affine3d& g1_goal) { g1_goal_ = g1_goal; }

    /** \brief Set goal pose of s2
     * \param g2_goal Goal pose */
    void setGoalPoseS2(const Eigen::Affine3d& g2_goal) { g2_goal_ = g2_goal; }

    /** \brief Set optimization variables
     * \param m Gradient variable
     * \param t Time variable */
    void setVariables(const Eigen::Vector3d& m, const double t) {
        var_.head(3) = m;
        var_(3) = t;

        setGradient(m);
        t_ = t;
    }

    /** \brief Set optimization variables
     * \param var Gradient and time variables stacked into 4D vector */
    void setVariables(const Eigen::Vector4d& var) {
        var_ = var;

        setGradient(var.head(3));
        t_ = var(3);
    }

    /** \brief Set gradient
     * \param gradient Gradient vector */
    void setGradient(const Eigen::Vector3d& gradient) {
        gradient_ = gradient;
        normal_ = gradient_.normalized();
    }

    /** \brief Get shape of s1
     * \return Shape3D structure for s1 object */
    Shape3D getS1Shape() const { return s1_->getShape(); }

    /** \brief Get shape of s2
     * \return Shape3D structure for s2 object */
    Shape3D getS2Shape() const { return s2_->getShape(); }

    /** \brief Get distance information
     * \return Results as ContinuousDistanceResult structure */
    ContinuousDistanceResult getDistanceInfo() const { return res_; }

    /** \brief Get optimization variables
     * \return 4D vector for gradient and time variable */
    Eigen::Vector4d getVariables() const { return var_; }

    /** \brief Get gradient vector
     * \return 3D vector for gradient vector */
    Eigen::Vector3d getGradient() const { return gradient_; }

    /** \brief Get normal vector
     * \return 3D vector of normal vector */
    Eigen::Vector3d getNormal() const { return normal_; }

    /** \brief Distance query, main routine */
    virtual void query();

  protected:
    /** \brief Setup (extract) parameters from shape info */
    void setup();

    /** \brief Set start poses for the two objects */
    void setStartPoses();

    /** \brief Setup optimization solver */
    virtual void setupSolver() = 0;

    /** \brief Initialization of optimizations */
    virtual void initialization();

    /** \brief Compute distance from point to closed-form contact space */
    virtual bool distancePointToCFC() = 0;

    /** \brief Compute the optimal solution for the closest point on contact
     * space */
    virtual void computeOptimalCFC() = 0;

    /** \brief Get optimization results */
    void results();

    /** \brief Necessary condition, normal/gradient at closest point on s1
     * should be colinear with the line connecting two closest points
     * \param normal Normal vector to evaluate the condition */
    void necessaryCondition(const Eigen::Vector3d& normal);

    /** \brief Object to compute CFC */
    ClosedFormContactSpace<double, G1, G2>* cfc_obj_;

    /** \brief The first object */
    G1* s1_;

    /** \brief The second object */
    G2* s2_;

    /** \brief Start pose of s1 */
    Eigen::Affine3d g1_start_;

    /** \brief Start pose of s2 */
    Eigen::Affine3d g2_start_;

    /** \brief Goal pose of s1 */
    Eigen::Affine3d g1_goal_;

    /** \brief Goal pose of s2 */
    Eigen::Affine3d g2_goal_;

    /** \brief Pose of s1 at optimal solution */
    Eigen::Affine3d g1_opt_ = Eigen::Affine3d::Identity();

    /** \brief Pose of s2 at optimal solution */
    Eigen::Affine3d g2_opt_ = Eigen::Affine3d::Identity();

    /** \brief Optimization variables */
    Eigen::Vector4d var_;

    /** \brief Time variable */
    double t_;

    /** \brief Variable for angular parameters */
    Eigen::Vector2d spherical_;

    /** \brief Gradient at a point on s1 */
    Eigen::Vector3d gradient_;

    /** \brief Normal at a point on s1 */
    Eigen::Vector3d normal_;

    /** \brief Collision request */
    ContinuousDistanceRequest req_;

    /** \brief Collision result */
    ContinuousDistanceResult res_;

    /** \brief Final number of iterations */
    int iter_ = 0;

    /** \brief The center of s2 as viewed in s1 local frame */
    Eigen::Vector3d s2_center_in_s1_;
};

}  // namespace collision_cfc

#include "ContinuousDistanceCFC-inl.h"

}  // namespace cfc
