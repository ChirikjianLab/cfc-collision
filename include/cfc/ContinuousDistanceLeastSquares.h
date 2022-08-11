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

#include "ContinuousDistanceCFC.h"

#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cfc {

namespace collision_cfc {

/** \class CostContinuousLeastSquares
 * \brief Computing closed-form contact space and cost function for continuous
 * distance query between two convex bodies with gradient-param surfaces */
template <typename G1, typename G2>
class CostContinuousLeastSquares {
  public:
    /** \brief Constructor
     * \param cfc_obj Object to compute CFC
     * \param g1_start Start pose of s1
     * \param g1_goal Goal pose of s1
     * \param g2_start Start pose of s2
     * \param g2_goal Goal pose of s2
     * \param motion_type The type of motion */
    CostContinuousLeastSquares(ClosedFormContactSpace<double, G1, G2>* cfc_obj,
                               const Eigen::Affine3d* g1_start,
                               const Eigen::Affine3d* g1_goal,
                               const Eigen::Affine3d* g2_start,
                               const Eigen::Affine3d* g2_goal,
                               const CCD_MOTION_TYPE& motion_type);

    ~CostContinuousLeastSquares();

    /** \brief Cost function */
    bool operator()(const double* psi, const double* t, double* cost) const;

  protected:
    /** \brief Object to compute CFC */
    ClosedFormContactSpace<double, G1, G2>* cfc_obj_;

    /** \brief Start pose of s1 */
    const Eigen::Affine3d* g1_start_;

    /** \brief Start pose of s2 */
    const Eigen::Affine3d* g2_start_;

    /** \brief Goal pose of s1 */
    const Eigen::Affine3d* g1_goal_;

    /** \brief Goal pose of s2 */
    const Eigen::Affine3d* g2_goal_;

    /** \brief Type of motions */
    const CCD_MOTION_TYPE& motion_type_;
};

/** \class ContinuousDistanceLeastSquares
 * \brief Narrow-phase continuous proximity queries between two 3D convex bodies
 * with smooth boundaries using gradient parameterization. Formulated as
 * least-squares optimization */
template <typename G1, typename G2>
class ContinuousDistanceLeastSquares : public ContinuousDistanceCFC<G1, G2> {
  public:
    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure)
     * \param g1_start Start pose of s1
     * \param g1_goal Goal pose of s1
     * \param s2_shape Second object (Shape3D structure)
     * \param g2_start Start pose of s2
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceLeastSquares(const Shape3D& s1_shape,
                                   const Eigen::Affine3d& g1_start,
                                   const Eigen::Affine3d& g1_goal,
                                   const Shape3D& s2_shape,
                                   const Eigen::Affine3d& g2_start,
                                   const Eigen::Affine3d& g2_goal,
                                   const ContinuousDistanceRequest& req);

    /** \brief Constructor
     * \param s1 First object
     * \param g1_start Start pose of s1
     * \param g1_goal Goal pose of s1
     * \param s2 Second object
     * \param g2_start Start pose of s2
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceLeastSquares(const G1* s1,
                                   const Eigen::Affine3d& g1_start,
                                   const Eigen::Affine3d& g1_goal, const G1* s2,
                                   const Eigen::Affine3d& g2_start,
                                   const Eigen::Affine3d& g2_goal,
                                   const ContinuousDistanceRequest& req);

    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure), current pose as start
     * \param g1_goal Goal pose of s1
     * \param s2_shape Second object (Shape3D structure), current pose as start
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceLeastSquares(const Shape3D& s1_shape,
                                   const Eigen::Affine3d& g1_goal,
                                   const Shape3D& s2_shape,
                                   const Eigen::Affine3d& g2_goal,
                                   const ContinuousDistanceRequest& req);

    /** \brief Constructor
     * \param s1 First object, current pose as start
     * \param g1_goal Goal pose of s1
     * \param s2 Second object, current pose as start
     * \param g2_goal Goal pose of s2
     * \param req ContinuousDistanceRequest object */
    ContinuousDistanceLeastSquares(const G1* s1, const Eigen::Affine3d& g1_goal,
                                   const G1* s2, const Eigen::Affine3d& g2_goal,
                                   const ContinuousDistanceRequest& req);

    virtual ~ContinuousDistanceLeastSquares();

  protected:
    bool distancePointToCFC() override;

    void computeOptimalCFC() override;

  private:
    void setupSolver() override;

    ceres::Problem problem_;
    ceres::CostFunction* cost_;
    ceres::Solver::Options options_;
    ceres::Solver::Summary summary_;
};

}  // namespace collision_cfc

#include "ContinuousDistanceLeastSquares-inl.h"

}  // namespace cfc
