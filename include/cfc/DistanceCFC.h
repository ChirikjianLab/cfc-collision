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

#include "DistanceSolver.h"
#include "geometry/ClosedFormContactSpace.h"

namespace cfc {

namespace collision_cfc {

/** \class DistanceCFC
 * \brief Narrow phase proximity queries between two 3D convex bodies with
 * smooth boundaries. Boundary parameterized by outward gradient/normal.
 * Optimization methods to be implemented by subclasses */
template <typename G1, typename G2>
class DistanceCFC : public DistanceSolver<G1, G2> {
  public:
    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure)
     * \param s2_shape Second object (Shape3D structure) */
    DistanceCFC(const Shape3D& s1_shape, const Shape3D& s2_shape);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object */
    DistanceCFC(const G1* s1, const G2* s2);

    virtual ~DistanceCFC() override;

    /** \brief Set variables
     * \param psi Angular variable */
    void setVariables(const double* psi) {
        psi_[0] = psi[0];
        psi_[1] = psi[1];
    }

    /** \brief Get optimization variables
     * \return List of variables */
    const double* getVariables() const { return psi_; }

    virtual void query() override;

  protected:
    /** \brief Setup object for computing CFC */
    void setCFC();

    /** \brief Setup optimization solver */
    virtual void setupSolver() = 0;

    /** \brief Initialization of optimizations */
    virtual void initialization();

    /** \brief Compute distance between point and contact space */
    virtual bool distancePointToCFC() = 0;

    /** \brief Compute point on closed-form contact space after finding the
     * optimal solution */
    void computeOptimalCFC();

    /** \brief Compute results */
    void results();

    /** \brief Necessary condition, normal/gradient at closest point on s1
     * should be colinear with the line connecting two closest points
     * \param normal Normal vector as Eigen::Vector3d */
    void necessaryCondition(const Eigen::Vector3d& normal);

    /** \brief Object to compute CFC */
    ClosedFormContactSpace<double, G1, G2>* cfc_obj_;

    /** \brief Optimization variables */
    double psi_[2];
};

}  // namespace collision_cfc

#include "DistanceCFC-inl.h"

}  // namespace cfc
