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

namespace collision_common_normal {

/** \class DistanceCommonNormalFixedPoint
 * \brief Computing distance query between two convex objects using common
 * normal concept and fixed-point iteration.
 *
 * Reference: Römer, U.J., Fidlin, A. and Seemann, W., 2020.
 * The normal parameterization and its application to collision detection.
 * Mechanism and Machine Theory, 151, p.103906. */
template <typename G1, typename G2>
class DistanceCommonNormalFixedPoint : public DistanceCommonNormal<G1, G2> {
  public:
    /** \brief Constructor
     * \param s1_shape First object (cfc::Shape3D structure)
     * \param s2_shape Second object (cfc::Shape3D structure) */
    DistanceCommonNormalFixedPoint(const cfc::Shape3D& s1_shape,
                                   const cfc::Shape3D& s2_shape);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object */
    DistanceCommonNormalFixedPoint(const G1* s1, const G2* s2);

    virtual ~DistanceCommonNormalFixedPoint();

  protected:
    /** \brief Optimization using common normal concept */
    virtual bool normalOptimization();

  private:
    /** \brief Main algorithm using fixed-point iteration to update normal
     * vectors
     * \param n_init Initial normal
     * \param max_iter Maximum number of iterations
     * \param tol Tolerance for stopping
     * \param iter Resulting number of iterations
     * \param flag Result flag */
    Eigen::Vector3d solve(const Eigen::Vector3d& n_init, const int max_iter,
                          const double tol, int* iter, bool* flag);

    /** \brief Step for fixed-point iteration
     * \param n_01 Normal at (i-2) step
     * \param n_02 Normal at (i-1) step */
    Eigen::Vector3d fixedPointStep(const Eigen::Vector3d& n_01,
                                   const Eigen::Vector3d& n_02);
};

}  // namespace collision_common_normal

#include "DistanceCommonNormalFixedPoint-inl.h"
