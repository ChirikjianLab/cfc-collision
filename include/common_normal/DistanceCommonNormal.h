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

namespace collision_common_normal {

/** \class DistanceCommonNormal
 * \brief Superclass for computing distance query between two convex objects
 * using common normal concept */
template <typename G1, typename G2>
class DistanceCommonNormal : public cfc::DistanceSolver<G1, G2> {
  public:
    /** \brief Constructor
     * \param s1_shape First object (cfc::Shape3D structure)
     * \param s2_shape Second object (cfc::Shape3D structure) */
    DistanceCommonNormal(const cfc::Shape3D& s1_shape,
                         const cfc::Shape3D& s2_shape);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object */
    DistanceCommonNormal(const G1* s1, const G2* s2);

    virtual ~DistanceCommonNormal();

    /** \brief Get optimization variables
     * \return Optimization variables as Eigen::Vector3d */
    Eigen::Vector3d getVariables() const { return this->normal_; }

    /** \brief Set optimization variables
     * \param normal Optimization variables (normal vector) as Eigen::Vector3d
     */
    void setVariables(const Eigen::Vector3d& normal) { this->normal_ = normal; }

    void query() override;

  protected:
    /** \brief Optimization using common normal concept */
    virtual bool normalOptimization() = 0;

  private:
    /** \brief Initialize the optimization */
    void initialization();
};

}  // namespace collision_common_normal

#include "DistanceCommonNormal-inl.h"
