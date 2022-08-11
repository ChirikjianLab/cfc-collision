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

#include "DistanceCFC.h"

namespace cfc {

namespace collision_cfc {

/** \class DistanceFixedPoint
 * \brief Narrow phase proximity queries between two convex bodies. Bodies are
 * parameterized by outward gradient (un-normalized). Collision detection is
 * solved by fixed-point iteration method */
template <typename G1, typename G2>
class DistanceFixedPoint : public DistanceCFC<G1, G2> {
  public:
    /** \brief Constructor
     * \param s1_shape First object (Shape3D structure)
     * \param s2_shape Second object (Shape3D structure) */
    DistanceFixedPoint(const Shape3D& s1_shape, const Shape3D& s2_shape);

    /** \brief Constructor
     * \param s1 First object
     * \param s2 Second object */
    DistanceFixedPoint(const G1* s1, const G2* s2);

    ~DistanceFixedPoint() override;

  protected:
    bool distancePointToCFC() override;

  private:
    virtual void setupSolver() override;

    /** \brief Main algorithm using fixed-point iteration to update gradient
     * vector
     * \brief m_init Initial gradient
     * \return Solution of the optimal gradient */
    Eigen::Vector3d solve(const Eigen::Vector3d& m_init);

    /** \brief Step for fixed-point iteration
     * \param m_01 Gradient at (i-2) step
     * \param m_02 Graident at (i-1) step */
    Eigen::Vector3d fixedPointStep(const Eigen::Vector3d& m_01,
                                   const Eigen::Vector3d& m_02);

  private:
    /** \brief Max number of iterations */
    int max_iter_ = 100;

    /** \brief Tolerence for stopping the iteration */
    double tol_ = 1e-12;

    /** \brief Collision detection flag */
    bool flag_ = false;
};

}  // namespace collision_cfc

#include "DistanceFixedPoint-inl.h"

}  // namespace cfc
