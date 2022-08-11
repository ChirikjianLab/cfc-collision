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

#include <limits>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

const double pi = 3.1415926;
const double NaN = std::numeric_limits<double>::quiet_NaN();
const double Inf = std::numeric_limits<double>::infinity();
const double EPSILON = 1e-8;

namespace cfc {

enum GEOM_TYPE { ELLIPSOID, SUPERQUADRICS, POLYELLIPSOID };

/** \brief Shape info for geometries (3D) */
struct Shape3D {
    /** \brief Semi-axes lengths (for poly-ellipsoid, it is the semi-axes along
     * positive axis direction) */
    double a[3] = {NaN, NaN, NaN};

    /** \brief Quaternion for orientation, order: [w, x, y, z] */
    double q[4] = {1.0, 0.0, 0.0, 0.0};

    /** \brief Power of exponents */
    double eps[2] = {1.0, 1.0};

    /** \brief Position [x, y, z] */
    double pos[3] = {0.0, 0.0, 0.0};

    /** \brief (for poly-ellipsoid only) Semi-axes lengths along negative axis
     * direction */
    double b[3] = {NaN, NaN, NaN};
};

/** \brief Type of motion for continuous collision detection */
enum CCD_MOTION_TYPE { PCG_LINEAR, SE_LINEAR, TRAN, ROT };

/** \brief Request for CCD */
struct ContinuousDistanceRequest {
    /** \brief Motion type */
    CCD_MOTION_TYPE motion_type = TRAN;

    /** \brief Max number of iterations */
    int max_num_iteration = 100;

    /** \brief Tolerance for the cost function */
    double cost_tolerance = 1e-8;

    /** \brief Tolerance for the jacobian */
    double jac_tolerance = 1e-10;
};

/** \brief Distance info */
struct DistanceResult {
    /** \brief Status for collision */
    bool is_collision = true;

    /** \brief Closest (Witness) point on s1 */
    Eigen::Vector3d closest_point_s1 = {NaN, NaN, NaN};

    /** \brief Closest (Witness) point on s2 */
    Eigen::Vector3d closest_point_s2 = {NaN, NaN, NaN};

    /** \brief Point on the contact space */
    Eigen::Vector3d point_on_contact_space = {NaN, NaN, NaN};

    /** \brief Normal at the optimal point on s1 */
    Eigen::Vector3d optimal_normal = {NaN, NaN, NaN};

    /** \brief Resulting distance/penetration */
    double distance = Inf;

    /** \brief Necessary condition for optimality */
    double necessary_condition = Inf;

    /** \brief Number of iterations after solving the optimization */
    int num_iteration = 0;
};

/** \brief Result for CCD */
struct ContinuousDistanceResult {
    /** \brief Status for collision */
    bool is_collision = true;

    /** \brief Closest (Witness) point on s1 */
    Eigen::Vector3d closest_point_s1 = {NaN, NaN, NaN};

    /** \brief Closest (Witness) point on s2 */
    Eigen::Vector3d closest_point_s2 = {NaN, NaN, NaN};

    /** \brief Point on the contact space */
    Eigen::Vector3d point_on_contact_space = {NaN, NaN, NaN};

    /** \brief Normal at the optimal point on s1 */
    Eigen::Vector3d optimal_normal = {NaN, NaN, NaN};

    /** \brief Resulting distance/penetration */
    double distance = Inf;

    /** \brief Necessary condition for optimality */
    double necessary_condition = Inf;

    /** \brief Time of travel at the optimal solution */
    double optimal_time = 0.0;

    /** \brief Number of iterations after solving the optimization */
    int num_iteration = 0;
};

/** \brief vertices on the parametric surface */
struct ParametricPoints {
    /** \brief x-coordinate */
    std::vector<double> x;

    /** \brief y-coordinate */
    std::vector<double> y;

    /** \brief z-coordinate */
    std::vector<double> z;
};
}  // namespace cfc
