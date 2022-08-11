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

#include "cfc/DistanceLeastSquares.h"
#include "fcl_interface/DistanceFCL.h"
#include "implicit/DistanceImplicit.h"
#include "util/Utils.h"

#include <gtest/gtest.h>
#include <fstream>

static const unsigned int SURFACE_MESH_PARAM = 20;

// Pair of Ellipsoid-SuperQuadrics
std::pair<cfc::Shape3D, cfc::Shape3D> initFixedES() {
    // Unit quaternion
    Eigen::Vector4d q1;
    q1 = {0.25, -0.16, -0.52, -1.6};
    q1.normalize();

    Eigen::Vector4d q2;
    q2 = {-0.5, 0.25, 0.166, -0.13};
    q2.normalize();

    // Initialize shapes
    cfc::Shape3D sq1 = {{18.26, 12.3, 8.52},
                        {q1[0], q1[1], q1[2], q1[3]},
                        {0.35, 1.27},
                        {-10.0, 8.25, 4.56}};

    cfc::Shape3D e2 = {{15.8, 12.5, 8.1},
                       {q2[0], q2[1], q2[2], q2[3]},
                       {1.0, 1.0},
                       {15.3, -2.2, 2.84}};

    return std::make_pair(sq1, e2);
}

// CFC-based distance
auto getCFCDistance(cfc::Shape3D s1, cfc::Shape3D s2) {
    cfc::collision_cfc::DistanceLeastSquares<cfc::SuperQuadrics, cfc::Ellipsoid>
        mink(s1, s2);
    mink.query();

    return mink.getDistanceInfo();
}

// FCL distance
auto getFCLDistance(cfc::Shape3D s1, cfc::Shape3D s2) {
    DistanceFCL<cfc::SuperQuadrics, cfc::Ellipsoid, fcl::OBBRSSd> fclDist(
        s1, s2, SURFACE_MESH_PARAM);
    fclDist.query();

    return fclDist.getDistanceInfo();
}

// Implicit-surface distance
auto getImplicitDistance(cfc::Shape3D s1, cfc::Shape3D s2) {
    collision_implicit::DistanceImplicit<cfc::SuperQuadrics, cfc::Ellipsoid>
        implicit(s1, s2);
    implicit.query();

    return implicit.getDistanceInfo();
}

// Test necessary condition: line connecting two closest points should be
// parallel to surface normal
TEST(ESDistance3D, necessaryCondition) {
    auto shapes = initFixedES();

    auto dist_info_mink = getCFCDistance(shapes.first, shapes.second);

    // Test result
    EXPECT_LE(dist_info_mink.necessary_condition, ACCURACY_THRESHOLD);
}

// Test distance results with FCL: should be very close, but Minkowski-based
// should be smaller than FCL
TEST(ESDistance3D, distanceWithFCL) {
    auto shapes = initFixedES();

    auto dist_info_mink = getCFCDistance(shapes.first, shapes.second);
    auto dist_info_fcl = getFCLDistance(shapes.first, shapes.second);

    // Test results
    EXPECT_LE(dist_info_mink.distance,
              std::fabs(dist_info_fcl.distance + ACCURACY_THRESHOLD));
}

// Test collision status results with FCL: should be the same
TEST(ESDistance3D, statusWithFCL) {
    auto shapes = initFixedES();

    auto dist_info_mink = getCFCDistance(shapes.first, shapes.second);
    auto dist_info_fcl = getFCLDistance(shapes.first, shapes.second);

    // Test results
    EXPECT_EQ(dist_info_mink.is_collision, dist_info_fcl.is_collision);
}

// Test distance results with Implicit-surface: should be very close
TEST(ESDistance3D, distanceWithImplicit) {
    auto shapes = initFixedES();

    auto dist_info_mink = getCFCDistance(shapes.first, shapes.second);
    auto dist_info_implicit = getImplicitDistance(shapes.first, shapes.second);

    // Test results
    EXPECT_LE(dist_info_mink.distance,
              dist_info_implicit.distance + ACCURACY_THRESHOLD);
}

// Test collision status results with Implicit-surface: should be the same
TEST(ESDistance3D, statusWithImplicit) {
    auto shapes = initFixedES();

    auto dist_info_mink = getCFCDistance(shapes.first, shapes.second);
    auto dist_info_implicit = getImplicitDistance(shapes.first, shapes.second);

    // Test results
    EXPECT_EQ(dist_info_mink.is_collision, dist_info_implicit.is_collision);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
