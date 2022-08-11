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

#include "cfc/ContinuousDistanceLeastSquares.h"
#include "util/Utils.h"

#include "gtest/gtest.h"

/** \brief Test necessary condition: line connecting two
 * closest points should be parallel to surface gradient */

/** SQ-SQ */
TEST(NecessaryCondition, ContinuousGradientSQ) {
    auto shapes = initFixedSuperQuadrics();
    auto start_poses = defFixedStartPoses();
    auto goal_poses = defFixedGoalPoses();

    cfc::ContinuousDistanceRequest request;
    request.motion_type = cfc::CCD_MOTION_TYPE::PCG_LINEAR;

    cfc::collision_cfc::ContinuousDistanceLeastSquares<cfc::SuperQuadrics,
                                                       cfc::SuperQuadrics>
        ccd(shapes.first, start_poses.first, goal_poses.first, shapes.second,
            start_poses.second, goal_poses.second, request);
    ccd.query();

    auto dist_info = ccd.getDistanceInfo();

    // Check necessary condition
    ASSERT_LE(dist_info.necessary_condition, ACCURACY_THRESHOLD);

    // Check normal validity
    EXPECT_DOUBLE_EQ(dist_info.optimal_normal.norm(), 1.0);
}

/** E-E */
TEST(NecessaryCondition, ContinuousGradientEE) {
    auto shapes = initFixedEllipsoids();
    auto start_poses = defFixedStartPoses();
    auto goal_poses = defFixedGoalPoses();

    cfc::ContinuousDistanceRequest request;
    request.motion_type = cfc::CCD_MOTION_TYPE::PCG_LINEAR;

    cfc::collision_cfc::ContinuousDistanceLeastSquares<cfc::Ellipsoid,
                                                       cfc::Ellipsoid>
        ccd(shapes.first, start_poses.first, goal_poses.first, shapes.second,
            start_poses.second, goal_poses.second, request);
    ccd.query();

    auto dist_info = ccd.getDistanceInfo();

    // Check necessary condition
    ASSERT_LE(dist_info.necessary_condition, ACCURACY_THRESHOLD);

    // Check normal validity
    EXPECT_DOUBLE_EQ(dist_info.optimal_normal.norm(), 1.0);
}

/** PolyE-PolyE */
TEST(NecessaryCondition, ContinuousGradientPolyE) {
    auto shapes = initFixedPolyEllipsoids();
    auto start_poses = defFixedStartPoses();
    auto goal_poses = defFixedGoalPoses();

    cfc::ContinuousDistanceRequest request;
    request.motion_type = cfc::CCD_MOTION_TYPE::PCG_LINEAR;

    cfc::collision_cfc::ContinuousDistanceLeastSquares<cfc::PolyEllipsoid,
                                                       cfc::PolyEllipsoid>
        ccd(shapes.first, start_poses.first, goal_poses.first, shapes.second,
            start_poses.second, goal_poses.second, request);
    ccd.query();

    auto dist_info = ccd.getDistanceInfo();

    // Check necessary condition
    ASSERT_LE(dist_info.necessary_condition, ACCURACY_THRESHOLD);

    // Check normal validity
    EXPECT_DOUBLE_EQ(dist_info.optimal_normal.norm(), 1.0);
}

int main(int argc, char **argv) {
    std::cout << "Unit tests for continuous collision using "
                 "gradient-parameterized closed-form contact space. Using "
                 "nonlinear least-squares optimization."
              << std::endl;

    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_repeat = argc > 1 ? stoi(argv[1]) : 1;
    return RUN_ALL_TESTS();
}
