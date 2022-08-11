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

#include "test/util/Utils.h"

#include "eigen3/Eigen/Dense"

#include <fstream>
#include <random>

cfc::Shape3D initRandomSuperQuadrics() {
    std::srand(static_cast<unsigned int>(clock()));

    // Random normalized quaternion
    Eigen::Vector4d q;
    q = {2.0 * rand() / RAND_MAX - 1.0, 2.0 * rand() / RAND_MAX - 1.0,
         2.0 * rand() / RAND_MAX - 1.0, 2.0 * rand() / RAND_MAX - 1.0};
    q.normalize();

    // Initialize shapes
    const cfc::Shape3D s = {
        {20.0 * rand() / RAND_MAX + 2.0, 6.0 * rand() / RAND_MAX + 2.0,
         16.0 * rand() / RAND_MAX + 2.0},
        {q[0], q[1], q[2], q[3]},
        {1.8 * rand() / RAND_MAX + 0.1, 1.8 * rand() / RAND_MAX + 0.1},
        {30.0 * (2.0 * rand() / RAND_MAX - 1.0),
         30.0 * (2.0 * rand() / RAND_MAX - 1.0),
         30.0 * (2.0 * rand() / RAND_MAX - 1.0)}};

    return s;
}

cfc::Shape3D initRandomEllipsoid() {
    std::srand(static_cast<unsigned int>(clock()));

    // Random normalized quaternion
    Eigen::Vector4d q;
    q = {2.0 * rand() / RAND_MAX - 1.0, 2.0 * rand() / RAND_MAX - 1.0,
         2.0 * rand() / RAND_MAX - 1.0, 2.0 * rand() / RAND_MAX - 1.0};
    q.normalize();

    // Initialize shapes
    const cfc::Shape3D e = {
        {20.0 * rand() / RAND_MAX + 2.0, 15.0 * rand() / RAND_MAX + 2.0,
         8.0 * rand() / RAND_MAX + 2.0},
        {q[0], q[1], q[2], q[3]},
        {1.0, 1.0},
        {30.0 * (2.0 * rand() / RAND_MAX - 1.0),
         30.0 * (2.0 * rand() / RAND_MAX - 1.0),
         30.0 * (2.0 * rand() / RAND_MAX - 1.0)}};

    return e;
}

cfc::Shape3D initRandomPolyEllipsoid() {
    std::srand(static_cast<unsigned int>(clock()));

    // Random normalized quaternion
    Eigen::Vector4d q;
    q = {2.0 * rand() / RAND_MAX - 1.0, 2.0 * rand() / RAND_MAX - 1.0,
         2.0 * rand() / RAND_MAX - 1.0, 2.0 * rand() / RAND_MAX - 1.0};
    q.normalize();

    // Initialize shapes
    const cfc::Shape3D pe = {
        {10.0 * rand() / RAND_MAX + 2.0, 8.0 * rand() / RAND_MAX + 2.0,
         6.0 * rand() / RAND_MAX + 2.0},
        {q[0], q[1], q[2], q[3]},
        {1.0, 1.0},
        {30.0 * (2.0 * rand() / RAND_MAX - 1.0),
         30.0 * (2.0 * rand() / RAND_MAX - 1.0),
         30.0 * (2.0 * rand() / RAND_MAX - 1.0)},
        {12.0 * rand() / RAND_MAX + 2.0, 25.0 * rand() / RAND_MAX + 2.0,
         20.0 * rand() / RAND_MAX + 2.0}};

    return pe;
}

std::pair<cfc::Shape3D, cfc::Shape3D> initFixedSuperQuadrics() {
    // Initialize shapes
    const cfc::Shape3D sq1 = {
        {15, 10, 5}, {0, 1, 0, 0}, {0.5, 0.7}, {2.82, -16.4, 7.52}};
    const cfc::Shape3D sq2 = {
        {5, 3, 2}, {0, 1, 0, 0}, {1.25, 0.2}, {-0.47, 8.44, 0.8}};

    return std::make_pair(sq1, sq2);
}

std::pair<cfc::Shape3D, cfc::Shape3D> initFixedEllipsoids() {
    // Initialize shapes
    const cfc::Shape3D e1 = {
        {20, 15, 10}, {0, 1, 0, 0}, {1.0, 1.0}, {-10.0, 7.25, 3.56}};
    const cfc::Shape3D e2 = {
        {15, 12, 8}, {0, 1, 0, 0}, {1.0, 1.0}, {15.3, -2.2, 2.84}};

    return std::make_pair(e1, e2);
}

std::pair<cfc::Shape3D, cfc::Shape3D> initFixedPolyEllipsoids() {
    // Initialize shapes
    const cfc::Shape3D pe1 = {
        {20, 10, 5}, {0, 1, 0, 0}, {1, 1}, {-5, 1, 8}, {25, 15, 8}};
    const cfc::Shape3D pe2 = {
        {15, 12, 8}, {0, 1, 0, 0}, {1, 1}, {5, -6, 4}, {18, 12.5, 10}};

    return std::make_pair(pe1, pe2);
}

std::pair<Eigen::Affine3d, Eigen::Affine3d> defFixedStartPoses() {
    const Eigen::Quaterniond q1(
        Eigen::Vector4d({-0.5, -0.5, 0.7, 0.1}).normalized());
    const Eigen::Quaterniond q2(
        Eigen::Vector4d({-0.5, -0.2, 0.3, 0.75}).normalized());

    Eigen::Affine3d g1_start;
    Eigen::Affine3d g2_start;

    g1_start.linear() = q1.toRotationMatrix();
    g2_start.linear() = q2.toRotationMatrix();

    g1_start.translation() = Eigen::Vector3d({3.0, -16, 7.5});
    g2_start.translation() = Eigen::Vector3d({-0.5, 8.5, 1.0});

    return std::make_pair(g1_start, g2_start);
}

std::pair<Eigen::Affine3d, Eigen::Affine3d> defFixedGoalPoses() {
    const Eigen::Quaterniond q1(
        Eigen::Vector4d({0.56, -0.42, -0.64, 0.3}).normalized());
    const Eigen::Quaterniond q2(
        Eigen::Vector4d({0.46, 0.77, 0.1, -0.42}).normalized());

    Eigen::Affine3d g1_goal;
    Eigen::Affine3d g2_goal;

    g1_goal.linear() = q1.toRotationMatrix();
    g2_goal.linear() = q2.toRotationMatrix();

    g1_goal.translation() = Eigen::Vector3d({37.8, 25.0, 1.5});
    g2_goal.translation() = Eigen::Vector3d({42.0, 5.5, -65.25});

    return std::make_pair(g1_goal, g2_goal);
}

std::pair<Eigen::Affine3d, Eigen::Affine3d> defRandStartGoalPoses() {
    Eigen::Affine3d g1;
    Eigen::Affine3d g2;

    // Unit random quaternions
    Eigen::Quaterniond q1(Eigen::Vector4d({2.0 * rand() / RAND_MAX - 1.0,
                                           2.0 * rand() / RAND_MAX - 1.0,
                                           2.0 * rand() / RAND_MAX - 1.0,
                                           2.0 * rand() / RAND_MAX - 1.0})
                              .normalized());

    Eigen::Quaterniond q2(Eigen::Vector4d({2.0 * rand() / RAND_MAX - 1.0,
                                           2.0 * rand() / RAND_MAX - 1.0,
                                           2.0 * rand() / RAND_MAX - 1.0,
                                           2.0 * rand() / RAND_MAX - 1.0})
                              .normalized());

    // Set random rotations and translations
    g1.linear() = q1.toRotationMatrix();
    g2.linear() = q2.toRotationMatrix();

    g1.translation() =
        Eigen::Vector3d({5.0 * (2.0 * rand() / RAND_MAX - 1.0),
                         25.0 * (2.0 * rand() / RAND_MAX - 1.0),
                         10.0 * (2.0 * rand() / RAND_MAX - 1.0)});
    g2.translation() =
        Eigen::Vector3d({50.0 * (2.0 * rand() / RAND_MAX - 1.0),
                         45.0 * (2.0 * rand() / RAND_MAX - 1.0),
                         70.0 * (2.0 * rand() / RAND_MAX - 1.0)});

    return std::make_pair(g1, g2);
}

void defineMultiRandStartGoal(
    const int numTrials,
    std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>& g) {
    for (size_t i = 0; i < size_t(numTrials); ++i) {
        g.emplace_back(defRandStartGoalPoses());
    }
}

void storeConfig(const std::vector<cfc::Shape3D>& object,
                 const std::string& file_prefix, const std::string& obj_name) {
    std::ofstream config_file;
    config_file.open(file_prefix + '_' + obj_name + ".csv");
    config_file << "Object" << ',' << "Semi_a" << ',' << "Semi_b" << ','
                << "Semi_c" << ',' << "Epsilon_1" << ',' << "Epsilon_2" << ','
                << "Pos_x" << ',' << "Pos_y" << ',' << "Pos_z" << ','
                << "Quat_1" << ',' << "Quat_2" << ',' << "Quat_3" << ','
                << "Quat_4" << std::endl;
    for (const auto& obj : object) {
        config_file << obj_name << ',' << obj.a[0] << ',' << obj.a[1] << ','
                    << obj.a[2] << ',' << obj.eps[0] << ',' << obj.eps[1] << ','
                    << obj.pos[0] << ',' << obj.pos[1] << ',' << obj.pos[2]
                    << ',' << obj.q[0] << ',' << obj.q[1] << ',' << obj.q[2]
                    << ',' << obj.q[3] << std::endl;
    }
    config_file.close();
}

void storeConfig(
    const std::vector<cfc::Shape3D>& object,
    const std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>& g,
    const std::string& file_prefix, const std::string& obj_name) {
    std::ofstream config_file;
    config_file.open(file_prefix + '_' + obj_name + ".csv");
    config_file << "Object" << ',' << "Semi_a" << ',' << "Semi_b" << ','
                << "Semi_c" << ',' << "Epsilon_1" << ',' << "Epsilon_2" << ','
                << "Start_Pos_x" << ',' << "Start_Pos_y" << ',' << "Start_Pos_z"
                << ',' << "Start_Quat_1" << ',' << "Start_Quat_2" << ','
                << "Start_Quat_3" << ',' << "Start_Quat_4"
                << "Goal_Pos_x" << ',' << "Goal_Pos_y" << ',' << "Goal_Pos_z"
                << ',' << "Goal_Quat_1" << ',' << "Goal_Quat_2" << ','
                << "Goal_Quat_3" << ',' << "Goal_Quat_4" << std::endl;

    for (size_t i = 0; i < object.size(); i++) {
        const Eigen::Vector3d tc_start = g.at(i).first.translation();
        const Eigen::Vector3d tc_goal = g.at(i).second.translation();
        const Eigen::Quaterniond q_start(g.at(i).first.linear());
        const Eigen::Quaterniond q_goal(g.at(i).second.linear());

        config_file << obj_name << ',' << object.at(i).a[0] << ','
                    << object.at(i).a[1] << ',' << object.at(i).a[2] << ','
                    << object.at(i).eps[0] << ',' << object.at(i).eps[1] << ','
                    << tc_start[0] << ',' << tc_start[1] << ',' << tc_start[2]
                    << ',' << q_start.w() << ',' << q_start.x() << ','
                    << q_start.y() << ',' << q_start.z() << ',' << tc_goal[0]
                    << ',' << tc_goal[1] << ',' << tc_goal[2] << ','
                    << q_goal.w() << ',' << q_goal.x() << ',' << q_goal.y()
                    << ',' << q_goal.z() << std::endl;
    }

    config_file.close();
}

void storeResults(const std::vector<cfc::DistanceResult>& dist_info,
                  const std::vector<double>& time_init,
                  const std::vector<double>& time_query,
                  const std::string& file_prefix, const std::string& method) {
    std::ofstream res_file;
    res_file.open(file_prefix + '_' + method + ".csv");

    res_file << "method" << ',' << "is_collision" << ',' << "distance" << ','
             << "normal_x" << ',' << "normal_y" << ',' << "normal_z" << ','
             << "closest_point_s1_x" << ',' << "closest_point_s1_y" << ','
             << "closest_point_s1_z" << ',' << "closest_point_s2_x" << ','
             << "closest_point_s2_y" << ',' << "closest_point_s2_z" << ','
             << "point_on_contact_space_x" << ',' << "point_on_contact_space_y"
             << ',' << "point_on_contact_space_z" << ',' << "condition" << ','
             << "time_init" << ',' << "time_query" << ',' << "time_total" << ','
             << "num_iter" << std::endl;
    for (size_t i = 0; i < dist_info.size(); i++) {
        res_file << method << ',' << dist_info.at(i).is_collision << ','
                 << dist_info.at(i).distance << ','
                 << dist_info.at(i).optimal_normal[0] << ','
                 << dist_info.at(i).optimal_normal[1] << ','
                 << dist_info.at(i).optimal_normal[2] << ','
                 << dist_info.at(i).closest_point_s1[0] << ','
                 << dist_info.at(i).closest_point_s1[1] << ','
                 << dist_info.at(i).closest_point_s1[2] << ','
                 << dist_info.at(i).closest_point_s2[0] << ','
                 << dist_info.at(i).closest_point_s2[1] << ','
                 << dist_info.at(i).closest_point_s2[2] << ','
                 << dist_info.at(i).point_on_contact_space[0] << ','
                 << dist_info.at(i).point_on_contact_space[1] << ','
                 << dist_info.at(i).point_on_contact_space[2] << ','
                 << dist_info.at(i).necessary_condition << ','
                 << time_init.at(i) << ',' << time_query.at(i) << ','
                 << time_init.at(i) + time_query.at(i) << ','
                 << dist_info.at(i).num_iteration << std::endl;
    }

    res_file.close();
}

void storeResults(
    const std::vector<cfc::ContinuousDistanceResult>& continuous_dist_info,
    const std::vector<double>& time_init, const std::vector<double>& time_query,
    const std::string& file_prefix, const std::string& method) {
    std::ofstream res_file;
    res_file.open(file_prefix + '_' + method + ".csv");

    res_file << "method" << ',' << "is_collision" << ',' << "distance" << ','
             << "optimal_time" << ',' << "normal_x" << ',' << "normal_y" << ','
             << "normal_z" << ',' << "closest_point_s1_x" << ','
             << "closest_point_s1_y" << ',' << "closest_point_s1_z" << ','
             << "closest_point_s2_x" << ',' << "closest_point_s2_y" << ','
             << "closest_point_s2_z" << ',' << "point_on_contact_space_x" << ','
             << "point_on_contact_space_y" << ',' << "point_on_contact_space_z"
             << ',' << "condition" << ',' << "time_init" << ',' << "time_query"
             << ',' << "time_total" << ',' << "num_iter" << std::endl;

    for (size_t i = 0; i < continuous_dist_info.size(); ++i) {
        res_file << method << ',' << continuous_dist_info.at(i).is_collision
                 << ',' << continuous_dist_info.at(i).distance << ','
                 << continuous_dist_info.at(i).optimal_time << ','
                 << continuous_dist_info.at(i).optimal_normal[0] << ','
                 << continuous_dist_info.at(i).optimal_normal[1] << ','
                 << continuous_dist_info.at(i).optimal_normal[2] << ','
                 << continuous_dist_info.at(i).closest_point_s1[0] << ','
                 << continuous_dist_info.at(i).closest_point_s1[1] << ','
                 << continuous_dist_info.at(i).closest_point_s1[2] << ','
                 << continuous_dist_info.at(i).closest_point_s2[0] << ','
                 << continuous_dist_info.at(i).closest_point_s2[1] << ','
                 << continuous_dist_info.at(i).closest_point_s2[2] << ','
                 << continuous_dist_info.at(i).point_on_contact_space[0] << ','
                 << continuous_dist_info.at(i).point_on_contact_space[1] << ','
                 << continuous_dist_info.at(i).point_on_contact_space[2] << ','
                 << continuous_dist_info.at(i).necessary_condition << ','
                 << time_init.at(i) << ',' << time_query.at(i) << ','
                 << time_init.at(i) + time_query.at(i) << ','
                 << continuous_dist_info.at(i).num_iteration << std::endl;
    }

    res_file.close();
}
