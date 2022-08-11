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

#include "geometry/GeometryInfo.h"

#include <iostream>
#include <typeinfo>
#include <utility>
#include <vector>

static const double ACCURACY_THRESHOLD = 1e-4;
static const unsigned int NUM_TRIALS_DISPLAY = 100;

/** \brief Initialization of different object shapes */
cfc::Shape3D initRandomSuperQuadrics();
cfc::Shape3D initRandomEllipsoid();
cfc::Shape3D initRandomPolyEllipsoid();

template <typename G>
std::string defineRandomShape(const int numTrials,
                              std::vector<cfc::Shape3D>& s);
std::pair<cfc::Shape3D, cfc::Shape3D> initFixedSuperQuadrics();
std::pair<cfc::Shape3D, cfc::Shape3D> initFixedEllipsoids();
std::pair<cfc::Shape3D, cfc::Shape3D> initFixedPolyEllipsoids();

/** \brief Definition of start and goal poses for CCD */
std::pair<Eigen::Affine3d, Eigen::Affine3d> defFixedStartPoses();
std::pair<Eigen::Affine3d, Eigen::Affine3d> defFixedGoalPoses();
std::pair<Eigen::Affine3d, Eigen::Affine3d> defRandStartGoalPoses();
void defineMultiRandStartGoal(
    const int numTrials,
    std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>& g);

/** \brief Metric for benchmark */
struct BenchmarkMetric {
    /** \brief Computation time for algorithm initialization */
    std::vector<double> timeInit;

    /** \brief Computation time for collision query */
    std::vector<double> timeQuery;

    /** \brief Averaged computation time for algorithm initialization */
    double timeInitAvg = 0.0;

    /** \brief Averaged computation time for collision query */
    double timeQueryAvg = 0.0;

    /** \brief List of collision status for all trials */
    std::vector<bool> flag;

    /** \brief Averaged necessary condition for optimality */
    double conditionAvg = 0.0;

    /** \brief Accuracy metric */
    double accuracy = 0.0;
};

/** \brief Compute accuracy for optimization-based algorithms */
template <typename R>
void computeAccuracy(const std::vector<R>& dist_info, BenchmarkMetric* metric);

/** \brief Store the object configuration */
void storeConfig(const std::vector<cfc::Shape3D>& object,
                 const std::string& file_prefix, const std::string& obj_name);

void storeConfig(
    const std::vector<cfc::Shape3D>& object,
    const std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>& g,
    const std::string& file_prefix, const std::string& obj_name);

/** \brief Store the testing results */
void storeResults(const std::vector<cfc::DistanceResult>& dist_info,
                  const std::vector<double>& time_init,
                  const std::vector<double>& time_query,
                  const std::string& file_prefix, const std::string& method);

void storeResults(
    const std::vector<cfc::ContinuousDistanceResult>& continuous_dist_info,
    const std::vector<double>& time_init, const std::vector<double>& time_query,
    const std::string& file_prefix, const std::string& method);

#include "Utils-inl.h"
