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

#include "DistanceQueryUtils.h"
#include "fcl_interface/DistanceFCL.h"

#include <chrono>

typedef std::chrono::high_resolution_clock Clock;

/** \brief Query distance using optimization-based algorithms */
template <typename C>
void queryDistanceOptimization(const cfc::Shape3D* s1, const cfc::Shape3D* s2,
                               BenchmarkMetric* metric,
                               std::vector<cfc::DistanceResult>* dist_info) {
    auto start = Clock::now();

    // Initialize collision detection algorithm
    C algorithm(*s1, *s2);

    auto end = Clock::now();
    chrono::duration<double, std::milli> time_init = end - start;

    start = Clock::now();

    // Query distance
    algorithm.query();

    end = Clock::now();
    chrono::duration<double, std::milli> time_query = end - start;

    // Record time and distance info
    metric->timeInit.emplace_back(time_init.count());
    metric->timeQuery.emplace_back(time_query.count());
    dist_info->emplace_back(algorithm.getDistanceInfo());
}

/** \brief Query distance using FCL methods */
template <typename G1, typename G2, typename BV>
void queryDistanceFCL(const cfc::Shape3D* s1, const cfc::Shape3D* s2,
                      const int numVtx, BenchmarkMetric* metric,
                      std::vector<cfc::DistanceResult>* distInfo) {
    // Intialize FCL collision objects
    auto start = Clock::now();

    DistanceFCL<G1, G2, BV> fcl(*s1, *s2, numVtx);

    auto end = Clock::now();
    chrono::duration<double, std::milli> timeInit = end - start;

    // Query distance
    start = Clock::now();

    fcl.query();

    end = Clock::now();
    chrono::duration<double, std::milli> timeQuery = end - start;

    // Record time and distance info
    metric->timeInit.emplace_back(timeInit.count());
    metric->timeQuery.emplace_back(timeQuery.count());
    distInfo->emplace_back(fcl.getDistanceInfo());
}

/** \brief Query continuous distance using optimization-based algorithms */
template <typename C>
void queryContinuousDistanceOptimization(
    const cfc::Shape3D* s1, const Eigen::Affine3d* g1_start,
    const Eigen::Affine3d* g2_start, const cfc::Shape3D* s2,
    const Eigen::Affine3d* g1_goal, const Eigen::Affine3d* g2_goal,
    const cfc::ContinuousDistanceRequest& ccd_request, BenchmarkMetric* metric,
    std::vector<cfc::ContinuousDistanceResult>* continuous_dist_info) {
    auto start = Clock::now();

    // Initialize collision detection algorithm
    C algorithm(*s1, *g1_start, *g1_goal, *s2, *g2_start, *g2_goal,
                ccd_request);

    auto end = Clock::now();
    chrono::duration<double, std::milli> time_init = end - start;

    start = Clock::now();

    // Query distance
    algorithm.query();

    end = Clock::now();
    chrono::duration<double, std::milli> time_query = end - start;

    // Record time and distance info
    metric->timeInit.emplace_back(time_init.count());
    metric->timeQuery.emplace_back(time_query.count());
    continuous_dist_info->emplace_back(algorithm.getDistanceInfo());
}
