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

#include "BenchmarkContinuousDistance.h"
#include "cfc/ContinuousDistanceLeastSquares.h"
#include "config.h"
#include "test/util/DistanceQueryUtils.h"
#include "util/FCLUtils.h"

template <typename C>
pair<BenchmarkMetric, vector<cfc::ContinuousDistanceResult>>
benchmarkContinuousOptimization(
    const vector<cfc::Shape3D>& s1,
    const vector<pair<Eigen::Affine3d, Eigen::Affine3d>>& g1,
    const vector<cfc::Shape3D>& s2,
    const vector<pair<Eigen::Affine3d, Eigen::Affine3d>>& g2,
    const cfc::ContinuousDistanceRequest& req, const int iter) {
    // Container for benchmark results
    BenchmarkMetric metric;

    // Container of distance information
    vector<cfc::ContinuousDistanceResult> continuous_dist_info;

    for (size_t i = 0; i < size_t(iter); i++) {
        if (i % NUM_TRIALS_DISPLAY == 0) {
            cout << "  - iteration -- " << i << endl;
        }

        // Benchmark
        queryContinuousDistanceOptimization<C>(
            &s1.at(i), &g1.at(i).first, &g1.at(i).second, &s2.at(i),
            &g2.at(i).first, &g2.at(i).second, req, &metric,
            &continuous_dist_info);

        // Record results
        metric.timeInitAvg += metric.timeInit.at(i);
        metric.timeQueryAvg += metric.timeQuery.at(i);
    }
    metric.timeInitAvg /= static_cast<double>(iter);
    metric.timeQueryAvg /= static_cast<double>(iter);

    // Compute accuracy
    computeAccuracy(continuous_dist_info, &metric);

    return make_pair(metric, continuous_dist_info);
}

template <typename G1, typename G2>
void benchmarkAlgorithms(const int iter, const int numVtx) {
    cout << "**************************************************" << endl;
    cout << "     Continuous Collision Detection Benchmark     " << endl;
    cout << "**************************************************" << endl;

    // Initialize the two shapes and start/goal poses
    vector<cfc::Shape3D> s1;
    vector<cfc::Shape3D> s2;
    vector<pair<Eigen::Affine3d, Eigen::Affine3d>> g1;
    vector<pair<Eigen::Affine3d, Eigen::Affine3d>> g2;

    cout << "Shapes: ";
    string s1_file_name = defineRandomShape<G1>(iter, s1);
    defineMultiRandStartGoal(iter, g1);

    cout << " - ";
    string s2_file_name = defineRandomShape<G2>(iter, s2);
    defineMultiRandStartGoal(iter, g2);

    cout << endl;

    // Initialize random number generator
    cout << "Num of trials: " << iter << endl;
    cout << "Num of vertices on surface: " << numVtx << endl;

    // Benchmark
    cout << "**************************************************" << endl;
    cout << "                Start benchmarking                " << endl;
    cout << "**************************************************" << endl;

    cfc::ContinuousDistanceRequest request;

    /* Linear interpolated motion */
    request.motion_type = cfc::CCD_MOTION_TYPE::PCG_LINEAR;

    // CFC-based, gradient + least-squares
    cout << "- CFC + Least-squares + Linear motion" << endl;
    auto res_cfc_grad_lsq_linear = benchmarkContinuousOptimization<
        cfc::collision_cfc::ContinuousDistanceLeastSquares<G1, G2>>(
        s1, g1, s2, g2, request, iter);

    /* Translation only */
    request.motion_type = cfc::CCD_MOTION_TYPE::TRAN;

    // CFC-based, gradient + least-squares
    cout << "- CFC + Least-squares + Translation motion" << endl;
    auto res_cfc_grad_lsq_tran = benchmarkContinuousOptimization<
        cfc::collision_cfc::ContinuousDistanceLeastSquares<G1, G2>>(
        s1, g1, s2, g2, request, iter);

    // Display average ellapsed time
    cout << "**************************************************" << endl;
    cout << "                      Results                     " << endl;
    cout << "**************************************************" << endl;

    cout << "Average ellapsed time:" << endl;
    cout << "- CFC + Least-squares + Linear motion -- Init: "
         << res_cfc_grad_lsq_linear.first.timeInitAvg
         << " ms, Query: " << res_cfc_grad_lsq_linear.first.timeQueryAvg
         << " ms" << endl;

    cout << "- CFC + Least-squares + Translation motion -- Init: "
         << res_cfc_grad_lsq_tran.first.timeInitAvg
         << " ms, Query: " << res_cfc_grad_lsq_tran.first.timeQueryAvg << " ms"
         << endl;

    cout << "--------------------------------------------------" << endl;
    cout << "Accuracy:" << endl;
    cout << "- CFC + Least-squares + Linear motion -- "
         << res_cfc_grad_lsq_linear.first.conditionAvg << ", "
         << res_cfc_grad_lsq_linear.first.accuracy << " %" << endl;

    cout << "- CFC + Least-squares + Translation motion -- "
         << res_cfc_grad_lsq_tran.first.conditionAvg << ", "
         << res_cfc_grad_lsq_tran.first.accuracy << " %" << endl;

    // Store configurations of objects
    const std::string res_file_folder =
        BENCHMARK_DATA_PATH_CONTINUOUS "/" + s1_file_name + "-" + s2_file_name;

    const std::string config_file_prefix =
        res_file_folder + "/bench_config_continuous_";
    storeConfig(s1, g1, config_file_prefix + s1_file_name, "s1");
    storeConfig(s2, g2, config_file_prefix + s2_file_name, "s2");

    // Store results
    const std::string res_file_prefix = res_file_folder +
                                        "/bench_result_continuous_" +
                                        s1_file_name + "_" + s2_file_name;
    storeResults(res_cfc_grad_lsq_linear.second,
                 res_cfc_grad_lsq_linear.first.timeInit,
                 res_cfc_grad_lsq_linear.first.timeQuery, res_file_prefix,
                 "CFCLeastSquaresLinear");
    storeResults(res_cfc_grad_lsq_tran.second,
                 res_cfc_grad_lsq_tran.first.timeInit,
                 res_cfc_grad_lsq_tran.first.timeQuery, res_file_prefix,
                 "CFCLeastSquaresTran");
}
