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

#include "BenchmarkDistance.h"
#include "cfc/DistanceFixedPoint.h"
#include "cfc/DistanceLeastSquares.h"
#include "cfc/DistanceLeastSquaresCommonNormal.h"
#include "common_normal/DistanceCommonNormalFixedPoint.h"
#include "common_normal/DistanceCommonNormalLeastSquares.h"
#include "config.h"
#include "implicit/DistanceImplicit.h"
#include "test/util/DistanceQueryUtils.h"
#include "util/FCLUtils.h"

template <typename C>
pair<BenchmarkMetric, vector<cfc::DistanceResult>> benchmarkOptimization(
    const vector<cfc::Shape3D>& s1, const vector<cfc::Shape3D>& s2,
    const int iter) {
    // Container for benchmark results
    BenchmarkMetric metric;

    // Container of distance information
    vector<cfc::DistanceResult> dist_info;

    for (size_t i = 0; i < size_t(iter); i++) {
        if (i % NUM_TRIALS_DISPLAY == 0) {
            cout << "  - iteration -- " << i << endl;
        }

        // Benchmark
        queryDistanceOptimization<C>(&s1.at(i), &s2.at(i), &metric, &dist_info);

        // Record results
        metric.timeInitAvg += metric.timeInit.at(i);
        metric.timeQueryAvg += metric.timeQuery.at(i);
    }
    metric.timeInitAvg /= static_cast<double>(iter);
    metric.timeQueryAvg /= static_cast<double>(iter);

    // Compute accuracy
    computeAccuracy(dist_info, &metric);

    return make_pair(metric, dist_info);
}

template <typename G1, typename G2, typename BV>
pair<BenchmarkMetric, vector<cfc::DistanceResult>> benchmarkFCL(
    const vector<cfc::Shape3D>& s1, const vector<cfc::Shape3D>& s2,
    const int iter, const int numVtx) {
    // Container for benchmark results
    BenchmarkMetric metric;

    // Container of distance information
    vector<cfc::DistanceResult> dist_info;

    for (size_t i = 0; i < size_t(iter); i++) {
        if (i % NUM_TRIALS_DISPLAY == 0) {
            cout << "  - iteration -- " << i << endl;
        }

        // Benchmark
        queryDistanceFCL<G1, G2, BV>(&s1.at(i), &s2.at(i), numVtx, &metric,
                                     &dist_info);

        // Record results
        metric.timeInitAvg += metric.timeInit.at(i);
        metric.timeQueryAvg += metric.timeQuery.at(i);
    }
    metric.timeInitAvg /= static_cast<double>(iter);
    metric.timeQueryAvg /= static_cast<double>(iter);

    // Compute accuracy
    computeAccuracy(dist_info, &metric);

    return make_pair(metric, dist_info);
}

template <typename G1, typename G2>
void benchmarkAlgorithms(const int iter, const int numVtx) {
    // Initialize the two shapes
    cout << "**************************************************" << endl;
    cout << "    Narrow-phase Collision Detection Benchmark    " << endl;
    cout << "**************************************************" << endl;

    vector<cfc::Shape3D> s1;
    vector<cfc::Shape3D> s2;

    cout << "Shapes: ";
    string s1_file_name = defineRandomShape<G1>(iter, s1);
    cout << " - ";
    string s2_file_name = defineRandomShape<G2>(iter, s2);
    cout << endl;

    // Initialize random number generator
    cout << "Num of trials: " << iter << endl;
    cout << "Num of vertices on surface: " << numVtx << endl;

    // Benchmark
    cout << "**************************************************" << endl;
    cout << "                Start benchmarking                " << endl;
    cout << "**************************************************" << endl;

    // CFC + fixed-point
    cout << "- CFC, Gradient + Fixed-point" << endl;
    auto res_cfc_grad_fp =
        benchmarkOptimization<cfc::collision_cfc::DistanceFixedPoint<G1, G2>>(
            s1, s2, iter);

    // CFC + least-squares
    cout << "- CFC, Gradient + Least-squares" << endl;
    auto res_cfc_grad_lsq_dist =
        benchmarkOptimization<cfc::collision_cfc::DistanceLeastSquares<G1, G2>>(
            s1, s2, iter);

    // CFC + least-squares, common normal
    cout << "- CFC, Gradient + Least-squares, common normal" << endl;
    auto res_cfc_grad_lsq_cn = benchmarkOptimization<
        cfc::collision_cfc::DistanceLeastSquaresCommonNormal<G1, G2>>(s1, s2,
                                                                      iter);

    // FCL
    cout << "- FCL" << endl;
    auto res_fcl =
        benchmarkFCL<G1, G2, fcl::OBBRSS<double>>(s1, s2, iter, numVtx);

    // Implicit, interior-point
    cout << "- Implicit + Convex constrained" << endl;
    auto res_implicit =
        benchmarkOptimization<collision_implicit::DistanceImplicit<G1, G2>>(
            s1, s2, iter);

    // Common normal, fixed-point
    cout << "- Common normal + Fixed-point" << endl;
    auto res_cn_fp = benchmarkOptimization<
        collision_common_normal::DistanceCommonNormalFixedPoint<G1, G2>>(s1, s2,
                                                                         iter);

    // Common normal, least-squares
    cout << "- Common normal + Least-squares" << endl;
    auto res_cn_lsq = benchmarkOptimization<
        collision_common_normal::DistanceCommonNormalLeastSquares<G1, G2>>(
        s1, s2, iter);

    // Display average ellapsed time
    cout << "**************************************************" << endl;
    cout << "                      Results                     " << endl;
    cout << "**************************************************" << endl;

    cout << "Average ellapsed time:" << endl;
    cout << "- CFC + Fixed-point -- Init: " << res_cfc_grad_fp.first.timeInitAvg
         << " ms, Query: " << res_cfc_grad_fp.first.timeQueryAvg << " ms"
         << endl;
    cout << "- CFC + Least-squares -- Init: "
         << res_cfc_grad_lsq_dist.first.timeInitAvg
         << " ms, Query: " << res_cfc_grad_lsq_dist.first.timeQueryAvg << " ms"
         << endl;
    cout << "- CFC + Least-squares, common normal -- Init: "
         << res_cfc_grad_lsq_cn.first.timeInitAvg
         << " ms, Query: " << res_cfc_grad_lsq_cn.first.timeQueryAvg << " ms"
         << endl;
    cout << "- FCL -- Init: " << res_fcl.first.timeInitAvg
         << " ms, Query: " << res_fcl.first.timeQueryAvg << " ms" << endl;
    cout << "- Implicit + Convex constrained -- Init: "
         << res_implicit.first.timeInitAvg
         << " ms, Query: " << res_implicit.first.timeQueryAvg << " ms" << endl;
    cout << "- Common normal + Fixed-point -- Init: "
         << res_cn_fp.first.timeInitAvg
         << " ms, Query: " << res_cn_fp.first.timeQueryAvg << " ms" << endl;
    cout << "- Common normal + Least-squares -- Init: "
         << res_cn_lsq.first.timeInitAvg
         << " ms, Query: " << res_cn_lsq.first.timeQueryAvg << " ms" << endl;

    cout << "--------------------------------------------------" << endl;
    cout << "Accuracy:" << endl;
    cout << "- CFC + Fixed-point -- " << res_cfc_grad_fp.first.conditionAvg
         << ", " << res_cfc_grad_fp.first.accuracy << " %" << endl;
    cout << "- CFC + Least-squares -- "
         << res_cfc_grad_lsq_dist.first.conditionAvg << ", "
         << res_cfc_grad_lsq_dist.first.accuracy << " %" << endl;
    cout << "- CFC + Least-squares, common normal -- "
         << res_cfc_grad_lsq_cn.first.conditionAvg << ", "
         << res_cfc_grad_lsq_cn.first.accuracy << " %" << endl;
    cout << "- Implicit + Convex constrained -- "
         << res_implicit.first.conditionAvg << ", "
         << res_implicit.first.accuracy << " %" << endl;
    cout << "- Common normal + Fixed-point -- " << res_cn_fp.first.conditionAvg
         << ", " << res_cn_fp.first.accuracy << " %" << endl;
    cout << "- Common normal + Least-squares -- "
         << res_cn_lsq.first.conditionAvg << ", " << res_cn_lsq.first.accuracy
         << " %" << endl;

    // Store configurations of objects
    const std::string res_file_folder =
        BENCHMARK_DATA_PATH_STATIC "/" + s1_file_name + "-" + s2_file_name;

    const std::string config_file_prefix = res_file_folder + "/bench_config_";
    storeConfig(s1, config_file_prefix + s1_file_name, "s1");
    storeConfig(s2, config_file_prefix + s2_file_name, "s2");

    // Store results
    const string resultFilePrefix =
        res_file_folder + "/bench_result_" + s1_file_name + "_" + s2_file_name;

    storeResults(res_cfc_grad_fp.second, res_cfc_grad_fp.first.timeInit,
                 res_cfc_grad_fp.first.timeQuery, resultFilePrefix,
                 "CFCFixedPoint");
    storeResults(res_cfc_grad_lsq_dist.second,
                 res_cfc_grad_lsq_dist.first.timeInit,
                 res_cfc_grad_lsq_dist.first.timeQuery, resultFilePrefix,
                 "CFCLeastSquares");
    storeResults(res_cfc_grad_lsq_cn.second, res_cfc_grad_lsq_cn.first.timeInit,
                 res_cfc_grad_lsq_cn.first.timeQuery, resultFilePrefix,
                 "CFCLeastSquaresCommonNormal");
    storeResults(res_fcl.second, res_fcl.first.timeInit,
                 res_fcl.first.timeQuery, resultFilePrefix, "FCL");
    storeResults(res_implicit.second, res_implicit.first.timeInit,
                 res_implicit.first.timeQuery, resultFilePrefix, "Implicit");
    storeResults(res_cn_fp.second, res_cn_fp.first.timeInit,
                 res_cn_fp.first.timeQuery, resultFilePrefix,
                 "CommonNormalFixedPoint");
    storeResults(res_cn_lsq.second, res_cn_lsq.first.timeInit,
                 res_cn_lsq.first.timeQuery, resultFilePrefix,
                 "CommonNormalLeastSquares");
}
