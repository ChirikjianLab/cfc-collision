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

#include "Utils.h"
#include "geometry/Ellipsoid.h"
#include "geometry/PolyEllipsoid.h"
#include "geometry/SuperQuadrics.h"

template <typename G>
std::string defineRandomShape(const int numTrials,
                              std::vector<cfc::Shape3D>& s) {
    //    const string type = typeid(G).name();
    string fileName;

    if (typeid(G) == typeid(cfc::Ellipsoid)) {
        fileName = "E";
        std::cout << "Ellipsoid";

        for (size_t i = 0; i < size_t(numTrials); ++i) {
            s.emplace_back(initRandomEllipsoid());
        }
    } else if (typeid(G) == typeid(cfc::SuperQuadrics)) {
        fileName = "SQ";
        std::cout << "Superquadrics";

        for (size_t i = 0; i < size_t(numTrials); ++i) {
            s.emplace_back(initRandomSuperQuadrics());
        }
    } else if (typeid(G) == typeid(cfc::PolyEllipsoid)) {
        fileName = "PE";
        std::cout << "Polyellipsoid";

        for (size_t i = 0; i < size_t(numTrials); ++i) {
            s.emplace_back(initRandomPolyEllipsoid());
        }
    } else {
        std::cerr << "Geometric type not supported!" << std::endl;
    }

    return fileName;
}

template <typename R>
void computeAccuracy(const std::vector<R>& dist_info, BenchmarkMetric* metric) {
    int num = 0;

    for (auto i = 0; i < dist_info.size(); ++i) {
        if (dist_info.at(i).necessary_condition < ACCURACY_THRESHOLD) {
            metric->conditionAvg += dist_info.at(i).necessary_condition;
            metric->accuracy += 1.0;
            metric->flag.emplace_back(true);

            num++;
        } else {
            metric->flag.emplace_back(false);
        }
    }

    metric->conditionAvg /= static_cast<double>(num);
    metric->accuracy /= static_cast<double>(dist_info.size());
    metric->accuracy *= 100.0;
}
