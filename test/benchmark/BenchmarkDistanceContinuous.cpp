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

#include "BenchmarkContinuousDistance.h"

/** \brief Main script for benchmarking narrow-phase continuous collision
 * detection algorithms
 * \param argv[1] iter, number of trials
 * \param argv[2] shapes, define two shape types
 * \param argv[3] numVtx (optional), number of vertices on discrete surfaces */

int main(int argc, char** argv) {
    if (argc <= 2) {
        std::cerr
            << "Incorrect inputs. Please provide: (1) Number of trials; "
               "(2) Shape types of the two bodies (i.e., all, SQ-SQ, etc); (3) "
               "[optional] Number of vertex parameters on the surface "
               "(for FCL)."
            << std::endl;
    }

    const int iter = stoi(argv[1]);
    const std::string shapePair = argv[2];
    const int numVtx = argc > 3 ? stoi(argv[3]) : 20;

    std::vector<std::string> typeShapes;
    if (shapePair == "all") {
        typeShapes = {"SQ-SQ", "SQ-E", "SQ-PE", "E-E", "E-PE", "PE-PE"};
    } else {
        typeShapes.emplace_back(shapePair);
    }

    for (auto typeShape : typeShapes) {
        if (typeShape == "SQ-SQ") {
            benchmarkAlgorithms<cfc::SuperQuadrics, cfc::SuperQuadrics>(iter,
                                                                        numVtx);
        } else if (typeShape == "SQ-E") {
            benchmarkAlgorithms<cfc::SuperQuadrics, cfc::Ellipsoid>(iter,
                                                                    numVtx);
        } else if (typeShape == "SQ-PE") {
            benchmarkAlgorithms<cfc::SuperQuadrics, cfc::PolyEllipsoid>(iter,
                                                                        numVtx);
        } else if (typeShape == "E-E") {
            benchmarkAlgorithms<cfc::Ellipsoid, cfc::Ellipsoid>(iter, numVtx);
        } else if (typeShape == "E-PE") {
            benchmarkAlgorithms<cfc::Ellipsoid, cfc::PolyEllipsoid>(iter,
                                                                    numVtx);
        } else if (typeShape == "PE-PE") {
            benchmarkAlgorithms<cfc::PolyEllipsoid, cfc::PolyEllipsoid>(iter,
                                                                        numVtx);
        } else {
            std::cerr << "Shape pair NOT SUPPORTED!" << std::endl;
        }
    }

    return 0;
}
