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
#include "util/MathUtils.h"

#include "fcl/fcl.h"

/** \brief 3D mesh: vertices and triangles */
struct EMesh {
    /** \brief Vertices for the mesh */
    std::vector<fcl::Vector3<double>> vertices;

    /** \brief Trangular surfaces for the mesh */
    std::vector<fcl::Triangle> triangles;
};

/** \brief Compute mesh from discrete point cloud
 * \param points Point cloud
 * \return Mesh structure */
template <typename P = cfc::ParametricPoints>
EMesh getMesh(P points);

/** \brief Generate surface points from geometric class, in global frame
 * \param obj Geometric object
 * \param num Number of points to be generated on boundary
 * \return cfc::ParametricPoints structure */
template <typename G>
cfc::ParametricPoints getBoundary3D(const G* obj, const int num);

/** \brief Generate surface points from cfc::Shape3D struct, in global frame
 * \param obj cfc::Shape3D object
 * \param num Number of points to be generated on boundary
 * \return cfc::ParametricPoints structure */
template <typename G>
cfc::ParametricPoints getBoundary3D(const cfc::Shape3D& obj, const int num);

/** \brief Generate surface meshes for an object, in global frame
 * \param obj Geometric object
 * \param num Number of points to be generated on boundary
 * \return EMesh structure */
template <typename G>
EMesh generateMesh(const G* object, const int num);

/** \brief Compute mesh from parametric surface points
 * \param points Point cloud
 * \return Eigen::MatrixXd structure storing mesh triangle information */
template <typename P>
Eigen::MatrixXd getMeshFaceFromSurf(P points);

#include "MeshGenerator-inl.h"
