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

#include "MeshGenerator.h"

#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

template <typename P>
EMesh getMesh(P points) {
    EMesh mesh;

    // Convert vertices to list of fcl::Vector3
    for (size_t i = 0; i < points.x.size(); ++i) {
        fcl::Vector3<double> v;
        v << points.x.at(i), points.y.at(i), points.z.at(i);
        mesh.vertices.push_back(v);
    }

    // Compute mesh faces from parametric surface
    Eigen::MatrixXd mesh_surf = getMeshFaceFromSurf(points);

    // Convert mesh faces to list of fcl::Triangles
    for (size_t i = 0; i < mesh_surf.rows(); ++i) {
        fcl::Triangle triangle(mesh_surf(i, 0), mesh_surf(i, 1),
                               mesh_surf(i, 2));
        mesh.triangles.push_back(triangle);
    }

    return mesh;
}

template <typename G>
cfc::ParametricPoints getBoundary3D(const G* obj, const int num) {
    double eta;
    double omega;
    cfc::ParametricPoints X;
    Eigen::Vector3d x;

    for (int i = 0; i < num; ++i) {
        eta = -pi / 2.0 + i * pi / (num - 1);

        for (int j = 0; j < num; ++j) {
            omega = -pi + j * (2.0 * pi) / (num - 1);

            x = obj->getBoundaryFromSpherical(Eigen::Vector2d(eta, omega));

            // Transform object to world frame
            const Eigen::Quaterniond quat = obj->quaternion();
            const Eigen::Vector3d center(
                {obj->pos()[0], obj->pos()[1], obj->pos()[2]});
            x = quat.matrix() * x + center;

            X.x.push_back(x(0));
            X.y.push_back(x(1));
            X.z.push_back(x(2));
        }
    }
    return X;
}

template <typename G>
cfc::ParametricPoints getBoundary3D(const cfc::Shape3D& obj, const int num) {
    G* objGeom = new G(obj);

    return getBoundary3D<G>(objGeom, num);
}

template <typename G>
EMesh generateMesh(const G* object, const int num) {
    EMesh mesh;

    cfc::ParametricPoints pts = getBoundary3D(object, num);
    mesh = getMesh(pts);

    return mesh;
}

template <typename P>
Eigen::MatrixXd getMeshFaceFromSurf(P points) {
    auto n = static_cast<int>(std::sqrt(points.x.size()));
    auto Num = (n - 1) * (n - 1);

    Eigen::ArrayXd q((n - 1) * (n - 1));
    for (auto i = 0; i < n - 1; ++i) {
        q.segment(i * (n - 1), (n - 1)) =
            Eigen::ArrayXd::LinSpaced(n - 1, i * n, (i + 1) * n - 2);
    }

    Eigen::MatrixXd faces = Eigen::MatrixXd::Zero(2 * Num, 3);
    faces.block(0, 0, Num, 1) = q + n + 1;
    faces.block(0, 1, Num, 1) = q + n;
    faces.block(0, 2, Num, 1) = q;
    faces.block(Num, 0, Num, 1) = q;
    faces.block(Num, 1, Num, 1) = q + 1;
    faces.block(Num, 2, Num, 1) = q + n + 1;

    return faces;
}
