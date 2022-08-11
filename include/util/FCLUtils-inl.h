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

#include "FCLUtils.h"

using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

template <typename G>
fcl::Transform3d getTransform(const G* object) {
    // setup transformations
    fcl::Matrix3d R = object->quaternion().toRotationMatrix();
    fcl::Vector3d T(object->pos()[0], object->pos()[1], object->pos()[2]);

    fcl::Transform3d g = fcl::Transform3d::Identity();
    g.translation() = T;
    g.linear() = R;

    return g;
}

template <typename G, typename BV>
fcl::CollisionObject<double> setCollisionObject(const G* object,
                                                const int num) {
    // If object is ellipsoid, use FCL pre-configured geometric model
    if (typeid(G) == typeid(cfc::Ellipsoid)) {
        GeometryPtr_t ellipsoidObj(new fcl::Ellipsoid<double>(
            object->a()[0], object->a()[1], object->a()[2]));

        return fcl::CollisionObject<double>(ellipsoidObj);
    }

    // For other geometric models, generate surface mesh
    G object_aux = *object;
    object_aux.setPosition(0, 0, 0);
    object_aux.setOrientation(0, 1, 0, 0);
    EMesh mesh = generateMesh(&object_aux, num);

    // add the mesh data into the BVHModel structure
    fcl::BVHModel<BV>* bvhObj = new fcl::BVHModel<BV>();
    bvhObj->beginModel();
    bvhObj->addSubModel(mesh.vertices, mesh.triangles);
    bvhObj->endModel();

    return fcl::CollisionObject<double>(GeometryPtr_t(bvhObj));
}

template <typename G, typename BV>
fcl::CollisionObject<double> setCollisionObject(const cfc::Shape3D& object,
                                                const int num) {
    G* objGeom = new G(object);

    return setCollisionObject<G, BV>(objGeom, num);
}
