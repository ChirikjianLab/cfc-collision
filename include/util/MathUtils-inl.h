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

#include "MathUtils.h"

#include <iostream>

using namespace std;

template <typename T>
T sgn(const T val) {
    return T((T(0.0) < val) - (val < T(0.0)));
}

template <typename T>
T eps_func(const T val, const double epsilon) {
    return sgn(val) * pow(abs(val), epsilon);
}

template <typename T>
Eigen::Transform<T, 3, 2> updatePoseAtTime(
    const T t, const Eigen::Transform<T, 3, 2>* g_start,
    const Eigen::Transform<T, 3, 2>* g_goal,
    const cfc::CCD_MOTION_TYPE& motion_type) {
    Eigen::Transform<T, 3, 2> g_updated;
    g_updated.setIdentity();

    // Update orientation based on motion type
    if (motion_type == cfc::TRAN) {
        g_updated.linear() = g_start->linear();
    } else if (motion_type == cfc::PCG_LINEAR) {
        // Use spherical linear interpolation (Slerp) for interpolating
        // orientations
        const Eigen::Quaternion<T> q_start(g_start->linear());
        const Eigen::Quaternion<T> q_goal(g_goal->linear());
        const Eigen::Quaternion<T> q_interp = q_start.slerp(t, q_goal);

        g_updated.linear() = q_interp.toRotationMatrix();
    }

    // Update translation
    g_updated.translation() =
        g_start->translation() +
        t * (g_goal->translation() - g_start->translation());

    return g_updated;
}
