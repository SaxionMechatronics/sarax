// BSD 3-Clause License
// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef MATHHELPER_H
#define MATHHELPER_H

#include <Eigen/Dense>
#include <Eigen/Geometry> 

namespace MathHelper {

// ----- Skew ----- //
Eigen::Matrix3d skew(Eigen::Vector3d a);
// for twists:
Eigen::Matrix<double, 4, 4> skewT(Eigen::Matrix<double, 6, 1> a);

// ----- Unskew ----- //
Eigen::Vector3d unskew(Eigen::Matrix3d a_tilde);
// for twists:
Eigen::Matrix<double, 6, 1> unskewT(Eigen::Matrix<double, 4, 4> a_tilde);

// ----- Antisymmetric ----- //
Eigen::Matrix3d antisym(Eigen::Matrix3d a);

// ----- Lie Group Adjoint ----- //
Eigen::Matrix<double, 6, 6> Adjoint(Eigen::Matrix<double, 4, 4> H_in);

// ----- InverseH  ----- //
Eigen::Matrix4d inverseH(Eigen::Matrix4d H_in);

// ----- Lie AlgebraAdjoint ----- //
Eigen::Matrix<double, 6, 6> adjoint(Eigen::Matrix<double, 6, 1> T_in);

// ----- sgn function for vector----- //
Eigen::Matrix<double, 6, 1> signArray(Eigen::Matrix<double, 6, 1> X_in);

Eigen::Matrix3d expMap(Eigen::Vector3d a);

Eigen::Matrix<double, 4, 4> expMapT(Eigen::Matrix<double, 6, 1> a);

Eigen::Matrix4d HmatrixComposition (const Eigen::Matrix3d& R, const Eigen::Vector3d& position);

void test();

}

#endif
