/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "argus/fusion/anisotropic_covariance.hpp"

namespace argus {
namespace fusion {

void AnisotropicCovariance::Inflate(Eigen::MatrixXd* P,
                                    const Eigen::Vector3d& weak_direction,
                                    double alpha) {
    if (P == nullptr) {
        return;
    }
    InflatePositionBlock(*P, weak_direction, alpha);
}

}  // namespace fusion
}  // namespace argus
