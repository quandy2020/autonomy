/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <boost/math/special_functions.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/grpah/signature_3d.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

HomotopySignature3d::HomotopySignature3d(const TimedElasticBandConfig& cfg)
    : config_(&cfg) {}

bool HomotopySignature3d::IsEqual(const Equivalence& other) const {
    const HomotopySignature3d* hother =
        dynamic_cast<const HomotopySignature3d*>(&other);
    if (hother) {
        if (hsignature3d_.size() == hother->hsignature3d_.size()) {
            for (size_t i = 0; i < hsignature3d_.size(); ++i) {
                if (std::abs(hother->hsignature3d_.at(i)) <
                        config_->homotopy.homotopy_signature_threshold ||
                    std::abs(hsignature3d_.at(i)) <
                        config_->homotopy.homotopy_signature_threshold) {
                    continue;
                }

                if (boost::math::sign(hother->hsignature3d_.at(i)) !=
                    boost::math::sign(hsignature3d_.at(i))) {
                    return false;
                }
            }
            return true;
        }
    } else {
        AERROR << "Cannot compare HomotopySignature3d with other Equivalence "
                  "types other than HomotopySignature3d.";
    }

    return false;
}

bool HomotopySignature3d::IsValid() const {
    for (const double& value : hsignature3d_) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

bool HomotopySignature3d::IsReasonable() const {
    for (const double& value : hsignature3d_) {
        if (value > 1.0) {
            return false;
        }
    }
    return true;
}

const std::vector<double>& HomotopySignature3d::values() const {
    return hsignature3d_;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
