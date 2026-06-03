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

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/grpah/signature_2d.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

HomotopySignature::HomotopySignature(const TimedElasticBandConfig& cfg)
    : config_(&cfg) {}

bool HomotopySignature::IsEqual(const Equivalence& other) const {
    const HomotopySignature* hother =
        dynamic_cast<const HomotopySignature*>(&other);
    if (hother) {
        double diff_real =
            std::abs(hother->hsignature_.real() - hsignature_.real());
        double diff_imag =
            std::abs(hother->hsignature_.imag() - hsignature_.imag());
        if (diff_real <= config_->homotopy.homotopy_signature_threshold &&
            diff_imag <= config_->homotopy.homotopy_signature_threshold) {
            return true;
        }
    } else {
        AERROR << "Cannot compare HomotopySignature with other Equivalence types "
                  "other than HomotopySignature.";
    }

    return false;
}

bool HomotopySignature::IsValid() const {
    return std::isfinite(hsignature_.real()) &&
           std::isfinite(hsignature_.imag());
}

bool HomotopySignature::IsReasonable() const {
    return true;
}

const std::complex<long double>& HomotopySignature::value() const {
    return hsignature_;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
