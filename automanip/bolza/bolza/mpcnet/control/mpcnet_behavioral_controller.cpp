/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bolza/mpcnet/control/mpcnet_behavioral_controller.hpp"

#include <bolza/core/misc/numerics.hpp>

namespace bolza {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MPCNetBehavioralController::ComputeInput(scalar_t t, const vector_t& x) {
  if (optimalControllerPtr_ != nullptr && learnedControllerPtr_ != nullptr) {
    if (numerics::almost_eq(alpha_, 0.0)) {
      return learnedControllerPtr_->ComputeInput(t, x);
    } else if (numerics::almost_eq(alpha_, 1.0)) {
      return optimalControllerPtr_->ComputeInput(t, x);
    } else {
      return alpha_ * optimalControllerPtr_->ComputeInput(t, x) + (1 - alpha_) * learnedControllerPtr_->ComputeInput(t, x);
    }
  } else {
    throw std::runtime_error(
        "[MPCNetBehavioralController::ComputeInput] cannot compute input, since optimal and/or learned controller not set.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int MPCNetBehavioralController::size() const {
  if (optimalControllerPtr_ != nullptr && learnedControllerPtr_ != nullptr) {
    return std::max(optimalControllerPtr_->size(), learnedControllerPtr_->size());
  } else if (optimalControllerPtr_ != nullptr) {
    return optimalControllerPtr_->size();
  } else if (learnedControllerPtr_ != nullptr) {
    return learnedControllerPtr_->size();
  } else {
    return 0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetBehavioralController::clear() {
  if (optimalControllerPtr_ != nullptr) {
    optimalControllerPtr_->clear();
  }
  if (learnedControllerPtr_ != nullptr) {
    learnedControllerPtr_->clear();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPCNetBehavioralController::empty() const {
  if (optimalControllerPtr_ != nullptr && learnedControllerPtr_ != nullptr) {
    return optimalControllerPtr_->empty() && learnedControllerPtr_->empty();
  } else if (optimalControllerPtr_ != nullptr) {
    return optimalControllerPtr_->empty();
  } else if (learnedControllerPtr_ != nullptr) {
    return learnedControllerPtr_->empty();
  } else {
    return true;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetBehavioralController::Concatenate(const ControllerBase* otherController, int index, int length) {
  if (optimalControllerPtr_ != nullptr) {
    optimalControllerPtr_->Concatenate(otherController, index, length);
  }
  if (learnedControllerPtr_ != nullptr) {
    learnedControllerPtr_->Concatenate(otherController, index, length);
  }
}

}  // namespace mpcnet
}  // namespace bolza
