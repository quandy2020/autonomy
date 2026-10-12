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

#pragma once

/**
 * @file mpcnet_controller_base.hpp
 * @brief The base class for all controllers that use a MPC-Net policy.
 */


#include <bolza/core/control/controller_base.hpp>

namespace bolza {
namespace mpcnet {

/**
 * The base class for all controllers that use a MPC-Net policy.
 */
class MPCNetControllerBase : public ControllerBase {
 public:
  /**
   * @brief Constructs a MPCNetControllerBase.
   */
  MPCNetControllerBase() = default;
  /**
   * @brief Destructor.
   */
  ~MPCNetControllerBase() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  MPCNetControllerBase* Clone() const override = 0;

  /**
   * Load the model of the policy.
   * @param [in] policyFilePath: Path to the file with the model of the policy.
   */
  virtual void LoadPolicyModel(const std::string& policyFilePath) = 0;

 protected:
  /**
   * @brief Constructs a MPCNetControllerBase.
   * @param [in] other: Other.
   */
  MPCNetControllerBase(const MPCNetControllerBase& other) = default;
};

}  // namespace mpcnet
}  // namespace bolza
