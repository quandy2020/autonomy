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

#include <gtest/gtest.h>
#include <filesystem>

#include <bolza/core/pre_computation.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>
#include <bolza/core/loopshaping/loopshaping_pre_computation.hpp>
#include <bolza/core/loopshaping/loopshaping_property_tree.hpp>

namespace bolza {

const std::vector<std::string> configNames = {"loopshaping_r.conf", "loopshaping_r_ballbot.conf", "loopshaping_s.conf",
                                              "loopshaping_s_integrator.conf"};

inline std::string GetAbsolutePathToConfigurationFile(const std::string& fileName) {
  const std::filesystem::path PathToTest(__FILE__);
  return (pathToTest.parent_path() / fileName).String();
}

class LoopshapingTestConfiguration {
 public:
  LoopshapingTestConfiguration(const std::string& configName);

 protected:
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<PreComputation> preComp_sys_;
  std::unique_ptr<LoopshapingPreComputation> preComp_;

  size_t systemStateDim_;
  size_t filterStateDim_;
  size_t inputDim_;

  const scalar_t tol = 1e-9;

  scalar_t t;
  vector_t x_;
  vector_t u_;
  vector_t x_sys_;
  vector_t u_sys_;
  vector_t x_filter_;
  vector_t u_filter_;

  vector_t x_disturbance_;
  vector_t u_disturbance_;
  vector_t x_sys_disturbance_;
  vector_t u_sys_disturbance_;
  vector_t x_filter_disturbance_;
  vector_t u_filter_disturbance_;

 private:
  void GetRandomStateInput(size_t systemStateDim, size_t filterStateDim, size_t inputDim, vector_t& x_sys, vector_t& u_sys,
                           vector_t& x_filter, vector_t& u_filter, vector_t& x, vector_t& u, scalar_t range = 1.0);
};

}  // namespace bolza
