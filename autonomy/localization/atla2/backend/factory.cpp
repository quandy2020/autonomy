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

#include "autonomy/localization/atla2/backend/optimizer_base.hpp"

#include "autonomy/localization/atla2/backend/graph/graph_optimizer.hpp"
#include "autonomy/localization/atla2/backend/iekf/iekf_optimizer.hpp"

namespace autonomy::localization::atla2 {

std::unique_ptr<OptimizerBase> CreateOptimizer(const Atla2Config& cfg) {
  std::unique_ptr<OptimizerBase> opt;
  if (cfg.backend == BackendType::kGraph || cfg.backend == BackendType::kCeres) {
    opt = std::make_unique<GraphOptimizer>();
  } else {
    opt = std::make_unique<IekfOptimizer>();
  }
  if (opt && !opt->Init(cfg)) {
    return nullptr;
  }
  return opt;
}

}  // namespace autonomy::localization::atla2
