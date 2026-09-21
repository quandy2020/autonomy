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

#include "autonomy/localization/atla2/frontend/factory/frontend_factory.hpp"

#include "autonomy/localization/atla2/frontend/lio/lio_frontend.hpp"
#include "autonomy/localization/atla2/frontend/livo/livo_frontend.hpp"
#include "autonomy/localization/atla2/frontend/lo/lo_frontend.hpp"
#include "autonomy/localization/atla2/frontend/vio/vio_frontend.hpp"
#include "autonomy/localization/atla2/frontend/vo/vo_frontend.hpp"

namespace autonomy::localization::atla2 {

std::unique_ptr<FrontendBase> CreateFrontend(const Atla2Config& cfg) {
  std::unique_ptr<FrontendBase> fe;
  switch (cfg.mode) {
    case FrontendMode::kVo:
      fe = std::make_unique<VoFrontend>();
      break;
    case FrontendMode::kLo:
      fe = std::make_unique<LoFrontend>();
      break;
    case FrontendMode::kLio:
      fe = std::make_unique<LioFrontend>();
      break;
    case FrontendMode::kLivo:
      fe = std::make_unique<LivoFrontend>();
      break;
    case FrontendMode::kVio:
    default:
      fe = std::make_unique<VioFrontend>();
      break;
  }
  if (fe && !fe->Init(cfg)) {
    return nullptr;
  }
  return fe;
}

}  // namespace autonomy::localization::atla2
