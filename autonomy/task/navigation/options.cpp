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

/**
 * @file options.cpp
 * @brief Options.
 */

#include "autonomy/task/navigation/options.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "glog/logging.h"
namespace autonomy { namespace task { namespace navigation {
proto::NavigatorOptions CreateOptions(const std::string& conf_file) {
  proto::NavigatorOptions options;
  const std::string file = conf_file.empty() ? "navigator.pb.txt" : conf_file;
  CHECK(common::LoadModuleConf("task", file, &options));
  return options;
}
}}}
