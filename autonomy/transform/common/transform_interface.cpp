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

#include "autonomy/transform/common/transform_interface.hpp"

#include <fstream>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace transform {
namespace common {

proto::TransformOptions LoadOptions(::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
  proto::TransformOptions options;

  if (!parameter_dictionary) {
    AWARN << "LoadOptions: parameter_dictionary is null";
    return options;
  }

  // Load extrinsic_file path from Lua config
  if (parameter_dictionary->HasKey("extrinsic_file")) {
    options.set_extrinsic_file(parameter_dictionary->GetString("extrinsic_file"));
  }

  return options;
}

proto::TransformOptions CreateOptions(const std::string& configuration_directory,
                                      const std::string& configuration_basename) {
  proto::TransformOptions options;

  // Construct full path to the configuration file
  std::string config_file = configuration_directory;
  if (!config_file.empty() && config_file.back() != '/') {
    config_file += '/';
  }
  config_file += configuration_basename;

  // Check if file exists
  std::ifstream file(config_file);
  if (!file.good()) {
    AERROR << "CreateOptions: Configuration file not found: " << config_file;
    return options;
  }
  file.close();

  // Set extrinsic_file to the full path of the yaml file
  options.set_extrinsic_file(config_file);

  AINFO << "CreateOptions: Set extrinsic_file to: " << config_file;

  return options;
}

}  // namespace common
}  // namespace transform
}  // namespace autonomy
