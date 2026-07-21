/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Implements HubConfig Lua loader.
 */

#include "autodriver/bridge/autonomy/hub_config_loader.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "autonomy/common/config.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autodriver {
namespace bridge {
namespace {

HubConfig LoadHubConfigFromDictionary(
  autonomy::common::LuaParameterDictionary * dict)
{
  if (!dict) {
    throw std::runtime_error("AUTODRIVER lua table is missing");
  }

  HubConfig config;
  if (dict->HasKey("register_builtin_mocks")) {
    config.register_builtin_mocks = dict->GetBool("register_builtin_mocks");
  }

  if (dict->HasKey("hub")) {
    auto hub = dict->GetDictionary("hub");
    if (hub->HasKey("alignment_window_ms")) {
      config.hub_options.alignment_window = std::chrono::milliseconds(
        hub->GetInt("alignment_window_ms"));
    }
    if (hub->HasKey("publish_period_ms")) {
      config.hub_options.publish_period = std::chrono::milliseconds(
        hub->GetInt("publish_period_ms"));
    }
    if (hub->HasKey("buffer_capacity")) {
      config.hub_options.buffer_capacity =
        static_cast<std::size_t>(hub->GetInt("buffer_capacity"));
    }
  }

  if (dict->HasKey("drivers")) {
    auto drivers = dict->GetDictionary("drivers")->GetArrayValuesAsDictionaries();
    for (auto & entry_ptr : drivers) {
      auto * entry = entry_ptr.get();
      DriverConfig driver;
      driver.factory_name = entry->GetString("factory_name");
      driver.sensor_id = entry->GetString("sensor_id");
      if (entry->HasKey("params")) {
        auto params = entry->GetDictionary("params");
        for (const std::string & key : params->GetKeys()) {
          if (key == "device" || key == "interface" || key == "format" ||
            key == "serial" || key == "model" || key == "stream")
          {
            driver.params[key] = params->GetString(key);
          } else if (key == "baud" || key == "width" || key == "height" ||
            key == "fps" || key == "index")
          {
            driver.params[key] = std::to_string(params->GetInt(key));
          } else if (key.find("can_id") != std::string::npos) {
            driver.params[key] = std::to_string(params->GetInt(key));
          } else {
            driver.params[key] = std::to_string(params->GetDouble(key));
          }
        }
      }
      config.drivers.push_back(std::move(driver));
    }
  }

  return config;
}

}  // namespace

HubConfig LoadHubConfigFromDirectory(
  const std::string & configuration_directory,
  const std::string & config_basename)
{
  std::string config_root = configuration_directory;
  if (config_root.empty()) {
    config_root = ::autonomy::common::kConfigurationFilesDirectory;
  }

  const auto dirs =
    ::autonomy::common::ConfigurationSearchDirectories(config_root);
  auto resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
    *resolver, config_basename);
  auto file_resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  ::autonomy::common::LuaParameterDictionary lua(code, std::move(file_resolver));
  return LoadHubConfigFromDictionary(lua.GetDictionary("AUTODRIVER").get());
}

}  // namespace bridge
}  // namespace autodriver
