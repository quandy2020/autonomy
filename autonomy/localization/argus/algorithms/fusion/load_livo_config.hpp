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

/**
 * @file load_livo_config.hpp
 * @brief Load LivoOdometryConfig from livo_esikf.yaml (+ nested robustness).
 */

#ifndef ARGUS_ODOMETRY_LOAD_LIVO_CONFIG_HPP_
#define ARGUS_ODOMETRY_LOAD_LIVO_CONFIG_HPP_

#include <string>

#include "argus/odometry/livo_odometry_config.hpp"

namespace argus {
namespace odometry {

/** @brief Outcome of loading a LIVO ESIKF YAML configuration. */
struct LoadLivoConfigResult {
    /** @brief True when parsing and validation succeeded. */
    bool ok = false;
    /** @brief Human-readable error when `ok` is false. */
    std::string error;
    /** @brief Populated configuration on success. */
    LivoOdometryConfig config;
    /** @brief Absolute path of the main YAML that was loaded. */
    std::string main_config_path;
};

/**
 * @brief Parse livo_esikf.yaml (or equivalent) into LivoOdometryConfig.
 *
 * Nested robustness YAML and loop_closure.yaml are resolved relative to the
 * main file directory. Validates threading.backend_threads == 1 and that
 * loop_closure.async is not false.
 *
 * @param yaml_path Path to the main LIVO YAML file.
 * @return Result with `ok`, parsed config, or `error` on failure.
 */
LoadLivoConfigResult LoadLivoOdometryConfig(const std::string& yaml_path);

}  // namespace odometry
}  // namespace argus

#endif  // ARGUS_ODOMETRY_LOAD_LIVO_CONFIG_HPP_
