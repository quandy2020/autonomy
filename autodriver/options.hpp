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
 * @file options.hpp
 * @brief Process CLI (CLI11): help, version, config paths, dry-run.
 *
 * Short flags follow Google CLI conventions (-h help, -n dry-run). Runtime
 * knobs (node_name, plugin_dir, pose_channel) stay in YAML / env.
 */

#pragma once

#include <string>

namespace autodriver {

/**
 * @brief Parsed process options for the autodriver binary.
 *
 * Empty optional-style strings mean “do not override YAML”.
 */
struct Options {
    /** @brief Root that contains config/ (AUTODRIVER_PATH / argv). */
    std::string config_directory;
    /** @brief Basename under config/, default autodriver_hardware.yaml. */
    std::string config_file;
    /**
     * @brief When true, load/validate config then exit without starting
     *        hardware (Google CLI: -n / --dry-run).
     */
    bool dry_run = false;
    /** @brief When true, force hotplug.udev = false after LoadConfig. */
    bool disable_udev = false;
};

/**
 * @brief Result of ParseCommandLine().
 */
enum class ParseStatus {
    kRun = 0,       ///< Continue into Run()
    kExitOk = 1,    ///< Help / version printed; exit 0
    kExitError = 2  ///< Parse failure; exit non-zero
};

/**
 * @brief Builds the colored version banner for --version and logs.
 * @return String such as "Autodriver 0.1.0 — …".
 */
std::string VersionString();

/**
 * @brief Parse argv with CLI11.
 * @param[in] argc Argument count.
 * @param[in] argv Argument vector (argv[0] is the program name).
 * @param[out] out Filled when the return value is ParseStatus::kRun.
 * @return ParseStatus::kRun to continue, kExitOk after help/version,
 *         or kExitError on parse failure.
 */
ParseStatus ParseCommandLine(int argc, char** argv, Options* out);

}  // namespace autodriver
