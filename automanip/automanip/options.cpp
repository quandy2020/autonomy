/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @brief CLI parsing for the automanip process.
 */

#include "options.hpp"

#include <iostream>
#include <string>

#include <CLI/CLI.hpp>

#include "automanip/conf/conf.hpp"

namespace automanip {

std::string VersionString() {
  return std::string(conf::kPackageName) + " " + conf::kFullVersion + " — " +
         conf::kPackageDescription;
}

ParseStatus ParseCommandLine(int argc, char** argv, Options* out) {
  if (out == nullptr) {
    return ParseStatus::kExitError;
  }
  CLI::App app{"Manipulator HAL and OCS2 kinematic arm controller"};
  app.set_version_flag("-V,--version", VersionString());
  app.add_option("config_directory", out->config_directory,
                 "Package root that contains config/");
  app.add_option("config_file", out->config_file,
                 "YAML basename under config/ (default automanip.yaml)");
  app.add_flag("-n,--dry-run", out->dry_run,
               "Load config and exit without starting the arm");
  try {
    app.parse(argc, argv);
  } catch (const CLI::CallForHelp&) {
    std::cout << app.help() << std::endl;
    return ParseStatus::kExitOk;
  } catch (const CLI::CallForVersion&) {
    std::cout << VersionString() << std::endl;
    return ParseStatus::kExitOk;
  } catch (const CLI::ParseError& error) {
    std::cerr << error.what() << std::endl;
    return ParseStatus::kExitError;
  }
  return ParseStatus::kRun;
}

}  // namespace automanip
