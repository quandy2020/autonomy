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
 * @file options.hpp
 * @brief Process CLI: help, version, config paths, dry-run.
 */

#pragma once

#include <string>

namespace automanip {

struct Options {
  std::string config_directory;
  std::string config_file;
  bool dry_run = false;
};

enum class ParseStatus { kRun = 0, kExitOk = 1, kExitError = 2 };

std::string VersionString();
ParseStatus ParseCommandLine(int argc, char** argv, Options* out);

}  // namespace automanip
