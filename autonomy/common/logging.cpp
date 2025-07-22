/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace common {

void InitializeGlog(char** argv) {
#ifndef _MSC_VER  // Broken in MSVC
  google::InstallFailureSignalHandler();
#endif
  google::InitGoogleLogging(argv[0]);
}

const char* __GetConstFileBaseName(const char* file) {
  const char* base = strrchr(file, '/');
  if (!base) {
    base = strrchr(file, '\\');
  }
  return base ? (base + 1) : file;
}

bool __CheckOptionImpl(const char* file,
                       const int line,
                       const bool result,
                       const char* expr_str) {
  if (result) {
    return true;
  } else {
    LOG(ERROR) << StringPrintf("[%s:%d] Check failed: %s",
                               __GetConstFileBaseName(file),
                               line,
                               expr_str);
    return false;
  }
}

}  // namespace common
}  // namespace autonomy
