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

#include "common/logging.hpp"

#include <atomic>
#include <mutex>
#include <string>

namespace atlas {
namespace common {
namespace {

std::mutex& InitMutex() {
  static std::mutex mu;
  return mu;
}

std::atomic_bool& InitializedFlag() {
  static std::atomic_bool flag{false};
  return flag;
}

}  // namespace

void InitLogging(const char* argv0) {
  std::lock_guard<std::mutex> lock(InitMutex());
  if (InitializedFlag().load(std::memory_order_relaxed)) {
    return;
  }
  google::InitGoogleLogging(argv0 != nullptr ? argv0 : "atlas");
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  InitializedFlag().store(true, std::memory_order_release);
}

void ShutdownLogging() {
  std::lock_guard<std::mutex> lock(InitMutex());
  if (!InitializedFlag().exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  google::ShutdownGoogleLogging();
}

bool IsLoggingInitialized() {
  return InitializedFlag().load(std::memory_order_acquire);
}

void SetMinLogSeverity(int severity) { FLAGS_minloglevel = severity; }

void SetLogDirectory(const std::string& dir) {
  if (dir.empty()) {
    FLAGS_logtostderr = 1;
    FLAGS_log_dir.clear();
    return;
  }
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;
  FLAGS_log_dir = dir;
}

}  // namespace common
}  // namespace atlas
