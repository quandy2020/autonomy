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

#pragma once

#include <string>

namespace autonomy {
namespace system {
namespace safety {

/**
 * Process-shared emergency-stop latch (Bridge E-Stop ↔ monitor MRM / Snapshot).
 * Backed by a small file so autonomy.monitor and autonomy.bridge agree.
 */
class SafetyLatch {
public:
    /// Override via AUTONOMY_SAFETY_LATCH_PATH; default under runtime dir.
    static std::string DefaultPath();

    explicit SafetyLatch(std::string path = DefaultPath());

    bool IsLatched() const;
    /// @return true on successful write
    bool SetLatched(bool latched, const std::string& reason = {});
    std::string Reason() const;
    const std::string& path() const { return path_; }

private:
    std::string path_;
};

}  // namespace safety
}  // namespace system
}  // namespace autonomy
