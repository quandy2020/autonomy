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

#include <optional>
#include <string>

#include "autonomy/system/monitor/system_health_snapshot.hpp"
#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace system {
namespace monitor {

/// Single health truth for Bridge GetHealth (read-only consumer).
class HealthSnapshotStore {
public:
    static std::string DefaultPath();

    explicit HealthSnapshotStore(std::string path = DefaultPath());

    bool Write(const SystemHealthSnapshot& snapshot) const;
    std::optional<::automsgs::rpcs::system::SystemHealth> Read() const;

    ::automsgs::rpcs::system::SystemHealth ToProto(
        const SystemHealthSnapshot& snapshot) const;

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
