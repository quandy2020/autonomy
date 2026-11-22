/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file snapshot_writer.cpp
 * @brief Episode directory: snapshot.mcap + sidecar.pb.
 */

#include "robot/data_agent/snapshot_writer/snapshot_writer.hpp"

#include "common/base/hash_util.hpp"
#include "common/base/path_util.hpp"
#include "common/base/time_util.hpp"
#include "robot/data_agent/snapshot_writer/autolink_writer_adapter.hpp"
#include "robot/data_agent/snapshot_writer/mcap_exporter.hpp"

#include <sstream>

namespace autoloop {
namespace robot {

SnapshotWriter::SnapshotWriter(SnapshotConfig cfg) : cfg_(std::move(cfg)) {}

common::Status SnapshotWriter::Write(const std::vector<Sample>& samples,
                                     const TriggerEvent& trigger,
                                     const VersionCheck& version,
                                     std::string* episode_dir) {
    const auto now = common::NowDualClock();
    std::ostringstream id;
    id << "ep-" << now.utc_ns() << "-" << static_cast<int>(trigger.kind);
    const std::string eid = id.str();
    const std::string dir = common::JoinPath(cfg_.root, eid);
    if (!common::EnsureDirectory(dir)) {
        return common::Status::Error("mkdir " + dir);
    }
    const std::string mcap = common::JoinPath(dir, "snapshot.mcap");
    if (!ExportMcap(mcap, samples)) {
        return common::Status::Error("mcap");
    }
    std::string digest;
    common::Sha256File(mcap, &digest);
    common::EpisodeSidecar sc;
    sc.set_episode_id(eid);
    *sc.mutable_version() = version.tuple;
    if (!samples.empty()) {
        *sc.mutable_start() = samples.front().clock;
        *sc.mutable_end() = samples.back().clock;
    } else {
        *sc.mutable_start() = now;
        *sc.mutable_end() = now;
    }
    sc.set_admitted(version.admitted);
    for (const auto& m : version.missing) {
        sc.add_missing_fields(m);
    }
    sc.set_robot_id(cfg_.robot_id);
    sc.set_priority(static_cast<int32_t>(trigger.priority));
    sc.set_content_sha256(digest);
    const std::string side = common::JoinPath(dir, "sidecar.pb");
    if (!WriteSidecar(side, sc)) {
        return common::Status::Error("sidecar");
    }
    if (episode_dir != nullptr) {
        *episode_dir = dir;
    }
    ++written_;
    return common::Status::OK();
}

}  // namespace robot
}  // namespace autoloop
