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

#include "cloud/ingestion/record_receiver/record_receiver.hpp"
#include "cloud/ingestion/record_receiver/hash_verifier.hpp"
#include "common/base/path_util.hpp"
#include "robot/data_agent/version_stamp/version_tuple.hpp"

namespace autoloop {
namespace cloud {

RecordReceiver::RecordReceiver(std::string archive_root)
    : store_(std::move(archive_root)) {}

common::Status RecordReceiver::Ingest(const std::string& episode_dir) {
    common::EpisodeSidecar sc;
    const auto side = common::JoinPath(episode_dir, "sidecar.pb");
    if (!common::ReadProtoBinary(side, &sc)) return common::Status::Error("sidecar");
    if (!sc.admitted() || !robot::IsAdmitted(sc.version())) {
        return common::Status::Error("not admitted");
    }
    const auto mcap = common::JoinPath(episode_dir, "snapshot.mcap");
    if (!VerifySidecarHash(sc, mcap)) return common::Status::Error("hash");
    return store_.Put(episode_dir, sc);
}

}  // namespace cloud
}  // namespace autoloop
