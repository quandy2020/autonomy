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

#pragma once

/**
 * @file hash_chain.hpp
 * @brief L4 SHA-256 hash chain over SafetyEvent records (latch-only, no actuator path).
 */

#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief SHA-256 hex digest of the input bytes.
 * @param [in] data: Bytes to hash.
 * @return Lowercase hex string (64 chars).
 */
std::string Sha256(const std::string& data);

/**
 * @brief Append-only hash chain of SafetyEvent. prev_hash links consecutive records.
 *
 * Latch-only: never publishes /cmd_vel. Persist with Save/Load for process restart.
 */
class SafetyHashChain {
public:
    /**
     * @brief Empty robot_id chain.
     */
    SafetyHashChain() : SafetyHashChain(std::string{}) {}

    /**
     * @brief Chain tagged with a robot id (hashed into the first link).
     * @param [in] robot_id: Robot identifier.
     */
    explicit SafetyHashChain(std::string robot_id);

    /**
     * @brief Append an event, filling prev_hash and this hash.
     * @param [in] event: Event body (kind, detail, clock).
     * @return Stored event with hashes filled.
     */
    proto::SafetyEvent Append(const proto::SafetyEvent& event);

    /**
     * @brief Verify that consecutive prev_hash links match.
     * @param [in] events: Chain to check.
     * @return True if the chain is intact.
     */
    bool Verify(const std::vector<proto::SafetyEvent>& events) const;

    /**
     * @brief Hash of the last event (seal for sidecar).
     * @return Hex digest; empty if the chain is empty.
     */
    std::string Seal() const;

    /**
     * @brief Number of events in the chain.
     * @return Count.
     */
    size_t size() const { return events_.size(); }

    /**
     * @brief Stored events in append order.
     * @return Event list.
     */
    const std::vector<proto::SafetyEvent>& events() const { return events_; }

    /**
     * @brief Serialize the chain to a protobuf file.
     * @param [in] path: Output path.
     * @return True on success.
     */
    bool Save(const std::string& path) const;

    /**
     * @brief Load a chain from a protobuf file (replaces in-memory state).
     * @param [in] path: Input path.
     * @return True on success.
     */
    bool Load(const std::string& path);

private:
    std::string robot_id_;
    std::string prev_hash_;               ///< Hash of the last appended event.
    std::vector<proto::SafetyEvent> events_;
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
