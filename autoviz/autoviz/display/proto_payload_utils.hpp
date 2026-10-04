/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file proto_payload_utils.hpp
 * @brief Lightweight protobuf-wire helpers for robot-description / JointState.
 *
 * Used by @ref RobotModelDisplay to ingest:
 * - @c std_msgs/String (or raw XML) robot description payloads
 * - @c sensor_msgs/JointState name / position / velocity / effort fields
 *
 * Lives in namespace @c proto_wire to separate wire-format concerns from
 * display drawing code.
 *
 * @see RobotModelDisplay
 * @see UrdfModel
 * @see ParsedJointState
 */

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace display {
namespace proto_wire {

/**
 * @struct ParsedJointState
 * @brief Parallel JointState arrays decoded from protobuf wire data.
 *
 * Vectors may differ in length when the publisher omits optional fields;
 * consumers should index carefully (typically by @c names).
 */
struct ParsedJointState {
  std::vector<std::string> names;      /**< Joint names. */
  std::vector<double> positions;       /**< Joint positions (rad / m). */
  std::vector<double> velocities;      /**< Joint velocities (optional). */
  std::vector<double> efforts;         /**< Joint efforts (optional). */
};

/**
 * @brief Extracts @c std_msgs/String.data or returns raw XML/text as-is.
 *
 * @param payload Raw channel bytes (protobuf String or plain text).
 * @param out Destination string; must be non-null.
 * @return @c true when @p out was filled with usable description text.
 */
bool UnwrapStdStringPayload(const std::string& payload, std::string* out);

/**
 * @brief Parses @c sensor_msgs/JointState fields from protobuf wire data.
 *
 * @param payload Raw JointState protobuf bytes.
 * @param out Destination struct; must be non-null.
 * @return @c true on successful parse of at least the name list.
 *
 * @see ParsedJointState
 */
bool ParseJointStatePayload(const std::string& payload, ParsedJointState* out);

}  // namespace proto_wire
}  // namespace display
}  // namespace autoviz
