/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace display {
namespace proto_wire {

struct ParsedJointState {
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

// Extracts std_msgs/String.data or returns raw XML/text payloads as-is.
bool UnwrapStdStringPayload(const std::string& payload, std::string* out);

// Parses sensor_msgs/JointState name and position fields from protobuf wire data.
bool ParseJointStatePayload(const std::string& payload, ParsedJointState* out);

}  // namespace proto_wire
}  // namespace display
}  // namespace autoviz
