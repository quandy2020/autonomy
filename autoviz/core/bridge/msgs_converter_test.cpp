/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <iostream>
#include <string>
#include <vector>

#include "autoviz/core/bridge/msgs_converter.hpp"

namespace {

struct Case {
  std::string input;
  std::string expected;
};

}  // namespace

int main() {
  const std::vector<Case> cases = {
      {"autonomy.commsgs.proto.sensor_msgs.Image", "sensor_msgs/msg/Image"},
      {"autonomy.commsgs.proto.sensor_msgs.PointCloud2", "sensor_msgs/msg/PointCloud2"},
      {"autonomy.commsgs.proto.sensor_msgs.JointState", "sensor_msgs/msg/JointState"},
      {"autonomy.commsgs.proto.geometry_msgs.PoseStamped", "geometry_msgs/msg/PoseStamped"},
      {"autonomy.commsgs.proto.geometry_msgs.TwistWithCovarianceStamped",
       "geometry_msgs/msg/TwistWithCovarianceStamped"},
      {"autonomy.commsgs.proto.map_msgs.OccupancyGrid", "nav_msgs/msg/OccupancyGrid"},
      {"autonomy.commsgs.proto.planning_msgs.Path", "nav_msgs/msg/Path"},
      {"autonomy.commsgs.proto.visualization_msgs.MarkerArray", "visualization_msgs/msg/MarkerArray"},
      {"autonomy.commsgs.proto.diagnostic_msgs.DiagnosticArray", "diagnostic_msgs/msg/DiagnosticArray"},
      {"autonomy.commsgs.proto.std_msgs.String", "std_msgs/msg/String"},
      {"autonomy.commsgs.proto.unknown_msgs.Custom", "autonomy.commsgs.proto.unknown_msgs.Custom"},
      {"", "unknown"},
  };

  int failed = 0;
  for (const auto& c : cases) {
    const std::string got = autoviz::MapSchemaNameToRos2Style(c.input);
    if (got != c.expected) {
      ++failed;
      std::cerr << "Case failed: input='" << c.input << "', expected='" << c.expected << "', got='" << got << "'\n";
    }
  }

  if (failed > 0) {
    std::cerr << "msgs_converter_test failed: " << failed << " case(s)\n";
    return 1;
  }
  std::cout << "msgs_converter_test passed (" << cases.size() << " cases)\n";
  return 0;
}

