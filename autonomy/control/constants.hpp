/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

namespace autonomy {
namespace control {

constexpr char kControllerServerNodeName[] = "controller_server";
constexpr char kCmdVelChannel[] = "/cmd_vel";
constexpr char kLocalCostmapTopicName[] = "/local_costmap";
constexpr char kMapTopicName[] = "/map";
constexpr char kOdomTopicName[] = "/odom";
constexpr char kScanTopicName[] = "/scan";
constexpr char kImageTopicName[] = "/camera/image_raw";
constexpr char kPointCloudTopicName[] = "/points";
constexpr char kLidarBevTopicName[] = "/perception/lidar_bev";
constexpr char kRawTensorTopicName[] = "/perception/policy_raw";
constexpr char kPolicyDiagnosticsTopicName[] = "/policy_controller/diagnostics";
constexpr char kPolicyReloadConfigTopicName[] = "/policy_controller/reload_config";
// MoGe LocalGrid (grid_map 2.5D) for person-follow MPPI.
constexpr char kFollowGridTopicName[] = "/perception/follow/grid";
constexpr char kFollowPathActionName[] = "/follow_path";
constexpr char kSpinActionName[] = "/spin";
constexpr char kBackUpActionName[] = "/backup";
constexpr char kWaitActionName[] = "/wait";

}  // namespace control
}  // namespace autonomy
