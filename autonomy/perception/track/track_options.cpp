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

/**
 * @file
 * @brief Default and Lua loaders for TrackOptions.
 */

#include "autonomy/perception/track/track_options.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/perception/track/common/constants.hpp"
#include "autonomy/perception/track/common/types.hpp"

namespace autonomy::perception::track {
namespace {

using ::autonomy::common::ConfigurationFileResolver;
using ::autonomy::common::LuaParameterDictionary;

void ApplyLua(LuaParameterDictionary* dict, proto::TrackOptions* opts) {
  if (dict->HasKey("enabled")) {
    opts->set_enabled(dict->GetBool("enabled"));
  }
  if (dict->HasKey("model_path")) {
    opts->set_model_path(dict->GetString("model_path"));
  }
  if (dict->HasKey("backend_id")) {
    opts->set_backend_id(dict->GetString("backend_id"));
  }
  if (dict->HasKey("odom_topic")) {
    opts->set_odom_topic(dict->GetString("odom_topic"));
  }
  if (dict->HasKey("depth_topic")) {
    opts->set_depth_topic(dict->GetString("depth_topic"));
  }
  if (dict->HasKey("camera_info_topic")) {
    opts->set_camera_info_topic(dict->GetString("camera_info_topic"));
  }
  if (dict->HasKey("camera_frame")) {
    opts->set_camera_frame(dict->GetString("camera_frame"));
  }
  if (dict->HasKey("base_frame")) {
    opts->set_base_frame(dict->GetString("base_frame"));
  }
  if (dict->HasKey("cmd_vel_topic")) {
    opts->set_cmd_vel_topic(dict->GetString("cmd_vel_topic"));
  }
  if (dict->HasKey("debug_path_topic")) {
    opts->set_debug_path_topic(dict->GetString("debug_path_topic"));
  }
  if (dict->HasKey("control_hz")) {
    opts->set_control_hz(dict->GetDouble("control_hz"));
  }
  if (dict->HasKey("image_height")) {
    opts->set_image_height(dict->GetInt("image_height"));
  }
  if (dict->HasKey("image_width")) {
    opts->set_image_width(dict->GetInt("image_width"));
  }
  if (dict->HasKey("horizon_num")) {
    opts->set_horizon_num(dict->GetInt("horizon_num"));
  }
  if (dict->HasKey("vertical_num")) {
    opts->set_vertical_num(dict->GetInt("vertical_num"));
  }
  if (dict->HasKey("horizon_camera_fov_deg")) {
    opts->set_horizon_camera_fov_deg(dict->GetDouble("horizon_camera_fov_deg"));
  }
  if (dict->HasKey("radio_range_m")) {
    opts->set_radio_range_m(dict->GetDouble("radio_range_m"));
  }
  if (dict->HasKey("vel_max_mps")) {
    opts->set_vel_max_mps(dict->GetDouble("vel_max_mps"));
  }
  if (dict->HasKey("wz_max_rps")) {
    opts->set_wz_max_rps(dict->GetDouble("wz_max_rps"));
  }
  if (dict->HasKey("objectness_threshold")) {
    opts->set_objectness_threshold(dict->GetDouble("objectness_threshold"));
  }
  if (dict->HasKey("nms_angle_deg")) {
    opts->set_nms_angle_deg(dict->GetDouble("nms_angle_deg"));
  }
  if (dict->HasKey("follow_distance_m")) {
    opts->set_follow_distance_m(dict->GetDouble("follow_distance_m"));
  }
  if (dict->HasKey("depth_scale")) {
    opts->set_depth_scale(dict->GetDouble("depth_scale"));
  }
  if (dict->HasKey("allow_heuristic_fallback")) {
    opts->set_allow_heuristic_fallback(
        dict->GetBool("allow_heuristic_fallback"));
  }
  if (dict->HasKey("trajectory_mode")) {
    opts->set_trajectory_mode(dict->GetString("trajectory_mode"));
  }
  if (dict->HasKey("minco_piece_duration_s")) {
    opts->set_minco_piece_duration_s(dict->GetDouble("minco_piece_duration_s"));
  }
  if (dict->HasKey("acc_max_mps2")) {
    opts->set_acc_max_mps2(dict->GetDouble("acc_max_mps2"));
  }
  if (dict->HasKey("minco_sample_horizon_s")) {
    opts->set_minco_sample_horizon_s(dict->GetDouble("minco_sample_horizon_s"));
  }
}

}  // namespace

proto::TrackOptions DefaultOptions() {
  proto::TrackOptions options;
  options.set_enabled(true);
  options.set_backend_id("onnx");
  options.set_odom_topic("/odom");
  options.set_depth_topic("/camera/depth/image_raw");
  options.set_camera_info_topic("/camera/depth/camera_info");
  options.set_camera_frame("camera_depth_optical_frame");
  options.set_base_frame("base_link");
  options.set_cmd_vel_topic(kTrackVelocityCommandTopic);
  options.set_debug_path_topic(kTrackDebugPathTopic);
  options.set_control_hz(defaults::kControlHz);
  options.set_image_height(defaults::kImageHeight);
  options.set_image_width(defaults::kImageWidth);
  options.set_horizon_num(defaults::kHorizontalBinCount);
  options.set_vertical_num(defaults::kVerticalBinCount);
  options.set_horizon_camera_fov_deg(
      defaults::kCameraHorizontalFieldOfViewDeg);
  options.set_radio_range_m(defaults::kPlanningHorizonM);
  options.set_vel_max_mps(defaults::kMaxLinearVelocityMps);
  options.set_wz_max_rps(defaults::kMaxYawRateRps);
  options.set_objectness_threshold(defaults::kObjectnessThreshold);
  options.set_nms_angle_deg(defaults::kSuppressionAngleDeg);
  options.set_follow_distance_m(defaults::kFollowDistanceM);
  options.set_depth_scale(defaults::kDepthScaleToMetres);
  options.set_allow_heuristic_fallback(defaults::kAllowHeuristicFallback);
  options.set_config_basename("perception/track_yopo.lua");
  options.set_trajectory_mode(defaults::kTrajectoryMode);
  options.set_minco_piece_duration_s(defaults::kMincoPieceDurationS);
  options.set_acc_max_mps2(defaults::kMaxAccelerationMps2);
  options.set_minco_sample_horizon_s(defaults::kMincoSampleHorizonS);
  return options;
}

void ApplyDefaults(proto::TrackOptions* options) {
  if (options == nullptr) {
    return;
  }
  if (options->image_height() <= 0) {
    options->set_image_height(defaults::kImageHeight);
  }
  if (options->image_width() <= 0) {
    options->set_image_width(defaults::kImageWidth);
  }
  if (options->horizon_num() <= 0) {
    options->set_horizon_num(defaults::kHorizontalBinCount);
  }
  if (options->vertical_num() <= 0) {
    options->set_vertical_num(defaults::kVerticalBinCount);
  }
  if (options->follow_distance_m() <= 0) {
    options->set_follow_distance_m(defaults::kFollowDistanceM);
  }
  if (options->objectness_threshold() <= 0) {
    options->set_objectness_threshold(defaults::kObjectnessThreshold);
  }
  if (options->nms_angle_deg() <= 0) {
    options->set_nms_angle_deg(defaults::kSuppressionAngleDeg);
  }
  if (options->vel_max_mps() <= 0) {
    options->set_vel_max_mps(defaults::kMaxLinearVelocityMps);
  }
  if (options->wz_max_rps() <= 0) {
    options->set_wz_max_rps(defaults::kMaxYawRateRps);
  }
  if (options->radio_range_m() <= 0) {
    options->set_radio_range_m(defaults::kPlanningHorizonM);
  }
  if (options->horizon_camera_fov_deg() <= 0) {
    options->set_horizon_camera_fov_deg(
        defaults::kCameraHorizontalFieldOfViewDeg);
  }
  if (options->depth_scale() <= 0) {
    options->set_depth_scale(defaults::kDepthScaleToMetres);
  }
  if (options->control_hz() <= 0) {
    options->set_control_hz(defaults::kControlHz);
  }
  if (options->cmd_vel_topic().empty()) {
    options->set_cmd_vel_topic(kTrackVelocityCommandTopic);
  }
  if (options->debug_path_topic().empty()) {
    options->set_debug_path_topic(kTrackDebugPathTopic);
  }
  if (options->base_frame().empty()) {
    options->set_base_frame("base_link");
  }
  if (options->trajectory_mode().empty()) {
    options->set_trajectory_mode(defaults::kTrajectoryMode);
  }
  if (options->minco_piece_duration_s() <= 0) {
    options->set_minco_piece_duration_s(defaults::kMincoPieceDurationS);
  }
  if (options->acc_max_mps2() <= 0) {
    options->set_acc_max_mps2(defaults::kMaxAccelerationMps2);
  }
  if (options->minco_sample_horizon_s() <= 0) {
    options->set_minco_sample_horizon_s(defaults::kMincoSampleHorizonS);
  }
}

proto::TrackOptions LoadOptions(const std::string& config_directory,
                                const std::string& relative_path) {
  proto::TrackOptions options = DefaultOptions();
  if (config_directory.empty()) {
    return options;
  }
  try {
    ConfigurationFileResolver resolver({config_directory});
    const std::string code = resolver.GetFileContentOrDie(relative_path);
    auto dict = LuaParameterDictionary::NonReferenceCounted(
        code, std::make_unique<ConfigurationFileResolver>(
                  std::vector<std::string>{config_directory}));
    ApplyLua(dict.get(), &options);
    ApplyDefaults(&options);
  } catch (const std::exception& exception) {
    AWARN << "TrackOptions: failed to load " << config_directory << "/"
          << relative_path << " (" << exception.what()
          << "); using defaults.";
  }
  return options;
}

}  // namespace autonomy::perception::track
