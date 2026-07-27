/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/view_state_io.hpp"

namespace autoviz {
namespace common {

SavedViewConfig ToSavedViewConfig(const std::string& name,
                                  const rendering::ViewState& state) {
  SavedViewConfig config;
  config.name = name;
  switch (state.type) {
    case rendering::ViewControllerType::kTopDown:
      config.type = "TopDown";
      break;
    case rendering::ViewControllerType::kXyOrbit:
      config.type = "XYOrbit";
      break;
    case rendering::ViewControllerType::kTopDownOrtho:
      config.type = "TopDownOrtho";
      break;
    case rendering::ViewControllerType::kFps:
      config.type = "FPS";
      break;
    case rendering::ViewControllerType::kFpsMotion:
      config.type = "FPSMotion";
      break;
    case rendering::ViewControllerType::kThirdPersonFollow:
      config.type = "ThirdPersonFollow";
      break;
    case rendering::ViewControllerType::kOrbit:
    default:
      config.type = "Orbit";
      break;
  }
  config.yaw = state.yaw;
  config.pitch = state.pitch;
  config.distance = state.distance;
  config.near_clip_distance = state.near_clip_distance;
  config.invert_z_axis = state.invert_z_axis;
  config.focal_shape_size = state.focal_shape_size;
  config.focal_shape_fixed_size = state.focal_shape_fixed_size;
  config.target_x = state.target.x();
  config.target_y = state.target.y();
  config.target_z = state.target.z();
  config.fps_position_x = state.fps_position.x();
  config.fps_position_y = state.fps_position.y();
  config.fps_position_z = state.fps_position.z();
  config.fps_yaw = state.fps_yaw;
  config.fps_pitch = state.fps_pitch;
  config.target_frame = state.target_frame;
  return config;
}

rendering::ViewState ToViewState(const SavedViewConfig& config) {
  rendering::ViewState state;
  if (config.type == "TopDown") {
    state.type = rendering::ViewControllerType::kTopDown;
  } else if (config.type == "XYOrbit") {
    state.type = rendering::ViewControllerType::kXyOrbit;
  } else if (config.type == "TopDownOrtho") {
    state.type = rendering::ViewControllerType::kTopDownOrtho;
  } else if (config.type == "FPS") {
    state.type = rendering::ViewControllerType::kFps;
  } else if (config.type == "FPSMotion") {
    state.type = rendering::ViewControllerType::kFpsMotion;
  } else if (config.type == "ThirdPersonFollow") {
    state.type = rendering::ViewControllerType::kThirdPersonFollow;
  } else {
    state.type = rendering::ViewControllerType::kOrbit;
  }
  state.yaw = config.yaw;
  state.pitch = config.pitch;
  state.distance = config.distance;
  state.near_clip_distance = config.near_clip_distance;
  state.invert_z_axis = config.invert_z_axis;
  state.focal_shape_size = config.focal_shape_size;
  state.focal_shape_fixed_size = config.focal_shape_fixed_size;
  state.target = QVector3D(config.target_x, config.target_y, config.target_z);
  state.fps_position =
      QVector3D(config.fps_position_x, config.fps_position_y,
                config.fps_position_z);
  state.fps_yaw = config.fps_yaw;
  state.fps_pitch = config.fps_pitch;
  state.target_frame = config.target_frame;
  return state;
}

}  // namespace common
}  // namespace autoviz
