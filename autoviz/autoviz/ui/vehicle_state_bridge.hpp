/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

#include <string>

#include "autoviz/qml/vehicle_state.hpp"

namespace autoviz {
namespace transform {
class Buffer;
}

/** Pull pose from TF into VehicleState for Qt Quick 3D preview. */
class VehicleStateBridge {
 public:
  void setSourceFrame(const std::string& frame) { source_frame_ = frame; }
  const std::string& sourceFrame() const { return source_frame_; }

  void updateFromTf(const std::string& fixed_frame, transform::Buffer* tf_buffer,
                    qml_vehicle::VehicleState* state);

  void resetOrigin();

 private:
  static double sceneScaleForModel(qml_vehicle::VehicleModelType type);

  std::string source_frame_ = "base_link";
  bool have_origin_ = false;
  QVector3D origin_;

  bool have_motion_sample_ = false;
  QVector3D last_relative_;
  double last_yaw_deg_ = 0.0;
  double last_sample_time_s_ = 0.0;
};

}  // namespace autoviz
