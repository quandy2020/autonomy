/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

#include <string>

namespace autoviz {
namespace transform {
class Buffer;
}
namespace qml_drone {
class DroneVehicleState;
}
}  // namespace autoviz

namespace autoviz {

/** Pull pose from TF into DroneVehicleState (fixed-frame relative preview). */
class DroneStateBridge {
 public:
  void setSourceFrame(const std::string& frame) { source_frame_ = frame; }
  const std::string& sourceFrame() const { return source_frame_; }

  void setFlightModeWhenValid(int mode) { flight_mode_when_valid_ = mode; }

  void updateFromTf(const std::string& fixed_frame, transform::Buffer* tf_buffer,
                    qml_drone::DroneVehicleState* state);

 private:
  std::string source_frame_ = "base_link";
  int flight_mode_when_valid_ = 1;
  bool have_origin_ = false;
  QVector3D origin_;
};

}  // namespace autoviz
