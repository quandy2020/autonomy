/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QQuaternion>
#include <QVector3D>

namespace autoviz {
namespace transform {
class Buffer;
}

namespace common {

/** rviz_common::FrameManager — fixed frame + TF lookup facade. */
class FrameManager {
 public:
  enum class SyncMode {
    kOff = 0,
    kExact = 1,
    kApprox = 2,
  };

  void setBuffer(transform::Buffer* buffer) { buffer_ = buffer; }
  void setIdentityMode(bool identity) { identity_mode_ = identity; }
  bool identityMode() const { return identity_mode_; }

  void setFixedFrame(const std::string& frame);
  const std::string& fixedFrame() const { return fixed_frame_; }

  void setPause(bool pause) { paused_ = pause; }
  bool paused() const { return paused_; }

  void setSyncMode(SyncMode mode) { sync_mode_ = mode; }
  SyncMode syncMode() const { return sync_mode_; }

  void syncTime(double sec);
  double currentTimeSec() const;

  bool getTransform(const std::string& frame, QVector3D* position,
                    QQuaternion* orientation) const;

  bool transformHasProblems(const std::string& frame,
                              std::string* error) const;
  bool frameHasProblems(const std::string& frame, std::string* error) const;

  std::vector<std::string> allFrameNames() const;

  void update();

 private:
  transform::Buffer* buffer_ = nullptr;
  bool identity_mode_ = false;
  std::string fixed_frame_ = "map";
  bool paused_ = false;
  SyncMode sync_mode_ = SyncMode::kOff;
  double synced_time_sec_ = 0.0;
  double wall_origin_sec_ = 0.0;
};

}  // namespace common
}  // namespace autoviz
