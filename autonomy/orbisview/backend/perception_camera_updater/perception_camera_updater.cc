/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/perception_camera_updater/perception_camera_updater.h"

#include <sstream>

#include "autonomy/orbisview/backend/common/render_schemas.h"

namespace autonomy {
namespace orbisview {
namespace backend {

void PerceptionCameraUpdater::Ingest(const core::StreamEnvelope& env) {
  if (env.schema != rendering::kSchemaImage &&
      env.schema != rendering::kSchemaDepthImage) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  channel_ = env.channel;
  schema_ = env.schema;
  bytes_ = env.payload.size();
  ++frames_;
}

std::string PerceptionCameraUpdater::Channel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return channel_;
}

std::string PerceptionCameraUpdater::StatusJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"channel\":\"" << channel_ << "\",\"schema\":\"" << schema_
      << "\",\"bytes\":" << bytes_ << ",\"frames\":" << frames_ << "}";
  return oss.str();
}

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
