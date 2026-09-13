/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/point_cloud/point_cloud_updater.h"

#include <sstream>

#include "autonomy/orbisview/backend/common/render_schemas.h"

namespace autonomy {
namespace orbisview {
namespace backend {

void PointCloudUpdater::Ingest(const core::StreamEnvelope& env) {
  if (env.schema != rendering::kSchemaPointCloud2) return;
  std::lock_guard<std::mutex> lock(mutex_);
  channel_ = env.channel;
  bytes_ = env.payload.size();
  ++frames_;
}

std::string PointCloudUpdater::Channel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return channel_;
}

std::string PointCloudUpdater::StatusJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"channel\":\"" << channel_ << "\",\"bytes\":" << bytes_
      << ",\"frames\":" << frames_ << "}";
  return oss.str();
}

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
