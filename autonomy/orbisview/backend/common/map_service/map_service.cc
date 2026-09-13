/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/map_service/map_service.h"

#include <sstream>

#include "autonomy/orbisview/backend/common/render_schemas.h"

namespace autonomy {
namespace orbisview {
namespace backend {

void MapService::Ingest(const core::StreamEnvelope& env) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (env.schema == rendering::kSchemaOccupancyGrid) {
    occupancy_channel_ = env.channel;
    occupancy_bytes_ = env.payload.size();
  } else if (env.schema == rendering::kSchemaVectorMap) {
    vector_map_channel_ = env.channel;
    vector_map_bytes_ = env.payload.size();
  }
}

std::string MapService::OccupancyChannel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return occupancy_channel_;
}

std::string MapService::VectorMapChannel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return vector_map_channel_;
}

std::string MapService::StatusJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"occupancy_channel\":\"" << occupancy_channel_
      << "\",\"occupancy_bytes\":" << occupancy_bytes_
      << ",\"vector_map_channel\":\"" << vector_map_channel_
      << "\",\"vector_map_bytes\":" << vector_map_bytes_ << "}";
  return oss.str();
}

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
