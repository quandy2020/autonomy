/*
 * Copyright 2026 The Openbot Authors
 *
 * MapService — caches latest map envelopes (Dreamview map_service counterpart).
 */

#pragma once

#include <mutex>
#include <string>

#include "autonomy/orbisview/backend/common/stream_envelope.h"

namespace autonomy {
namespace orbisview {
namespace backend {

class MapService {
 public:
  void Ingest(const core::StreamEnvelope& env);
  std::string OccupancyChannel() const;
  std::string VectorMapChannel() const;
  std::string StatusJson() const;

 private:
  mutable std::mutex mutex_;
  std::string occupancy_channel_;
  std::string vector_map_channel_;
  size_t occupancy_bytes_{0};
  size_t vector_map_bytes_{0};
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
