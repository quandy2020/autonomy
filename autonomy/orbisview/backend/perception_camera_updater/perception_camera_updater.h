/*
 * Copyright 2026 The Openbot Authors
 *
 * PerceptionCameraUpdater — tracks latest Image / Depth envelopes.
 */

#pragma once

#include <mutex>
#include <string>

#include "autonomy/orbisview/backend/common/stream_envelope.h"

namespace autonomy {
namespace orbisview {
namespace backend {

class PerceptionCameraUpdater {
 public:
  void Ingest(const core::StreamEnvelope& env);
  std::string Channel() const;
  std::string StatusJson() const;

 private:
  mutable std::mutex mutex_;
  std::string channel_;
  std::string schema_;
  size_t bytes_{0};
  uint64_t frames_{0};
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
