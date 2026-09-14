/*
 * Copyright 2026 The Openbot Authors
 *
 * SimulationWorldUpdater — world state ingest + JSON dump.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/orbisview/backend/simulation_world/simulation_world_service.hpp"
#include "autonomy/orbisview/backend/common/stream_envelope.hpp"

namespace autonomy {
namespace orbisview {
namespace backend {

class SimulationWorldUpdater {
 public:
  SimulationWorldUpdater();

  core::SimulationWorldService* Service() { return service_.get(); }
  const core::SimulationWorldService* Service() const { return service_.get(); }

  void Ingest(const core::StreamEnvelope& env);
  std::string WorldJson() const;

 private:
  std::unique_ptr<core::SimulationWorldService> service_;
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
