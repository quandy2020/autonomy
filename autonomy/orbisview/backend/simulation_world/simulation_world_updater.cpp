/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/simulation_world/simulation_world_updater.hpp"

#include <cmath>

#include "autonomy/orbisview/backend/common/util/json_util.hpp"
#include "autonomy/orbisview/backend/common/render_schemas.hpp"

namespace autonomy {
namespace orbisview {
namespace backend {

SimulationWorldUpdater::SimulationWorldUpdater()
    : service_(std::make_unique<core::SimulationWorldService>()) {}

void SimulationWorldUpdater::Ingest(const core::StreamEnvelope& env) {
  if (env.encoding != "json" || env.payload.empty() || !service_) return;
  const std::string json(env.payload.begin(), env.payload.end());
  using namespace rendering;
  using util::ExtractJsonBool;
  using util::ExtractJsonNumber;
  using util::ExtractJsonString;

  if (env.schema == kSchemaPose) {
    service_->SetPose(ExtractJsonNumber(json, "x", 0.0),
                      ExtractJsonNumber(json, "y", 0.0),
                      ExtractJsonNumber(json, "yaw", 0.0));
  } else if (env.schema == kSchemaChassis || env.schema == kSchemaTwist2D) {
    core::WorldChassis c;
    c.vx = ExtractJsonNumber(json, "vx", 0.0);
    c.wz = ExtractJsonNumber(json, "wz", 0.0);
    c.gear = ExtractJsonString(json, "gear");
    if (c.gear.empty()) c.gear = "D";
    c.throttle = ExtractJsonNumber(json, "throttle", std::abs(c.vx));
    c.brake = ExtractJsonNumber(json, "brake", 0.0);
    c.steering = ExtractJsonNumber(json, "steering", c.wz);
    c.driving_mode = ExtractJsonString(json, "driving_mode");
    if (c.driving_mode.empty()) c.driving_mode = "AUTO";
    service_->SetChassis(c);
  } else if (env.schema == kSchemaNavigation) {
    if (ExtractJsonBool(json, "has_goal", false)) {
      const double gx = ExtractJsonNumber(json, "x", 0.0);
      const double gy = ExtractJsonNumber(json, "y", 0.0);
      if (json.find("\"goal\"") != std::string::npos) {
        auto gpos = json.find("\"goal\"");
        auto brace = json.find('{', gpos);
        auto end = json.find('}', brace);
        if (brace != std::string::npos && end != std::string::npos) {
          const auto g = json.substr(brace, end - brace + 1);
          service_->SetGoal(ExtractJsonNumber(g, "x", 0.0),
                            ExtractJsonNumber(g, "y", 0.0),
                            ExtractJsonNumber(g, "yaw", 0.0));
        } else {
          service_->SetGoal(gx, gy);
        }
      }
    }
  }
}

std::string SimulationWorldUpdater::WorldJson() const {
  return service_ ? service_->ToJson() : "{}";
}

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
