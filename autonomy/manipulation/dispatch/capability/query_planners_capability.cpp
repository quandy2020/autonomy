/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/query_planners_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool QueryPlannersCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

std::vector<PlannerInterfaceInfo> QueryPlannersCapability::ListPlanners()
    const {
  struct Entry {
    const char* id;
    const char* desc;
  };
  static const Entry kIds[] = {
      {"ompl", "OMPL via OmplInterface + ompl_planning.conf"},
      {"ompl_interface", "OmplInterfacePlanner (configs + Context)"},
      {"pilz_ptp", "Pilz PTP cosine joint motion"},
      {"pilz_lin", "Pilz LIN Cartesian straight line"},
      {"pilz_circ", "Pilz CIRC circular arc"},
      {"pilz_sequence", "Pilz Sequence (PTP/LIN/CIRC + blend)"},
      {"chomp", "CHOMP covariant gradient optimizer"},
      {"stomp", "STOMP correlated-noise optimizer"},
  };
  std::vector<PlannerInterfaceInfo> out;
  for (const auto& e : kIds) {
    PlannerInterfaceInfo info;
    info.set_name(e.id);
    info.set_pipeline_id("manipulation");
    info.set_description(e.desc);
    out.push_back(info);
  }
  return out;
}

std::unordered_map<std::string, std::string>
QueryPlannersCapability::GetPlannerParams(const std::string& planner_id) const {
  std::unordered_map<std::string, std::string> params;
  if (!server_) {
    return params;
  }
  const auto& opt = server_->options();
  params["planner_id"] =
      planner_id.empty() ? opt.planner_id() : planner_id;
  params["planning_time_milliseconds"] = std::to_string(opt.planning_time_milliseconds());
  params["max_velocity"] = std::to_string(opt.max_velocity());
  params["max_acceleration"] = std::to_string(opt.max_acceleration());
  params["max_attempts"] = std::to_string(opt.max_planning_attempts());
  return params;
}

bool QueryPlannersCapability::SetPlannerParams(
    const std::string& planner_id,
    const std::unordered_map<std::string, std::string>& params) {
  if (!server_) {
    return false;
  }
  return server_->SetPlannerParams(planner_id, params);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(QueryPlannersCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
