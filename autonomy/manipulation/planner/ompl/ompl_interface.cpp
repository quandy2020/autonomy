/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/ompl/ompl_interface.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

bool OmplInterface::Init(const std::string& default_planner_id,
                         const std::string& config_path) {
  planner_ = CreateOmplPlanner();
  if (!planner_ || !planner_->Init(default_planner_id)) {
    return false;
  }
  default_id_ = default_planner_id;
  context_.SetPlanner(planner_);
  context_.SetConstraintSamplerManager(&sampler_manager_);
  sampler_manager_.LoadExternalPluginDescriptions("");

  std::string path = config_path;
  if (path.empty()) {
    common::ResolveModuleConfPath("manipulation", "ompl_planning.conf", &path);
  }
  std::string err;
  if (!path.empty() && LoadPlannerConfigs(path, &err)) {
    AINFO << "OmplInterface loaded " << configs_.size()
          << " planner configs from " << path;
  } else {
    if (!path.empty() && !err.empty()) {
      AWARN << "OmplInterface config load failed (" << err
            << "); using defaults";
    }
    SetPlannerConfigs(DefaultOmplPlannerConfigs());
  }
  return true;
}

bool OmplInterface::LoadPlannerConfigs(const std::string& path,
                                       std::string* error) {
  std::vector<OmplPlannerConfig> cfgs;
  if (!LoadOmplPlannerConfigsFile(path, &cfgs, error)) {
    return false;
  }
  SetPlannerConfigs(std::move(cfgs));
  return true;
}

const OmplPlannerConfig* OmplInterface::LookupConfig(
    const std::string& planner_id) const {
  if (planner_id.empty()) {
    return nullptr;
  }
  const auto it = by_name_.find(planner_id);
  if (it != by_name_.end()) {
    return &it->second;
  }
  return nullptr;
}

MotionPlanResponse OmplInterface::Plan(const MotionPlanRequest& request) {
  OmplPlannerConfig cfg;
  cfg.planner_id = default_id_;
  if (const auto* found = LookupConfig(request.planner_id)) {
    cfg = *found;
  } else if (!configs_.empty()) {
    // Fall back: match bare type against config.planner_id.
    for (const auto& c : configs_) {
      if (c.planner_id == request.planner_id || c.name == request.planner_id) {
        cfg = c;
        break;
      }
    }
  }
  context_.SetConfig(cfg);
  return context_.Solve(request);
}

bool OmplInterfacePlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id.empty() ? "ompl" : planner_id;
  return interface_.Init(planner_id_);
}

MotionPlanResponse OmplInterfacePlanner::Plan(
    const MotionPlanRequest& request) {
  MotionPlanRequest req = request;
  if (req.planner_id.empty()) {
    req.planner_id = planner_id_;
  }
  return interface_.Plan(req);
}

std::shared_ptr<PlannerBase> CreateOmplInterfacePlanner() {
  return std::make_shared<OmplInterfacePlanner>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(OmplInterfacePlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
