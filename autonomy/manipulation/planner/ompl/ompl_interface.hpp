/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL interface façade (MoveIt ompl_interface lite).
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/constraints/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/planner/ompl/model_based_planning_context.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planner.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planning_config.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/**
 * @brief Thin ompl_interface: configs + ConstraintSamplerManager + Context + Plan().
 *
 * Full MoveIt ompl_interface also owns ModelBasedPlanningContext factories per
 * group; this lite keeps one context backed by OmplPlanner.
 */
class OmplInterface {
 public:
  /**
   * @brief Init planner; load @p config_path or DefaultOmplPlannerConfigs().
   */
  bool Init(const std::string& default_planner_id = "ompl",
            const std::string& config_path = {});

  constraints::ConstraintSamplerManager* GetConstraintSamplerManager() {
    return &sampler_manager_;
  }

  ModelBasedPlanningContext* GetPlanningContext() { return &context_; }

  void SetPlannerConfigs(std::vector<OmplPlannerConfig> configs) {
    configs_ = std::move(configs);
    by_name_.clear();
    for (const auto& c : configs_) {
      by_name_[c.name] = c;
    }
  }

  /** @brief Load conf file; on failure keep current / apply defaults. */
  bool LoadPlannerConfigs(const std::string& path, std::string* error = nullptr);

  const std::vector<OmplPlannerConfig>& GetPlannerConfigs() const {
    return configs_;
  }

  /** @brief Plan using optional named config override on @p request.pb.planner_id(). */
  ::autonomy::manipulation::proto::MotionPlanResponse Plan(const MotionPlanRequest& request);

 private:
  const OmplPlannerConfig* LookupConfig(const std::string& planner_id) const;

  common::PlannerInterface::SharedPtr planner_;
  std::string default_id_ = "ompl";
  std::vector<OmplPlannerConfig> configs_;
  std::unordered_map<std::string, OmplPlannerConfig> by_name_;
  constraints::ConstraintSamplerManager sampler_manager_;
  ModelBasedPlanningContext context_;
};

/**
 * @brief PlannerInterface plugin that routes through OmplInterface (conf + context).
 *
 * Registered as plugin class OmplInterfacePlanner; alias "ompl_interface".
 * Default "ompl" still maps to OmplPlanner for backward compatibility; server
 * prefers OmplInterface when available via CreateOmplInterfacePlanner().
 */
class OmplInterfacePlanner : public common::PlannerInterface {
 public:
  bool Init(const std::string& planner_id) override;
  ::autonomy::manipulation::proto::MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  OmplInterface interface_;
  std::string planner_id_;
};

common::PlannerInterface::SharedPtr CreateOmplInterfacePlanner();

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
