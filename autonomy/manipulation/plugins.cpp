/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/plugins.hpp"

#include <mutex>
#include <unordered_map>

#include "autonomy/manipulation/collision/collision_detector.hpp"
#include "autonomy/manipulation/execution/controller_manager.hpp"
#include "autonomy/manipulation/kinematics/kinematics_base.hpp"
#include "autonomy/manipulation/planning/planner_base.hpp"
#include "autonomy/manipulation/planning/planning_request_adapter.hpp"
#include "autonomy/manipulation/server/capability.hpp"

namespace autonomy {
namespace manipulation {
namespace {

const std::unordered_map<std::string, std::string>& AliasMap() {
  static const std::unordered_map<std::string, std::string> kAliases = {
      // planners
      {"joint_interpolation", "JointInterpolationPlanner"},
      {"cartesian", "CartesianPlanner"},
      {"rrt_connect", "RrtConnectPlanner"},
      {"pilz_ptp", "PilzPtpPlanner"},
      {"pilz_lin", "PilzLinPlanner"},
      {"pilz_circ", "PilzCircPlanner"},
      {"LIN", "PilzLinPlanner"},
      {"CIRC", "PilzCircPlanner"},
      {"PTP", "PilzPtpPlanner"},
      {"chomp", "ChompPlanner"},
      {"stomp", "StompPlanner"},
      {"hybrid", "HybridPlanner"},
      {"ompl", "OmplPlanner"},
      // kinematics
      {"stub", "StubKinematics"},
      {"kdl", "KdlKinematics"},
      {"ikfast", "IkFastKinematics"},
      {"cached_kdl", "CachedKinematics"},
      // collision
      {"aabb", "AabbCollisionDetector"},
      {"fcl", "FclCollisionDetector"},
      // controllers
      {"simple", "SimpleControllerManager"},
      {"autolink_trajectory", "AutolinkTrajectoryController"},
      {"arm_controller", "AutolinkTrajectoryController"},
      // adapters
      {"time_parameterization", "TimeParameterizeAdapter"},
      {"validate_path", "ValidatePathAdapter"},
      {"dense_sample", "DenseSampleAdapter"},
      {"check_constraints", "CheckConstraintsAdapter"},
      {"fix_start_state_bounds", "FixStartStateBoundsAdapter"},
      // capabilities (avoid clashing with planner id "cartesian")
      {"plan", "PlanCapability"},
      {"execute", "ExecuteCapability"},
      {"plan_and_execute", "PlanAndExecuteCapability"},
      {"cartesian_path", "CartesianCapability"},
      {"fk_ik", "FkIkCapability"},
      {"get_planning_scene", "GetPlanningSceneCapability"},
      {"apply_planning_scene", "ApplyPlanningSceneCapability"},
      {"state_validation", "StateValidationCapability"},
      {"query_planners", "QueryPlannersCapability"},
      {"clear_octomap", "ClearOctomapCapability"},
      {"clear_scene", "ClearSceneCapability"},
      {"validate_trajectory", "ValidateTrajectoryCapability"},
      {"get_urdf", "GetUrdfCapability"},
      {"save_load_geometry", "SaveLoadGeometryCapability"},
  };
  return kAliases;
}

}  // namespace

std::string ResolvePluginAlias(const std::string& id) {
  const auto& aliases = AliasMap();
  const auto it = aliases.find(id);
  return it == aliases.end() ? id : it->second;
}

void RegisterManipulationPlugins() {
  static std::once_flag once;
  std::call_once(once, [] {
    auto* pm = autolink::plugin_manager::PluginManager::Instance();

    pm->RegisterInProcessClass<planning::PlannerBase>(
        "JointInterpolationPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("CartesianPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("RrtConnectPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("PilzPtpPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("PilzLinPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("PilzCircPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("ChompPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("StompPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("HybridPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("OmplPlanner");

    pm->RegisterInProcessClass<kinematics::KinematicsBase>("StubKinematics");
    pm->RegisterInProcessClass<kinematics::KinematicsBase>("IkFastKinematics");
    pm->RegisterInProcessClass<kinematics::KinematicsBase>("CachedKinematics");
#ifdef AUTONOMY_HAS_KDL
    pm->RegisterInProcessClass<kinematics::KinematicsBase>("KdlKinematics");
#endif

    pm->RegisterInProcessClass<collision::CollisionDetector>(
        "AabbCollisionDetector");
#ifdef AUTONOMY_HAS_FCL
    pm->RegisterInProcessClass<collision::CollisionDetector>(
        "FclCollisionDetector");
#endif

    pm->RegisterInProcessClass<execution::ControllerManager>(
        "SimpleControllerManager");
    pm->RegisterInProcessClass<execution::ControllerManager>(
        "AutolinkTrajectoryController");

    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "TimeParameterizeAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "ValidatePathAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "DenseSampleAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "CheckConstraintsAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "FixStartStateBoundsAdapter");

    pm->RegisterInProcessClass<server::Capability>("PlanCapability");
    pm->RegisterInProcessClass<server::Capability>("ExecuteCapability");
    pm->RegisterInProcessClass<server::Capability>("PlanAndExecuteCapability");
    pm->RegisterInProcessClass<server::Capability>("CartesianCapability");
    pm->RegisterInProcessClass<server::Capability>("FkIkCapability");
    pm->RegisterInProcessClass<server::Capability>(
        "GetPlanningSceneCapability");
    pm->RegisterInProcessClass<server::Capability>(
        "ApplyPlanningSceneCapability");
    pm->RegisterInProcessClass<server::Capability>("StateValidationCapability");
    pm->RegisterInProcessClass<server::Capability>("QueryPlannersCapability");
    pm->RegisterInProcessClass<server::Capability>("ClearOctomapCapability");
    pm->RegisterInProcessClass<server::Capability>("ClearSceneCapability");
    pm->RegisterInProcessClass<server::Capability>(
        "ValidateTrajectoryCapability");
    pm->RegisterInProcessClass<server::Capability>("GetUrdfCapability");
    pm->RegisterInProcessClass<server::Capability>(
        "SaveLoadGeometryCapability");
  });
}

}  // namespace manipulation
}  // namespace autonomy
