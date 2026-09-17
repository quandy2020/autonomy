/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/plugin_ids.hpp"

#include <mutex>
#include <unordered_map>

#include "autonomy/manipulation/dispatch/capability/capabilities.hpp"
#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/common/controller_interface.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/constraints/constraint_sampler_types.hpp"
#include "autonomy/manipulation/pipeline/planning_request_adapters.hpp"

namespace autonomy {
namespace manipulation {
namespace {

const std::unordered_map<std::string, std::string>& AliasMap() {
  static const std::unordered_map<std::string, std::string> kAliases = {
      {"pilz_ptp", "PilzPtpPlanner"},
      {"pilz_lin", "PilzLinPlanner"},
      {"pilz_circ", "PilzCircPlanner"},
      {"pilz_sequence", "PilzSequencePlanner"},
      {"LIN", "PilzLinPlanner"},
      {"CIRC", "PilzCircPlanner"},
      {"PTP", "PilzPtpPlanner"},
      {"SEQUENCE", "PilzSequencePlanner"},
      {"chomp", "ChompPlanner"},
      {"stomp", "StompPlanner"},
      {"ompl", "OmplInterfacePlanner"},
      {"ompl_interface", "OmplInterfacePlanner"},
      {"OmplPlanner", "OmplPlanner"},
      {"RRTConnect", "OmplInterfacePlanner"},
      {"RRT", "OmplInterfacePlanner"},
      {"RRTstar", "OmplInterfacePlanner"},
      {"KPIECE", "OmplInterfacePlanner"},
      {"BiTRRT", "OmplInterfacePlanner"},
      {"EST", "OmplInterfacePlanner"},
      {"bitrrt", "OmplInterfacePlanner"},
      {"est", "OmplInterfacePlanner"},
      {"stub", "NullKinematics"},
      {"null", "NullKinematics"},
      {"kdl", "KdlKinematics"},
      {"ikfast", "IkFastKinematics"},
      {"cached_kdl", "CachedKinematics"},
      {"trac_ik", "TracIkKinematics"},
      {"TracIkKinematics", "TracIkKinematics"},
      {"aabb", "AabbCollisionDetector"},
      {"fcl", "FclCollisionDetector"},
      {"simple", "LoggingTrajectoryController"},
      {"joint_trajectory", "JointTrajectoryController"},
      {"arm_controller", "JointTrajectoryController"},
      {"time_parameterization", "ApplyTimeParameterizationAdapter"},
      {"TimeParameterizeAdapter", "ApplyTimeParameterizationAdapter"},
      {"ApplyTimeParameterizationAdapter",
       "ApplyTimeParameterizationAdapter"},
      {"validate_path", "ValidateTrajectoryPathAdapter"},
      {"ValidatePathAdapter", "ValidateTrajectoryPathAdapter"},
      {"ValidateTrajectoryPathAdapter", "ValidateTrajectoryPathAdapter"},
      {"dense_sample", "DensifyJointTrajectoryAdapter"},
      {"DenseSampleAdapter", "DensifyJointTrajectoryAdapter"},
      {"DensifyJointTrajectoryAdapter", "DensifyJointTrajectoryAdapter"},
      {"check_constraints", "CheckPathConstraintsAdapter"},
      {"CheckConstraintsAdapter", "CheckPathConstraintsAdapter"},
      {"CheckPathConstraintsAdapter", "CheckPathConstraintsAdapter"},
      {"fix_start_state_bounds", "FixStartStateBoundsAdapter"},
      {"fix_start_state_path_constraints",
       "FixStartStatePathConstraintsAdapter"},
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
      {"get_dynamics", "GetDynamicsCapability"},
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

    pm->RegisterInProcessClass<common::PlannerInterface>("PilzPtpPlanner");
    pm->RegisterInProcessClass<common::PlannerInterface>("PilzLinPlanner");
    pm->RegisterInProcessClass<common::PlannerInterface>("PilzCircPlanner");
    pm->RegisterInProcessClass<common::PlannerInterface>("PilzSequencePlanner");
    pm->RegisterInProcessClass<common::PlannerInterface>("ChompPlanner");
    pm->RegisterInProcessClass<common::PlannerInterface>("StompPlanner");
    pm->RegisterInProcessClass<common::PlannerInterface>("OmplPlanner");
    pm->RegisterInProcessClass<common::PlannerInterface>("OmplInterfacePlanner");

    pm->RegisterInProcessClass<common::KinematicsInterface>("NullKinematics");
    pm->RegisterInProcessClass<common::KinematicsInterface>("IkFastKinematics");
    pm->RegisterInProcessClass<common::KinematicsInterface>("CachedKinematics");
    pm->RegisterInProcessClass<common::KinematicsInterface>("TracIkKinematics");
#ifdef AUTONOMY_HAS_KDL
    pm->RegisterInProcessClass<common::KinematicsInterface>("KdlKinematics");
#endif

    pm->RegisterInProcessClass<constraints::ConstraintSamplerAllocator>(
        "UnionConstraintSamplerAllocator");
    pm->RegisterInProcessClass<constraints::ConstraintSamplerAllocator>(
        "IkConstraintSamplerAllocator");
    pm->RegisterInProcessClass<constraints::ConstraintSamplerAllocator>(
        "JointConstraintSamplerAllocator");

    pm->RegisterInProcessClass<common::CollisionInterface>(
        "AabbCollisionDetector");
#ifdef AUTONOMY_HAS_FCL
    pm->RegisterInProcessClass<common::CollisionInterface>(
        "FclCollisionDetector");
#endif

    pm->RegisterInProcessClass<common::ControllerInterface>(
        "LoggingTrajectoryController");
    pm->RegisterInProcessClass<common::ControllerInterface>(
        "JointTrajectoryController");

    pm->RegisterInProcessClass<planner::PlanningRequestAdapter>(
        "ApplyTimeParameterizationAdapter");
    pm->RegisterInProcessClass<planner::PlanningRequestAdapter>(
        "ValidateTrajectoryPathAdapter");
    pm->RegisterInProcessClass<planner::PlanningRequestAdapter>(
        "DensifyJointTrajectoryAdapter");
    pm->RegisterInProcessClass<planner::PlanningRequestAdapter>(
        "CheckPathConstraintsAdapter");
    pm->RegisterInProcessClass<planner::PlanningRequestAdapter>(
        "FixStartStateBoundsAdapter");
    pm->RegisterInProcessClass<planner::PlanningRequestAdapter>(
        "FixStartStatePathConstraintsAdapter");

    pm->RegisterInProcessClass<dispatch::Capability>("PlanCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("ExecuteCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("PlanAndExecuteCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("CartesianCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("FkIkCapability");
    pm->RegisterInProcessClass<dispatch::Capability>(
        "GetPlanningSceneCapability");
    pm->RegisterInProcessClass<dispatch::Capability>(
        "ApplyPlanningSceneCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("StateValidationCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("QueryPlannersCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("ClearOctomapCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("ClearSceneCapability");
    pm->RegisterInProcessClass<dispatch::Capability>(
        "ValidateTrajectoryCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("GetUrdfCapability");
    pm->RegisterInProcessClass<dispatch::Capability>(
        "SaveLoadGeometryCapability");
    pm->RegisterInProcessClass<dispatch::Capability>("GetDynamicsCapability");
  });
}

}  // namespace manipulation
}  // namespace autonomy
