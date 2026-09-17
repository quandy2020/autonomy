/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/common/plugin_ids.hpp"

#include <mutex>
#include <unordered_map>

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/common/controller_interface.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler_allocator.hpp"
#include "autonomy/manipulation/planner/pipeline/planning_request_adapter.hpp"

namespace autonomy {
namespace manipulation {
namespace {

const std::unordered_map<std::string, std::string>& AliasMap() {
  static const std::unordered_map<std::string, std::string> kAliases = {
      {"joint_interpolation", "JointInterpolationPlanner"},
      {"cartesian", "CartesianPlanner"},
      {"rrt_connect", "RrtConnectPlanner"},
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
      {"hybrid", "HybridPlanner"},
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
      {"stub", "StubKinematics"},
      {"kdl", "KdlKinematics"},
      {"ikfast", "IkFastKinematics"},
      {"cached_kdl", "CachedKinematics"},
      {"trac_ik", "TracIkKinematics"},
      {"TracIkKinematics", "TracIkKinematics"},
      {"aabb", "AabbCollisionDetector"},
      {"fcl", "FclCollisionDetector"},
      {"simple", "SimpleControllerManager"},
      {"autolink_trajectory", "AutolinkTrajectoryController"},
      {"arm_controller", "AutolinkTrajectoryController"},
      {"time_parameterization", "ApplyTimeParameterizationAdapter"},
      {"TimeParameterizeAdapter", "ApplyTimeParameterizationAdapter"},
      {"ApplyTimeParameterizationAdapter",
       "ApplyTimeParameterizationAdapter"},
      {"validate_path", "ValidatePathAdapter"},
      {"dense_sample", "DenseSampleAdapter"},
      {"check_constraints", "CheckConstraintsAdapter"},
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

    pm->RegisterInProcessClass<planning::PlannerBase>(
        "JointInterpolationPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("CartesianPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("RrtConnectPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("PilzPtpPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("PilzLinPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("PilzCircPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("PilzSequencePlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("ChompPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("StompPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("HybridPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("OmplPlanner");
    pm->RegisterInProcessClass<planning::PlannerBase>("OmplInterfacePlanner");

    pm->RegisterInProcessClass<kinematics::KinematicsBase>("StubKinematics");
    pm->RegisterInProcessClass<kinematics::KinematicsBase>("IkFastKinematics");
    pm->RegisterInProcessClass<kinematics::KinematicsBase>("CachedKinematics");
    pm->RegisterInProcessClass<kinematics::KinematicsBase>("TracIkKinematics");
#ifdef AUTONOMY_HAS_KDL
    pm->RegisterInProcessClass<kinematics::KinematicsBase>("KdlKinematics");
#endif

    pm->RegisterInProcessClass<constraint_samplers::ConstraintSamplerAllocator>(
        "UnionConstraintSamplerAllocator");
    pm->RegisterInProcessClass<constraint_samplers::ConstraintSamplerAllocator>(
        "IkConstraintSamplerAllocator");
    pm->RegisterInProcessClass<constraint_samplers::ConstraintSamplerAllocator>(
        "JointConstraintSamplerAllocator");

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
        "ApplyTimeParameterizationAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "ValidatePathAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "DenseSampleAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "CheckConstraintsAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "FixStartStateBoundsAdapter");
    pm->RegisterInProcessClass<planning::PlanningRequestAdapter>(
        "FixStartStatePathConstraintsAdapter");

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
    pm->RegisterInProcessClass<server::Capability>("GetDynamicsCapability");
  });
}

}  // namespace manipulation
}  // namespace autonomy
