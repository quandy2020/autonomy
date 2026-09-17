/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/planner/pipeline/planning_request_adapter.hpp"
#include "autonomy/manipulation/common/plugin_ids.hpp"
#include "autonomy/manipulation/dispatch/capability/capability.hpp"

namespace autonomy {
namespace manipulation {
namespace {

TEST(PluginsTest, RegistersDefaultPlanners) {
  RegisterManipulationPlugins();
  auto planner = CreatePlugin<planning::PlannerBase>("joint_interpolation");
  ASSERT_NE(planner, nullptr);
  EXPECT_TRUE(planner->Init("joint_interpolation"));

  EXPECT_NE(CreatePlugin<planning::PlannerBase>("rrt_connect"), nullptr);
  EXPECT_NE(CreatePlugin<planning::PlannerBase>("cartesian"), nullptr);
  EXPECT_NE(CreatePlugin<planning::PlannerBase>("hybrid"), nullptr);
  EXPECT_NE(CreatePlugin<planning::PlannerBase>("chomp"), nullptr);
  EXPECT_NE(CreatePlugin<planning::PlannerBase>("pilz_sequence"), nullptr);
  EXPECT_NE(CreatePlugin<collision::CollisionDetector>("aabb"), nullptr);
  EXPECT_NE(CreatePlugin<kinematics::KinematicsBase>("stub"), nullptr);
}

TEST(PluginsTest, ResolveAliasAndClassName) {
  EXPECT_EQ(ResolvePluginAlias("ompl"), "OmplInterfacePlanner");
  EXPECT_EQ(ResolvePluginAlias("hybrid"), "HybridPlanner");
  EXPECT_EQ(ResolvePluginAlias("OmplPlanner"), "OmplPlanner");
  EXPECT_EQ(ResolvePluginAlias("ompl_interface"), "OmplInterfacePlanner");
  EXPECT_EQ(ResolvePluginAlias("trac_ik"), "TracIkKinematics");
  EXPECT_EQ(ResolvePluginAlias("arm_controller"),
            "AutolinkTrajectoryController");
  EXPECT_EQ(ResolvePluginAlias("check_constraints"),
            "CheckConstraintsAdapter");
  EXPECT_EQ(ResolvePluginAlias("fix_start_state_bounds"),
            "FixStartStateBoundsAdapter");
  EXPECT_EQ(ResolvePluginAlias("fix_start_state_path_constraints"),
            "FixStartStatePathConstraintsAdapter");
  EXPECT_EQ(ResolvePluginAlias("time_parameterization"),
            "ApplyTimeParameterizationAdapter");
  EXPECT_EQ(ResolvePluginAlias("TimeParameterizeAdapter"),
            "ApplyTimeParameterizationAdapter");
}

TEST(PluginsTest, CreateAdapters) {
  RegisterManipulationPlugins();
  // Alias "time_parameterization" → ApplyTimeParameterizationAdapter.
  auto time_param = CreatePlugin<planning::PlanningRequestAdapter>(
      "time_parameterization");
  ASSERT_NE(time_param, nullptr);
  EXPECT_EQ(time_param->GetName(), "time_parameterization");
  EXPECT_NE(CreatePlugin<planning::PlanningRequestAdapter>(
                "ApplyTimeParameterizationAdapter"),
            nullptr);
  EXPECT_NE(
      CreatePlugin<planning::PlanningRequestAdapter>("check_constraints"),
      nullptr);
  EXPECT_NE(CreatePlugin<planning::PlanningRequestAdapter>(
                "fix_start_state_path_constraints"),
            nullptr);
}

TEST(PluginsTest, CreateCapabilities) {
  RegisterManipulationPlugins();
  EXPECT_NE(CreatePlugin<server::Capability>("plan"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("cartesian_path"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("state_validation"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("query_planners"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("clear_octomap"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("clear_scene"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("validate_trajectory"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("get_urdf"), nullptr);
  EXPECT_NE(CreatePlugin<server::Capability>("save_load_geometry"), nullptr);
}

}  // namespace
}  // namespace manipulation
}  // namespace autonomy
