/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/pipeline/planning_request_adapters.hpp"
#include "autonomy/manipulation/plugin_ids.hpp"
#include "autonomy/manipulation/dispatch/capability/capabilities.hpp"

namespace autonomy {
namespace manipulation {
namespace {

TEST(PluginsTest, RegistersDefaultPlanners) {
  RegisterManipulationPlugins();
  auto planner = CreatePlugin<common::PlannerInterface>("pilz_ptp");
  ASSERT_NE(planner, nullptr);
  EXPECT_TRUE(planner->Init("pilz_ptp"));

  EXPECT_NE(CreatePlugin<common::PlannerInterface>("chomp"), nullptr);
  EXPECT_NE(CreatePlugin<common::PlannerInterface>("stomp"), nullptr);
  EXPECT_NE(CreatePlugin<common::PlannerInterface>("ompl"), nullptr);
  EXPECT_NE(CreatePlugin<common::PlannerInterface>("pilz_sequence"), nullptr);
  EXPECT_NE(CreatePlugin<common::CollisionInterface>("aabb"), nullptr);
  EXPECT_NE(CreatePlugin<common::KinematicsInterface>("stub"), nullptr);
}

TEST(PluginsTest, ResolveAliasAndClassName) {
  EXPECT_EQ(ResolvePluginAlias("ompl"), "OmplInterfacePlanner");
  EXPECT_EQ(ResolvePluginAlias("chomp"), "ChompPlanner");
  EXPECT_EQ(ResolvePluginAlias("stomp"), "StompPlanner");
  EXPECT_EQ(ResolvePluginAlias("OmplPlanner"), "OmplPlanner");
  EXPECT_EQ(ResolvePluginAlias("ompl_interface"), "OmplInterfacePlanner");
  EXPECT_EQ(ResolvePluginAlias("trac_ik"), "TracIkKinematics");
  EXPECT_EQ(ResolvePluginAlias("arm_controller"),
            "JointTrajectoryController");
  EXPECT_EQ(ResolvePluginAlias("check_constraints"),
            "CheckPathConstraintsAdapter");
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
  auto time_param = CreatePlugin<planner::PlanningRequestAdapter>(
      "time_parameterization");
  ASSERT_NE(time_param, nullptr);
  EXPECT_EQ(time_param->GetName(), "time_parameterization");
  EXPECT_NE(CreatePlugin<planner::PlanningRequestAdapter>(
                "ApplyTimeParameterizationAdapter"),
            nullptr);
  EXPECT_NE(
      CreatePlugin<planner::PlanningRequestAdapter>("check_constraints"),
      nullptr);
  EXPECT_NE(CreatePlugin<planner::PlanningRequestAdapter>(
                "fix_start_state_path_constraints"),
            nullptr);
}

TEST(PluginsTest, CreateCapabilities) {
  RegisterManipulationPlugins();
  EXPECT_NE(CreatePlugin<dispatch::Capability>("plan"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("cartesian_path"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("state_validation"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("query_planners"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("clear_octomap"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("clear_scene"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("validate_trajectory"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("get_urdf"), nullptr);
  EXPECT_NE(CreatePlugin<dispatch::Capability>("save_load_geometry"), nullptr);
}

}  // namespace
}  // namespace manipulation
}  // namespace autonomy
