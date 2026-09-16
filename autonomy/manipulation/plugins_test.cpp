/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/collision/collision_detector.hpp"
#include "autonomy/manipulation/kinematics/kinematics_base.hpp"
#include "autonomy/manipulation/planning/planner_base.hpp"
#include "autonomy/manipulation/planning/planning_request_adapter.hpp"
#include "autonomy/manipulation/plugins.hpp"
#include "autonomy/manipulation/server/capability.hpp"

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
  EXPECT_NE(CreatePlugin<collision::CollisionDetector>("aabb"), nullptr);
  EXPECT_NE(CreatePlugin<kinematics::KinematicsBase>("stub"), nullptr);
}

TEST(PluginsTest, ResolveAliasAndClassName) {
  EXPECT_EQ(ResolvePluginAlias("ompl"), "OmplPlanner");
  EXPECT_EQ(ResolvePluginAlias("hybrid"), "HybridPlanner");
  EXPECT_EQ(ResolvePluginAlias("OmplPlanner"), "OmplPlanner");
  EXPECT_EQ(ResolvePluginAlias("arm_controller"),
            "AutolinkTrajectoryController");
  EXPECT_EQ(ResolvePluginAlias("check_constraints"),
            "CheckConstraintsAdapter");
  EXPECT_EQ(ResolvePluginAlias("fix_start_state_bounds"),
            "FixStartStateBoundsAdapter");
}

TEST(PluginsTest, CreateAdapters) {
  RegisterManipulationPlugins();
  EXPECT_NE(CreatePlugin<planning::PlanningRequestAdapter>(
                "time_parameterization"),
            nullptr);
  EXPECT_NE(
      CreatePlugin<planning::PlanningRequestAdapter>("check_constraints"),
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
