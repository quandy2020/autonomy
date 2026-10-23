#include "slam_system_builder.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace app {
namespace {

TEST(SlamSystemBuilder, RegistersBuiltinFrontendTypes) {
  const std::vector<std::string> types = SlamSystemBuilder::BuiltinTypes();
  ASSERT_EQ(types.size(), 2u);
  EXPECT_EQ(types[0], "lidar_frontend");
  EXPECT_EQ(types[1], "visual_frontend");

  common::ComponentFactory factory;
  SlamSystemBuilder{}.RegisterBuiltins(&factory);
  common::Component::Config config;
  config.name = "front";
  std::unique_ptr<common::Component> created =
      factory.Create("lidar_frontend", config);
  ASSERT_NE(created, nullptr);
  EXPECT_EQ(created->name(), "front");
  EXPECT_EQ(factory.Create("missing", config), nullptr);
}

TEST(SlamSystemBuilder, BuildInstallsConfiguredComponents) {
  SlamSystemConfig config = DefaultSlamSystemConfig();
  config.components = {{"lidar", "lidar_frontend"},
                       {"vision", "visual_frontend"}};
  const std::unique_ptr<SlamSystem> system = SlamSystemBuilder{}.Build(config);
  ASSERT_NE(system, nullptr);
  common::Component* lidar = system->registry().Find("lidar");
  common::Component* vision = system->registry().Find("vision");
  ASSERT_NE(lidar, nullptr);
  ASSERT_NE(vision, nullptr);
  EXPECT_TRUE(lidar->Start());
  EXPECT_EQ(lidar->health().level, common::HealthLevel::kOk);
  EXPECT_EQ(lidar->health().message, "lidar_frontend");
  EXPECT_EQ(vision->health().level, common::HealthLevel::kUnknown);

  config.components.push_back({"other", "not_a_frontend"});
  EXPECT_EQ(SlamSystemBuilder{}.Build(config), nullptr);
}

TEST(SlamSystemBuilder, VisualPipelineBuildsWithoutLidarSteps) {
  SlamSystemConfig config = DefaultSlamSystemConfig();
  config.pipeline = "visual";
  config.components = {{"vision", "visual_frontend"}};
  const std::unique_ptr<SlamSystem> system = SlamSystemBuilder{}.Build(config);
  ASSERT_NE(system, nullptr);
  ASSERT_TRUE(system->Start());
  EXPECT_FALSE(system->AddScan(1.0));
}

}  // namespace
}  // namespace app
}  // namespace atlas
