#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"

#include <gtest/gtest.h>

namespace autoviz::converter {

TEST(FoxgloveRegistry, NavPathMapsToSceneUpdate) {
  const AutomsgsFoxgloveEntry* e = FindFoxgloveRule("automsgs.msgs.nav_msgs.Path");
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->proto_full_name, "automsgs.msgs.nav_msgs.Path");
  EXPECT_EQ(e->foxglove_canonical_name, "foxglove.SceneUpdate");
  EXPECT_EQ(e->strategy, AutomsgsFoxgloveStrategy::kConvertToSceneUpdate);
}

TEST(FoxgloveRegistry, UnknownTypeReturnsNull) {
  EXPECT_EQ(FindFoxgloveRule("definitely.not.ARealMessageType"), nullptr);
}

TEST(FoxgloveRegistry, GenericMessageUsesLogStrategy) {
  const AutomsgsFoxgloveEntry* e =
      FindFoxgloveRule("automsgs.msgs.geometry_msgs.Point");
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->strategy, AutomsgsFoxgloveStrategy::kConvertToLog);
  EXPECT_EQ(e->foxglove_canonical_name, "foxglove.Log");
}

TEST(FoxgloveRegistry, LaserScanTarget) {
  const AutomsgsFoxgloveEntry* e =
      FindFoxgloveRule("automsgs.msgs.sensor_msgs.LaserScan");
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->foxglove_canonical_name, "foxglove.LaserScan");
  EXPECT_EQ(e->strategy, AutomsgsFoxgloveStrategy::kConvertToLaserScan);
}

}  // namespace autoviz::converter
