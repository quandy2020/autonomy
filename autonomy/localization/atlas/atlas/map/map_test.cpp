#include "atlas/map/map.hpp"

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(MapTest, StartsEmpty) {
    Map map;
    EXPECT_EQ(map.KeyFramesInMap(), 0U);
    EXPECT_EQ(map.MapPointsInMap(), 0U);
    EXPECT_EQ(map.GetCoordinateEpoch(), 0U);
}

TEST(MapTest, ClearAdvancesCoordinateEpoch) {
    Map map;
    map.clear();
    EXPECT_EQ(map.GetCoordinateEpoch(), 1U);
}

TEST(MapTest, ScaledRotationAdvancesCoordinateEpoch) {
    Map map;
    map.ApplyScaledRotation(autonomy::common::math::SE3f(), 1.0F);
    EXPECT_EQ(map.GetCoordinateEpoch(), 1U);
}

}  // namespace
}  // namespace autonomy::localization::atlas
