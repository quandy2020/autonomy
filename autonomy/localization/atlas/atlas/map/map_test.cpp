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

TEST(MapTest, ReadsCoordinateEpochAndImuReadinessTogether) {
    Map map;
    std::uint64_t coordinate_epoch = 99;
    bool imu_initialized = true;
    ASSERT_TRUE(
        map.GetCoordinateState(&coordinate_epoch, &imu_initialized));
    EXPECT_EQ(coordinate_epoch, 0U);
    EXPECT_FALSE(imu_initialized);

    map.ApplyScaledRotation(autonomy::common::math::SE3f(), 1.0F);
    map.SetImuInitialized();
    ASSERT_TRUE(
        map.GetCoordinateState(&coordinate_epoch, &imu_initialized));
    EXPECT_EQ(coordinate_epoch, 1U);
    EXPECT_TRUE(imu_initialized);
    EXPECT_FALSE(map.GetCoordinateState(nullptr, &imu_initialized));
    EXPECT_FALSE(map.GetCoordinateState(&coordinate_epoch, nullptr));
}

}  // namespace
}  // namespace autonomy::localization::atlas
