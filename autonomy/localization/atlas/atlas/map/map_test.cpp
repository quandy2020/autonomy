#include "atlas/map/map.hpp"

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(MapTest, StartsEmpty) {
    Map map;
    EXPECT_EQ(map.KeyFramesInMap(), 0U);
    EXPECT_EQ(map.MapPointsInMap(), 0U);
}

}  // namespace
}  // namespace autonomy::localization::atlas
