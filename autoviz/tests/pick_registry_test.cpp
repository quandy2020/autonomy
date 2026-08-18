/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/pick_registry.hpp"

#include <gtest/gtest.h>

namespace autoviz {
namespace common {
namespace {

TEST(PickRegistry, RangeLookupByHandleAndIndex) {
  PickRegistry registry;
  PickRangeRecord range;
  range.display_name = "map_points";
  range.display_type = "PointCloud2";
  range.positions = {QVector3D(1.f, 0.f, 0.f), QVector3D(2.f, 0.f, 0.f),
                     QVector3D(3.f, 0.f, 0.f)};
  const PickHandle base = registry.registerPickRange(std::move(range));
  ASSERT_NE(base, kInvalidPickHandle);

  const PickRecord* first = registry.lookup(base);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->display_name, "map_points");
  EXPECT_EQ(first->point_index, 0);
  EXPECT_FLOAT_EQ(first->position.x(), 1.f);

  const PickRecord* last = registry.lookup(base + 2);
  ASSERT_NE(last, nullptr);
  EXPECT_EQ(last->point_index, 2);
  EXPECT_FLOAT_EQ(last->position.x(), 3.f);

  EXPECT_EQ(registry.lookup(base + 3), nullptr);
  EXPECT_EQ(registry.lookupByDisplayAndPointIndex("map_points", 1), base + 1);
  EXPECT_EQ(registry.lookupByDisplayAndPointIndex("map_points", 9),
            kInvalidPickHandle);
}

TEST(PickHandleAllocator, RangeStaysContiguous) {
  PickHandleAllocator allocator;
  const PickHandle a = allocator.allocate();
  const PickHandle base = allocator.allocateRange(4);
  EXPECT_EQ(base, a + 1);
  EXPECT_EQ(allocator.allocate(), base + 4);
}

}  // namespace
}  // namespace common
}  // namespace autoviz
