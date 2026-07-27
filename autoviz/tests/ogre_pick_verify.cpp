/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Headless checks for Ogre pick handle resolution (no GUI required).
 *****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <string>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/pick_registry.hpp"

namespace {

int expectTrue(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    return 1;
  }
  std::cout << "PASS: " << message << '\n';
  return 0;
}

}  // namespace

int main() {
  int failures = 0;

  const autoviz::common::PickHandle handle = 0x010203;
  const autoviz::common::PickColor color =
      autoviz::common::handleToPickColor(handle);
  failures += expectTrue(
      color.r == 0x01 && color.g == 0x02 && color.b == 0x03,
      "handleToPickColor encodes RGB");
  failures += expectTrue(
      autoviz::common::pickColorToHandle(color.r, color.g, color.b) == handle,
      "pickColorToHandle roundtrip");

  autoviz::common::PickRegistry registry;
  autoviz::common::PickRecord cloud_record;
  cloud_record.display_name = "TestCloud";
  cloud_record.display_type = "PointCloud2";
  cloud_record.point_index = -1;
  const autoviz::common::PickHandle cloud_handle =
      registry.registerPick(cloud_record);

  autoviz::common::PickRecord point_record;
  point_record.display_name = "TestCloud";
  point_record.display_type = "PointCloud2";
  point_record.point_index = 3;
  const autoviz::common::PickHandle point_handle =
      registry.registerPick(point_record);

  failures += expectTrue(
      registry.lookupByDisplayAndPointIndex("TestCloud", 3) == point_handle,
      "lookupByDisplayAndPointIndex resolves per-point handle");
  failures += expectTrue(
      registry.lookupByDisplayAndPointIndex("TestCloud", 99) ==
          autoviz::common::kInvalidPickHandle,
      "lookupByDisplayAndPointIndex misses unknown index");
  failures += expectTrue(registry.lookup(cloud_handle) != nullptr,
                         "cloud-level pick record stored");

  if (failures != 0) {
    std::cerr << failures << " check(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All ogre pick verify checks passed.\n";
  return EXIT_SUCCESS;
}
