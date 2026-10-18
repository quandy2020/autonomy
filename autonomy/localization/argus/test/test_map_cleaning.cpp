/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <unordered_map>

#include "argus/map/map_confidence.hpp"
#include "argus/map/map_self_cleaning.hpp"
#include "argus/map/voxel_map.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::map::BumpMapConfidence;
  using argus::map::MapPointConfidence;
  using argus::map::MapSelfCleaning;
  using argus::map::MapSelfCleaningConfig;
  using argus::map::ShouldPruneVoxel;
  using argus::map::VoxelLocation;
  using argus::map::VoxelMapConfig;
  using argus::map::VoxelMapManager;
  using argus::map::VoxelOctoTree;

  MapPointConfidence weak;
  for (int i = 0; i < 4; ++i) {
    BumpMapConfidence(&weak, true);
  }
  if (!ShouldPruneVoxel(weak, 0.3f, 5)) {
    return Fail("weak voxel should prune");
  }
  MapPointConfidence anchor;
  for (int i = 0; i < 10; ++i) {
    BumpMapConfidence(&anchor, false);
  }
  anchor.confidence = 0.1f;
  if (ShouldPruneVoxel(anchor, 0.3f, 5)) {
    return Fail("high-obs anchor must be kept");
  }

  std::unordered_map<VoxelLocation, VoxelOctoTree*> hash;
  VoxelMapConfig vcfg;
  VoxelMapManager mgr(vcfg, hash);

  auto* weak_tree = new VoxelOctoTree(0, 0, 1, 50, 0.01f);
  for (int i = 0; i < 3; ++i) {
    BumpMapConfidence(&weak_tree->confidence_, true);
  }
  hash[VoxelLocation(0, 0, 0)] = weak_tree;

  auto* strong_tree = new VoxelOctoTree(0, 0, 1, 50, 0.01f);
  for (int i = 0; i < 10; ++i) {
    BumpMapConfidence(&strong_tree->confidence_, false);
  }
  strong_tree->confidence_.confidence = 0.2f;  // dipped but high obs
  hash[VoxelLocation(5, 0, 0)] = strong_tree;

  MapSelfCleaningConfig ccfg;
  ccfg.enable = true;
  ccfg.confidence_threshold = 0.35f;
  ccfg.min_observation_count_to_keep = 8;
  MapSelfCleaning cleaner(ccfg);

  const int erased = cleaner.Prune(mgr, ccfg.confidence_threshold);
  if (erased != 1) {
    return Fail("expected exactly one pruned voxel");
  }
  if (hash.count(VoxelLocation(0, 0, 0)) != 0) {
    return Fail("weak voxel still present");
  }
  if (hash.count(VoxelLocation(5, 0, 0)) == 0) {
    return Fail("strong voxel missing");
  }

  delete hash[VoxelLocation(5, 0, 0)];
  hash.clear();

  std::cout << "OK map_cleaning (erased=" << erased << ")\n";
  return 0;
}
