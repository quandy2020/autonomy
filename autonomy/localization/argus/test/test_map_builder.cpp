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

#include <filesystem>
#include <iostream>

#include "argus/mapping/map_builder.hpp"
#include "argus/mapping/map_package.hpp"
#include "argus/relocalization/prior_map_loader.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::mapping::IsMapPackageDirectory;
  using argus::mapping::MapBuilder;
  using argus::mapping::MapBuilderConfig;
  using argus::mapping::MapBuilderKeyFrame;
  using argus::mapping::MapPackageMeta;
  using argus::mapping::kMapPackageVersion;
  using argus::relocalization::PriorMapLoader;
  using argus::sensor::PointType;

  MapBuilderConfig cfg;
  cfg.session_id = "S0";
  cfg.ikd_downsample_resolution = 0.5;
  cfg.prior_resolution = 0.5;
  MapBuilder builder(cfg);

  for (int i = 0; i < 5; ++i) {
    MapBuilderKeyFrame kf;
    kf.local_id = i;
    kf.stamp_sec = 0.1 * i;
    kf.T_w_b = transform::Rigid3d::Translation(Eigen::Vector3d(i * 2.0, 0, 0));
    for (int j = 0; j < 20; ++j) {
      PointType p;
      p.x = static_cast<float>(j * 0.2);
      p.y = 1.f;
      p.z = 0.f;
      kf.cloud.push_back(p);
    }
    builder.AddKeyFrame(kf);
  }

  if (builder.keyframe_count() != 5) {
    return Fail("kf count");
  }
  if (builder.map_point_count() == 0) {
    return Fail("map empty");
  }

  auto prior = builder.ExportPriorMap();
  if (!prior || prior->size() == 0) {
    return Fail("prior export");
  }

  builder.AddVisualLandmark(Eigen::Vector3d(1, 2, 3),
                            Eigen::Vector3d(0, 0, 1));
  builder.AddVisualLandmark(Eigen::Vector3d(4, 5, 6),
                            Eigen::Vector3d(1, 0, 0));
  if (builder.visual_map().LandmarkCount() != 2) {
    return Fail("visual landmark count");
  }

  const std::string dir = "argus_test_map_builder_out";
  std::filesystem::remove_all(dir);
  if (!builder.Save(dir)) {
    return Fail("save");
  }
  if (!IsMapPackageDirectory(dir)) {
    return Fail("package dir detect");
  }
  for (const char* name :
       {"map_meta.txt", "session.txt", "map.xyz", "prior.xyz", "visual.xyz",
        "trajectory.tum"}) {
    if (!std::filesystem::exists(dir + "/" + name)) {
      return Fail(name);
    }
  }
  if (builder.package_meta().version != kMapPackageVersion) {
    return Fail("package version");
  }

  // Reloc loader prefers prior.xyz inside package.
  auto prior_loaded = PriorMapLoader::LoadPath(dir, 0.5);
  if (!prior_loaded || prior_loaded->size() == 0) {
    return Fail("LoadPath prior from package");
  }

  MapBuilder loaded(cfg);
  if (!loaded.Load(dir)) {
    return Fail("load");
  }
  if (loaded.keyframe_count() != 5) {
    return Fail("loaded kf count");
  }
  if (loaded.map_point_count() == 0) {
    return Fail("loaded map empty");
  }
  if (loaded.visual_map().LandmarkCount() != 2) {
    return Fail("loaded visual landmarks");
  }
  if (loaded.package_meta().session_id != "S0") {
    return Fail("loaded meta session_id");
  }

  // Re-save after load must keep full cloud (ExportPoints, not empty kf clouds).
  const std::string dir2 = "argus_test_map_builder_out2";
  std::filesystem::remove_all(dir2);
  if (!loaded.Save(dir2)) {
    return Fail("re-save");
  }
  MapBuilder loaded2(cfg);
  if (!loaded2.Load(dir2)) {
    return Fail("re-load");
  }
  if (loaded2.map_point_count() == 0) {
    return Fail("re-load map empty");
  }
  if (builder.package_meta().map_xyz_fnv64 == 0) {
    return Fail("map_xyz_fnv64 unset");
  }
  std::string verr;
  if (!argus::mapping::VerifyMapPackage(dir, builder.package_meta(), &verr)) {
    return Fail(verr.empty() ? "verify package" : verr.c_str());
  }

  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(dir2);

  std::cout << "OK map_builder (pts=" << builder.map_point_count()
            << " prior=" << prior->size()
            << " visual=" << builder.visual_map().LandmarkCount()
            << " pkg=v" << kMapPackageVersion << ")\n";
  return 0;
}
