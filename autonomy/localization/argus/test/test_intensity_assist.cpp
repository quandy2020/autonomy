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

#include "argus/fusion/intensity_assist.hpp"
#include "argus/loop_closure/multi_modal_consensus.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::fusion::IntensityAssist;
  using argus::fusion::IntensityAssistConfig;
  using argus::loop_closure::LoopCandidate;
  using argus::loop_closure::MultiModalConsensus;
  using argus::loop_closure::MultiModalConsensusConfig;
  using argus::perception::DegeneracyResult;
  using argus::sensor::PointCloudXYZI;
  using argus::sensor::PointType;

  PointCloudXYZI textured;
  for (int i = 0; i < 40; ++i) {
    PointType p;
    p.x = static_cast<float>(i);
    p.y = 0.f;
    p.z = 0.f;
    p.intensity = static_cast<float>((i % 2) * 40);
    textured.push_back(p);
  }
  PointCloudXYZI flat;
  for (int i = 0; i < 40; ++i) {
    PointType p;
    p.intensity = 10.f;
    flat.push_back(p);
  }

  IntensityAssistConfig config;
  config.enable = true;
  config.min_intensity_std_dev = 5.0;
  config.inflation_relief = 0.5;
  IntensityAssist assist(config);
  DegeneracyResult degeneracy;
  degeneracy.is_degenerated = true;

  const auto ok = assist.Evaluate(degeneracy, textured);
  if (!ok.available || ok.inflation_multiplier >= 0.99) {
    return Fail("textured degeneracy should relieve inflation");
  }
  const auto no = assist.Evaluate(degeneracy, flat);
  if (no.available) {
    return Fail("flat intensity should not assist");
  }

  MultiModalConsensusConfig mcfg;
  mcfg.min_id_gap = 2;
  mcfg.min_path_separation_m = 1.0;
  mcfg.min_confidence = 0.4;
  mcfg.pose_consistency_weight = 0.2;
  MultiModalConsensus cons(mcfg);
  LoopCandidate good;
  good.query_id = 20;
  good.match_id = 0;
  good.score = 0.9;
  good.T_query_match = transform::Rigid3d::Identity();
  double conf_good = 0.0;
  if (!cons.AcceptCandidate(good, 50.0, &conf_good, 0.8)) {
    return Fail("good candidate rejected");
  }
  LoopCandidate bad_yaw = good;
  bad_yaw.T_query_match = transform::Rigid3d::Rotation(
      Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()));
  double conf_bad = 0.0;
  cons.AcceptCandidate(bad_yaw, 50.0, &conf_bad, 0.8);
  if (!(conf_bad < conf_good)) {
    return Fail("pose consistency should lower confidence");
  }

  std::cout << "OK intensity_assist (relief=" << ok.inflation_multiplier
            << " conf " << conf_good << "->" << conf_bad << ")\n";
  return 0;
}
