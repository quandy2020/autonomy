/*
 * Copyright 2026 The Openbot Authors
 *
 * CLI: autonomy.manipulation.setup — GenerateSetup without Qt Setup Assistant.
 *
 * Usage:
 *   autonomy.manipulation.setup --urdf=arm.urdf --srdf=arm.srdf --out=conf/
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/setup/setup_assistant_lite.hpp"

DEFINE_string(urdf, "", "Path to robot URDF");
DEFINE_string(srdf, "", "Path to robot SRDF (optional)");
DEFINE_string(out, "share/autonomy/manipulation/conf",
              "Output directory for manipulation.pb.txt");
DEFINE_string(group, "arm", "Planning group name");
DEFINE_string(base, "base_link", "Base frame");
DEFINE_string(tip, "tool0", "Tip frame");
DEFINE_string(planner, "pilz_ptp", "Default planner_id");
DEFINE_string(collision, "fcl", "Collision detector id");
DEFINE_bool(convexparts, true, "Emit empty *.convexparts templates");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;

  if (FLAGS_urdf.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " --urdf=robot.urdf [--srdf=robot.srdf] [--out=conf/]\n";
    return 1;
  }

  autonomy::manipulation::setup::SetupRequest req;
  req.urdf_path = FLAGS_urdf;
  req.srdf_path = FLAGS_srdf;
  req.output_dir = FLAGS_out;
  req.planning_group = FLAGS_group;
  req.base_frame = FLAGS_base;
  req.tip_frame = FLAGS_tip;
  req.planner_id = FLAGS_planner;
  req.collision_detector = FLAGS_collision;
  req.emit_convexparts_templates = FLAGS_convexparts;

  const auto result = autonomy::manipulation::setup::GenerateSetup(req);
  if (!result.success) {
    AERROR << "setup failed: " << result.error;
    return 2;
  }
  AINFO << "wrote " << result.manipulation_pb_txt;
  AINFO << "meshes=" << result.mesh_paths_found.size()
        << " convexparts_templates=" << result.convexparts_written.size();
  return 0;
}
