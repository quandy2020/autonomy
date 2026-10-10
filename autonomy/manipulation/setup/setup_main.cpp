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

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autonomy/common/cli_options.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/setup/setup_assistant_lite.hpp"

int main(int argc, char** argv) {
  CLI::App app{"autonomy.manipulation.setup"};
  std::string urdf;
  std::string srdf;
  std::string out = "share/autonomy/manipulation/conf";
  std::string group = "arm";
  std::string base = "base_link";
  std::string tip = "tool0";
  std::string planner = "pilz_ptp";
  std::string collision = "fcl";
  bool convexparts = true;

  app.add_option("--urdf", urdf, "Path to robot URDF")->required();
  app.add_option("--srdf", srdf, "Path to robot SRDF (optional)");
  app.add_option("--out", out, "Output directory for manipulation.pb.txt")
      ->capture_default_str();
  app.add_option("--group", group, "Planning group name")->capture_default_str();
  app.add_option("--base", base, "Base frame")->capture_default_str();
  app.add_option("--tip", tip, "Tip frame")->capture_default_str();
  app.add_option("--planner", planner, "Default planner_id")
      ->capture_default_str();
  app.add_option("--collision", collision, "Collision detector id")
      ->capture_default_str();
  app.add_option("--convexparts", convexparts,
                 "Emit empty *.convexparts templates")
      ->capture_default_str();
  autonomy::common::ParseOrExit(app, argc, argv);

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  autonomy::manipulation::setup::SetupRequest req;
  req.urdf_path = urdf;
  req.srdf_path = srdf;
  req.output_dir = out;
  req.planning_group = group;
  req.base_frame = base;
  req.tip_frame = tip;
  req.planner_id = planner;
  req.collision_detector = collision;
  req.emit_convexparts_templates = convexparts;

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
