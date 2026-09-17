/*
 * Copyright 2026 The Openbot Authors
 *
 * Setup Assistant lite (MoveIt Setup Assistant subset, no Qt GUI).
 *
 * Generates manipulation conf + planner/kinematics sidecars + empty
 * *.convexparts templates from URDF/SRDF.
 */

#pragma once

#include <string>
#include <vector>

namespace autonomy {
namespace manipulation {
namespace setup {

/** @brief Inputs for lite setup generation. */
struct SetupRequest {
  std::string urdf_path;
  std::string srdf_path;
  std::string output_dir;  // e.g. conf/
  std::string planning_group = "arm";
  std::string base_frame = "base_link";
  std::string tip_frame = "tool0";
  std::string planner_id = "pilz_ptp";
  std::string collision_detector = "fcl";
  bool emit_convexparts_templates = true;
  bool emit_planner_confs = true;
  bool enable_online_decompose_default = false;
};

/** @brief Result paths written by GenerateSetup. */
struct SetupResult {
  bool success = false;
  std::string error;
  std::string manipulation_pb_txt;
  std::vector<std::string> conf_written;
  std::vector<std::string> convexparts_written;
  std::vector<std::string> mesh_paths_found;
};

/**
 * @brief Generate MoveIt-style runtime conf without Setup Assistant GUI.
 *
 * Writes `manipulation.pb.txt` plus optional ompl/chomp/stomp/kinematics/
 * controllers/planners/constraint_samplers templates under @p req.output_dir.
 * Does not launch RViz or interactive wizards.
 */
SetupResult GenerateSetup(const SetupRequest& req);

}  // namespace setup
}  // namespace manipulation
}  // namespace autonomy
