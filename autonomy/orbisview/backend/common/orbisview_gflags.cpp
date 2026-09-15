/*
 * Copyright 2026 The Openbot Authors
 *
 * OrbisView gflags definitions.
 */

#include "autonomy/orbisview/backend/common/orbisview_gflags.hpp"

DEFINE_string(host, "0.0.0.0", "Bind address (0.0.0.0 = all interfaces / LAN)");
DEFINE_uint32(port, 8766, "CivetWeb listening port");
DEFINE_bool(mock, false, "Enable mock visualization channels");
DEFINE_bool(autolink, true, "Enable Autolink channel discovery/subscribe");
DEFINE_string(document_root, "",
              "Optional static file root (e.g. frontend/dist)");
DEFINE_string(plugin_dir, "",
              "Optional directory of *.so/*.dylib plugins to scan at start");
DEFINE_string(cmd_vel_channel, "/cmd_vel",
              "Autolink channel for TwistStamped teleop publish (autosim-compatible)");
DEFINE_string(goal_pose_channel, "/goal_pose",
              "Autolink PoseStamped channel for single-pose navigation goals");
DEFINE_string(goal_poses_channel, "/goal_poses",
              "Autolink PoseStampedArray channel for multi-pose navigation routes");
DEFINE_string(cancel_navigation_channel, "/cancel_navigation",
              "Autolink Bool channel to cancel the active navigation task");
