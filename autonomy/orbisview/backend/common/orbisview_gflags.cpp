/*
 * Copyright 2026 The Openbot Authors
 *
 * OrbisView gflags definitions.
 */

#include "autonomy/orbisview/backend/common/orbisview_gflags.hpp"

DEFINE_string(host, "0.0.0.0", "Bind address (0.0.0.0 = all interfaces / LAN)");
DEFINE_uint32(port, 8766, "CivetWeb listening port");
DEFINE_bool(mock, true, "Enable mock visualization channels");
DEFINE_bool(autolink, false, "Enable Autolink channel discovery/subscribe");
DEFINE_string(document_root, "",
              "Optional static file root (e.g. frontend/dist)");
DEFINE_string(plugin_dir, "",
              "Optional directory of *.so/*.dylib plugins to scan at start");
DEFINE_string(cmd_vel_channel, "/cmd_vel",
              "Autolink channel for Twist2D teleop publish");
