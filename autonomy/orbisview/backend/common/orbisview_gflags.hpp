/*
 * Copyright 2026 The Openbot Authors
 *
 * OrbisView gflags (Dreamview backend/common/dreamview_gflags counterpart).
 */

#pragma once

#include "gflags/gflags.h"

DECLARE_string(host);
DECLARE_uint32(port);
DECLARE_bool(mock);
DECLARE_bool(autolink);
DECLARE_string(document_root);
DECLARE_string(plugin_dir);
DECLARE_string(cmd_vel_channel);
DECLARE_string(goal_pose_channel);
DECLARE_string(goal_poses_channel);
DECLARE_string(cancel_navigation_channel);
