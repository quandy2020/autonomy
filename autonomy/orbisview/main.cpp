/*
 * Copyright 2026 The Openbot Authors
 *
 * OrbisView entry (Dreamview main.cpp counterpart).
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <glog/logging.h>

#include "autolink/init.hpp"
#include "autonomy/orbisview/backend/common/orbisview_gflags.hpp"
#include "autonomy/orbisview/backend/orbisview.hpp"

#if defined(ORBISVIEW_WITH_AUTOLINK)
#include "autolink/autolink.hpp"
#endif

namespace {
std::atomic<bool> g_running{true};
void OnSignal(int) { g_running = false; }
}  // namespace

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

#if defined(ORBISVIEW_WITH_AUTOLINK)
  if (FLAGS_autolink) {
    if (!autolink::Init(argv[0])) {
      LOG(ERROR) << "autolink::Init failed";
      return EXIT_FAILURE;
    }
  }
#endif

  autonomy::orbisview::backend::ServerOptions options;
  options.host = FLAGS_host;
  options.port = static_cast<uint16_t>(FLAGS_port);
  options.enable_mock = FLAGS_mock;
  options.enable_autolink = FLAGS_autolink;
  options.document_root = FLAGS_document_root;
  options.plugin_dir = FLAGS_plugin_dir;
  options.cmd_vel_channel = FLAGS_cmd_vel_channel;
  options.goal_pose_channel = FLAGS_goal_pose_channel;
  options.goal_poses_channel = FLAGS_goal_poses_channel;
  options.cancel_navigation_channel = FLAGS_cancel_navigation_channel;
  options.hmi_modes_dir = "autonomy/orbisview/conf/hmi_modes";

  autonomy::orbisview::backend::Orbisview orbisview;
  if (!orbisview.Init(options) || !orbisview.Start()) {
    return EXIT_FAILURE;
  }

  LOG(INFO) << "OrbisView running. WebSocket: ws://" << options.host << ':'
            << options.port << "/ws";
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  orbisview.Stop();
#if defined(ORBISVIEW_WITH_AUTOLINK)
  if (FLAGS_autolink) {
    autolink::Clear();
  }
#endif
  return EXIT_SUCCESS;
}
