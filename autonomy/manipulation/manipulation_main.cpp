/*
 * Copyright 2026 The Openbot Authors
 *
 * Standalone manipulation process (MoveIt move_group analogue).
 */

#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/manipulation_options.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace {

bool LoadOptions(autonomy::manipulation::ManipulationOptions* options) {
  using autonomy::common::FLAGS_conf;
  const std::string conf =
      FLAGS_conf.empty() ? "manipulation.pb.txt" : FLAGS_conf;
  return autonomy::common::LoadModuleConf("manipulation", conf, options);
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;

  autolink::Init(argv[0]);

  autonomy::manipulation::ManipulationOptions options;
  if (!LoadOptions(&options)) {
    AWARN << "manipulation_main: conf load failed; using defaults";
  }

  autonomy::manipulation::ManipulationServer server;
  if (!server.Init(options)) {
    AERROR << "manipulation_main: Init failed";
    return EXIT_FAILURE;
  }
  if (!server.Start()) {
    AERROR << "manipulation_main: Start failed";
    return EXIT_FAILURE;
  }

  AINFO << "manipulation_main: running";
  autolink::WaitForShutdown();
  server.Stop();
  return EXIT_SUCCESS;
}
