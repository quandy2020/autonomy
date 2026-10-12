/*
 * Copyright 2026 The Openbot Authors
 *
 * Standalone manipulation process (MoveIt move_group analogue).
 */

#include <cstdlib>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "automanip/manipulation_options.hpp"
#include "automanip/manipulation_server.hpp"

namespace {

bool LoadOptions(automanip::ManipulationOptions* options) {
  using autonomy::common::FLAGS_conf;
  const std::string conf =
      FLAGS_conf.empty() ? "manipulation.pb.txt" : FLAGS_conf;
  return autonomy::common::LoadModuleConf("automanip", conf, options);
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"automanip.manipulation"};
  autonomy::common::BindCommonOptions(app);
  autonomy::common::ParseOrExit(app, argc, argv);
  FLAGS_alsologtostderr = true;

  autolink::Init(argv[0]);

  automanip::ManipulationOptions options;
  if (!LoadOptions(&options)) {
    AWARN << "manipulation_main: conf load failed; using defaults";
  }

  automanip::ManipulationServer server;
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
