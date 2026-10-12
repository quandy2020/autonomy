/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file main.cpp
 * @brief Process entry: CLI → ManipulationServer (MoveIt move_group analogue).
 *
 * Usage: automanip [options]
 * Planning conf: share/automanip/conf/manipulation.pb.txt (or --conf).
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
#include "options.hpp"

namespace {

bool LoadOptions(automanip::ManipulationOptions* options) {
  using autonomy::common::FLAGS_conf;
  const std::string conf =
      FLAGS_conf.empty() ? "manipulation.pb.txt" : FLAGS_conf;
  return autonomy::common::LoadModuleConf("automanip", conf, options);
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"automanip"};
  autonomy::common::BindCommonOptions(app);
  bool dry_run = false;
  app.add_flag("-n,--dry-run", dry_run, "Load conf and exit");
  autonomy::common::ParseOrExit(app, argc, argv);
  FLAGS_alsologtostderr = true;

  autolink::Init(argv[0]);
  AINFO << automanip::VersionString();

  automanip::ManipulationOptions options;
  if (!LoadOptions(&options)) {
    AWARN << "automanip: conf load failed; using defaults";
  }
  if (dry_run) {
    AINFO << "dry-run: action=" << options.action_name()
          << " group=" << options.planning_group()
          << " planner=" << options.planner_id();
    autolink::Clear();
    return EXIT_SUCCESS;
  }

  automanip::ManipulationServer server;
  if (!server.Init(options)) {
    AERROR << "automanip: Init failed";
    return EXIT_FAILURE;
  }
  if (!server.Start()) {
    AERROR << "automanip: Start failed";
    return EXIT_FAILURE;
  }

  AINFO << "automanip running";
  autolink::WaitForShutdown();
  server.Stop();
  autolink::Clear();
  return EXIT_SUCCESS;
}
