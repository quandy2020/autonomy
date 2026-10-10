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
 * @brief Process entry: CLI → LoadConfig → ArmManager.
 *
 * Usage: automanip [options] [configuration_directory] [configuration_file]
 */

#include <atomic>
#include <csignal>
#include <string>

#include "options.hpp"

#include "arm/arm_manager.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/init.hpp"
#include "autolink/time/duration.hpp"
#include "automanip/config_loader.hpp"

namespace {
std::atomic<bool> g_running{true};

void HandleSignal(int) { g_running = false; }

int Run(const automanip::Options& opts) {
  automanip::Config config = automanip::LoadConfig(opts.config_directory,
                                                   opts.config_file);
  if (opts.dry_run) {
    AINFO << "dry-run: node_name=" << config.node_name
          << " arm.enable=" << config.arm.enable
          << " backend=" << config.arm.backend
          << " dof=" << config.arm.plant.chain.dof();
    return 0;
  }
  auto node = autolink::CreateNode(config.node_name);
  if (!node) {
    AERROR << "autolink node failed";
    return 1;
  }
  automanip::arm::ArmManager arm;
  if (!arm.Start(node.get(), config)) {
    AERROR << "ArmManager failed";
    return 1;
  }
  AINFO << "automanip running (Ctrl+C to stop)";
  while (g_running.load()) {
    autolink::Duration(100'000'000).Sleep();
  }
  AINFO << "automanip shutting down";
  arm.Stop();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  automanip::Options opts;
  const automanip::ParseStatus status =
      automanip::ParseCommandLine(argc, argv, &opts);
  if (status == automanip::ParseStatus::kExitOk) {
    return 0;
  }
  if (status == automanip::ParseStatus::kExitError) {
    return 1;
  }

  autolink::Init(argv[0]);
  AINFO << automanip::VersionString();
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  int exit_code = 1;
  try {
    exit_code = Run(opts);
  } catch (const std::exception& ex) {
    AERROR << "automanip failed: " << ex.what();
    exit_code = 1;
  }
  autolink::Clear();
  return exit_code;
}
