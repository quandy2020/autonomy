/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>
#include <cstdint>
#include <chrono>
#include <stdexcept>
#include <string>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/task/task_server.hpp"
#ifdef AUTONOMY_BUILD_COVERAGE
#include "autonomy/task/navigation/coverage/runtime.hpp"
#include "autonomy/task/navigation/coverage/configuration_loader.hpp"
#endif
#include "autonomy/task/exploration/integration/native_rgbd_platform.hpp"

namespace autonomy::task {
namespace {

struct TaskCli {
    std::string config_directory;
    std::string native_rgbd_config;  ///< Explicit actual-calibration platform configuration; empty preserves ordinary startup.
    uint32_t feedback_period_ms = 100;
    bool exclusive_navigation_tasks = true;
    bool enable_coverage = false;
    std::string coverage_config;
};

::autonomy::task::proto::TaskServerOptions BuildOptions(const TaskCli& cli)
{
    auto options = TaskServer::DefaultOptions();
    // Empty flag must not wipe the path resolved by BtDefaults::Apply.
    if (!cli.config_directory.empty()) {
        options.set_config_directory(cli.config_directory);
    }
    options.mutable_scheduler()->set_feedback_period_ms(cli.feedback_period_ms);
    options.mutable_scheduler()->set_exclusive_navigation_tasks(
        cli.exclusive_navigation_tasks);
    return options;
}

}  // namespace
}  // namespace autonomy::task

int main(int argc, char** argv)
{
    CLI::App app{"autonomy.task"};
    autonomy::task::TaskCli cli;
    app.add_option("--coverage_config", cli.coverage_config, "Complete coverage runtime JSON configuration.");
    app.add_flag("--enable_coverage", cli.enable_coverage, "Enable native complete coverage planner and navigator actions.");
    app.add_option("--config_directory", cli.config_directory,
                   "Task conf root (default: resolve autonomy/task/conf via "
                   "AUTONOMY_PATH). Contains behavior_tree/ XML.");
    app.add_option("--native_rgbd_config",cli.native_rgbd_config,
                   "Install real Atlas/RGB-D exploration providers from explicit native calibration and YAML settings.");
    app.add_option("--feedback_period_ms", cli.feedback_period_ms,
                   "Scheduler feedback polling period in milliseconds.")
        ->capture_default_str();
    app.add_option("--exclusive_navigation_tasks",
                   cli.exclusive_navigation_tasks,
                   "Only one navigation-class task at a time.")
        ->capture_default_str();
    autonomy::common::ParseOrExit(app, argc, argv);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    auto server = std::make_shared<autonomy::task::TaskServer>();
    if (!server->Configure(autonomy::task::BuildOptions(cli))) {
        LOG(ERROR) << "TaskServer configure failed";
        return EXIT_FAILURE;
    }
    std::shared_ptr<autonomy::task::exploration::NativeRgbdPlatform> native_rgbd;  ///< Own actual providers through task shutdown.
    if (!cli.native_rgbd_config.empty()) {
#ifdef AUTONOMY_TASK_HAS_NATIVE_ATLAS_MAPPING
        try {
            auto options = autonomy::task::exploration::LoadNativeRgbdPlatformOptions(cli.native_rgbd_config);
            if (!server->navigation() || !server->navigation()->SetFrameNames(
                    options.localization.navigation_frame,options.navigation_base_frame))
                throw std::runtime_error("native navigation frame configuration requires its idle actual task");
            native_rgbd = std::make_shared<autonomy::task::exploration::NativeRgbdPlatform>(
                server->exploration(),server->node(),std::move(options));
            if (!native_rgbd->Install()) throw std::runtime_error("native RGB-D platform installation failed");
        } catch (const std::exception& error) {
            LOG(ERROR) << "Native RGB-D exploration startup failed: " << error.what();
            server->Shutdown();
            if (native_rgbd) { native_rgbd->Shutdown(); native_rgbd->WaitForShutdown(std::chrono::seconds(5)); }
            return EXIT_FAILURE;
        }
#else
        LOG(ERROR) << "Native RGB-D exploration requires the actual enabled Atlas mapping domain";
        return EXIT_FAILURE;
#endif
    }
#ifdef AUTONOMY_BUILD_COVERAGE
    std::shared_ptr<autonomy::task::coverage::CoverageRuntime> coverage_runtime;
    if (cli.enable_coverage) {
        coverage_runtime = std::make_shared<autonomy::task::coverage::CoverageRuntime>(
            server->node(), server->scheduler(), server->navigation_client(),
            autonomy::task::coverage::LoadConfiguration(cli.coverage_config));
    }
#else
    if (cli.enable_coverage) {
        LOG(ERROR) << "Coverage navigation requires AUTONOMY_BUILD_COVERAGE=ON";
        return EXIT_FAILURE;
    }
#endif
    if (!server->Start()) {
        LOG(ERROR) << "TaskServer start failed";
        server->Shutdown();
        if (native_rgbd) { native_rgbd->Shutdown(); native_rgbd->WaitForShutdown(std::chrono::seconds(5)); }
        return EXIT_FAILURE;
    }

    LOG(INFO) << "task_main running";
    autolink::WaitForShutdown();

#ifdef AUTONOMY_BUILD_COVERAGE
    if (coverage_runtime) coverage_runtime->Shutdown();
#endif
    server->Shutdown();
    if (native_rgbd) {
        native_rgbd->Shutdown();
        if (!native_rgbd->WaitForShutdown(std::chrono::seconds(5))) {
            LOG(ERROR) << "Native RGB-D provider retirement did not complete";
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}
