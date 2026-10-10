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
#include <string>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/task/task_server.hpp"

namespace autonomy::task {
namespace {

struct TaskCli {
    std::string config_directory;
    uint32_t feedback_period_ms = 100;
    bool exclusive_navigation_tasks = true;
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
    app.add_option("--config_directory", cli.config_directory,
                   "Task conf root (default: resolve autonomy/task/conf via "
                   "AUTONOMY_PATH). Contains behavior_tree/ XML.");
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
    if (!server->Start()) {
        LOG(ERROR) << "TaskServer start failed";
        return EXIT_FAILURE;
    }

    LOG(INFO) << "task_main running";
    autolink::WaitForShutdown();

    server->Shutdown();
    return EXIT_SUCCESS;
}
