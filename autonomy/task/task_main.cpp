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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/task/task_server.hpp"

DEFINE_string(config_directory, "config",
              "Root directory for task behavior tree XML files.");
DEFINE_uint32(feedback_period_ms, 100,
              "Scheduler feedback polling period in milliseconds.");
DEFINE_bool(exclusive_navigation_tasks, true,
            "Only one navigation-class task at a time.");

namespace autonomy::task {
namespace {

::autonomy::task::proto::TaskServerOptions BuildOptions()
{
    auto options = TaskServer::DefaultOptions();
    options.set_config_directory(FLAGS_config_directory);
    options.mutable_scheduler()->set_feedback_period_ms(FLAGS_feedback_period_ms);
    options.mutable_scheduler()->set_exclusive_navigation_tasks(
        FLAGS_exclusive_navigation_tasks);
    return options;
}

}  // namespace
}  // namespace autonomy::task

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    auto server = std::make_shared<autonomy::task::TaskServer>();
    if (!server->Configure(autonomy::task::BuildOptions())) {
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
