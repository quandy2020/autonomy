/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <signal.h>

#include "gflags/gflags.h"
#include "autonomy/common/version.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tools/god_viewer/viewer_flags.hpp"
#include "autonomy/tools/god_viewer/viewer_bridge.hpp"

namespace autonomy {
namespace tools {
namespace god_viewer {
namespace {

void SigintHandler(int sig)
{
    LOG(INFO) << "Shutdown autonomy system.";
    exit(EXIT_SUCCESS);
}

void Run()
{
    // 'Crtl + C' sign handler
    signal(SIGINT, SigintHandler);

    // Show autonomu app version
    autonomy::common::ShowVersion();
    LOG(INFO) << "Autonomy open robot for everyone enjoy !!!";

    // Run foxglove viewer for show msgs
    auto viewer = std::make_shared<ViewerBridge>(
        FLAGS_foxglove_config_directory, FLAGS_foxglove_config_basename);
    viewer->Run();
}

} // namespace 
} // namespace god_viewer
} // namespace tools
} // namespace autonomy

int main(int argc, char **argv)
{
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;  
	google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!autonomy::tools::god_viewer::FLAGS_foxglove_config_directory.empty())
      << "-foxglove_config_directory is missing.";
    CHECK(!autonomy::tools::god_viewer::FLAGS_foxglove_config_basename.empty())
      << "-foxglove_config_basename is missing.";

    ::autonomy::tools::god_viewer::Run();
    google::ShutdownGoogleLogging();  
    return EXIT_SUCCESS;
}

