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

#include "autonomy/tools/god_viewer/viewer_bridge.hpp"

#include "absl/strings/str_cat.h"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/tools/god_viewer/channel/channel_example_handler.hpp"
#include "autonomy/tools/god_viewer/channel/channel_path_handler.hpp"
#include "autonomy/tools/god_viewer/channel/channel_point_cloud_hander.hpp"
#include "autonomy/tools/god_viewer/channel/channel_image_handler.hpp"
#include "autonomy/tools/god_viewer/channel/channel_ccupancy_grid_hander.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 

ViewerBridge::ViewerBridge(
    const std::string& configration_directory,
    const std::string& configration_basename)
{
    thread_pool_ = std::make_shared<common::ThreadPool>(4);
    bool init_finihed = LoadOptions(configration_directory, configration_basename);
    if (init_finihed) {
        LOG(INFO) << "Foxglove bridge god viewer init successfully !";
    }
}  

void ViewerBridge::Run()
{
    auto channel0 = std::make_shared<channel::LogExampleHandler>("/hello");
    auto channel1 = std::make_shared<channel::PathHandler>(server_handler_, "/cube");
    auto channel2 = std::make_shared<channel::PathHandler>(server_handler_, "/path");
    auto channel3 = std::make_shared<channel::OccupancyGridHandler>(server_handler_, "/map");
    auto channel4 = std::make_shared<channel::PointCloudHandler>(server_handler_, "/point_cloud");
    auto channel5 = std::make_shared<channel::ImageHandler>(server_handler_, "/image0");

    while (true)
    {
        channel0->Send();
        channel1->Send();
        channel2->Send(channel2->GenerateCircularPath(5.0));
        channel3->SendTest();
        channel4->SendTest();
        channel5->SendTest();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void ViewerBridge::ShutDown()
{
    server_handler_->server->stop();
}

bool ViewerBridge::LoadOptions(
    const std::string& configuration_directory, 
    const std::string& configuration_basename)
{
    auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(
        std::vector<std::string>{configuration_directory});
    const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
    ::autonomy::common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    server_handler_ = CreateFoxgloveViewerOptions(&lua_parameter_dictionary);
    if (server_handler_  == nullptr) {
        LOG(ERROR) << "Load foxglove viewer option error, init server_handler is nullptr";
        return false;
    }
    return true;
}

}   // namespace god_viewer
}   // namespace tools
}   // namespace autonomy