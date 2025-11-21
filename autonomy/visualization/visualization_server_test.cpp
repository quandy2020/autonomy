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

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "autonomy/localization/utils/image_loader.hpp"
#include "autonomy/visualization/visualization_server.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/random.hpp"
#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/map/costmap_2d/map_mode.hpp"
namespace autonomy {
namespace visualization { 

VisualizationServer::SharedPtr GetServer()
{
    proto::VisualizationOptions options;
    options.set_host("0.0.0.0");
    options.set_port(8765);
    options.set_mcap_filename("");
    options.set_write_mcap_data(false);
    return std::make_shared<VisualizationServer>(options);
}

std::vector<cv::Mat> CreateImageMsgs()
{
    std::vector<cv::Mat> images;
    std::string image_path = "/workspace/autonomy/build";
    localization::utils::ReadImage(image_path, images);
    return images;
}

commsgs::planning_msgs::Path CreatePathMsgs()
{
    commsgs::planning_msgs::Path msgs;
    msgs.header.frame_id = "map";

    const float CENTER_X = 0.5f;
    const float CENTER_Y = 0.0f;
    const float RADIUS = 2.8f;
    const int NUM_POINTS = 100;
    for (int i = 0; i < NUM_POINTS; ++i) {

        commsgs::geometry_msgs::PoseStamped pose;
        float angle = 2.0f * M_PI * i / NUM_POINTS;
        pose.pose.position.x = CENTER_X + RADIUS * cos(angle);
        pose.pose.position.y = CENTER_Y + RADIUS * sin(angle);
        pose.pose.position.z = 0.0;
        msgs.poses.push_back(pose);
    }
    return msgs;
}


commsgs::map_msgs::OccupancyGrid CreateMapMsgs()
{
    std::string yaml_file = "/workspace/autonomy/configuration_files/map/turtlebot3_house.yaml";
    LOG(INFO) << "map_file: " << yaml_file;

    commsgs::map_msgs::OccupancyGrid map_data;
    if (map::costmap_2d::loadMapFromYaml(yaml_file, map_data) != map::costmap_2d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        LOG(ERROR) << "Load yaml file error.";
        return map_data;
    }

    return map_data;
}

commsgs::geometry_msgs::Point CreatePointMsgs()
{
    commsgs::geometry_msgs::Point msgs;
    msgs.x = autonomy::common::math::RandomUniformInteger(0, 100);
    msgs.y = autonomy::common::math::RandomUniformInteger(0, 50);
    msgs.z = autonomy::common::math::RandomUniformInteger(0, 60);
    return msgs;
}


TEST(VisualizationServer, TestPublish) 
{
    auto server = GetServer();
    auto path = CreatePathMsgs();
    auto image = CreateImageMsgs();
    auto map = CreateMapMsgs();


    while (true) {
        server->Publish<commsgs::planning_msgs::Path>("path", std::move(path));
        // server->Publish<cv::Mat>("image", std::move(image[0]));
        server->Publish<commsgs::map_msgs::OccupancyGrid>("map", std::move(map));

        auto point = CreatePointMsgs();
        server->Publish<commsgs::geometry_msgs::Point>("curve", std::move(point));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

}   // namespace visualization
}   // namespace autonomy