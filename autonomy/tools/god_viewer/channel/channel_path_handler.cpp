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

#include "autonomy/tools/god_viewer/channel/channel_path_handler.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

PathHandler::PathHandler(ServerHander::SharedPtr options, const std::string& topic)
    : topic_{topic}
{
    LOG(INFO) << "Init path handler topic: " << topic_;
    channel_ = std::make_unique<foxglove::schemas::SceneUpdateChannel>(
        foxglove::schemas::SceneUpdateChannel::create(topic_).value());
}

bool PathHandler::Send()
{
    foxglove::schemas::CubePrimitive cube;
    cube.size = foxglove::schemas::Vector3{1, 1, 2};
    cube.color = foxglove::schemas::Color{1, 0, 0, 1};

    foxglove::schemas::SceneEntity entity;
    entity.frame_id = "map";
    entity.id = "box";
    entity.cubes.push_back(cube);

    foxglove::schemas::SceneUpdate scene_update;
    scene_update.entities.push_back(entity);

    channel_->log(scene_update);
    return true;
}

bool PathHandler::SendTest()
{
    return true;
}

bool PathHandler::Send(const commsgs::planning_msgs::Path& msgs)
{
    auto line = FromCommsgs(msgs);
    foxglove::schemas::SceneEntity entity;
    entity.frame_id = "map";
    entity.id = "line";
    entity.lines.push_back(line);
    foxglove::schemas::SceneUpdate scene_update;
    scene_update.entities.push_back(entity);
    channel_->log(scene_update);
    return true;
}

commsgs::planning_msgs::Path PathHandler::GenerateCircularPath(double radius)
{
    commsgs::planning_msgs::Path path;
    int num_points = 500;
    double z_height = 0.5;
    double z_amplitude = 0.3;

    for (int i = 0; i < num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        commsgs::geometry_msgs::PoseStamped pt;
        pt.pose.position.x = radius * cos(angle);
        pt.pose.position.y = radius * sin(angle);
        pt.pose.position.z = z_height + z_amplitude * sin(angle * 3); // 添加一些高度变化
        path.poses.push_back(pt);
    }

    return path;
}

foxglove::schemas::LinePrimitive PathHandler::FromCommsgs(const commsgs::planning_msgs::Path& msgs)
{
    // type	LineType	Drawing primitive to use for lines
    // pose	Pose	Origin of lines relative to reference frame
    // thickness	float64	Line thickness
    // scale_invariant	boolean	Indicates whether thickness is a fixed size in screen pixels (true), or specified in world coordinates and scales with distance from the camera (false)
    // points	Point3[]	Points along the line
    // color	Color	Solid color to use for the whole line. One of color or colors must be provided.
    // colors	Color[]	Per-point colors (if specified, must have the same length as points). One of color or colors must be provided.
    // indices	uint32[]	Indices into the points and colors attribute arrays, which can be used to avoid duplicating attribute data.

    foxglove::schemas::LinePrimitive line;
    line.type = foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    line.thickness = 0.08;
    line.color = foxglove::schemas::Color{0, 1, 0, 1};
    for (int i = 0; i < msgs.poses.size(); i++) {
        line.points.push_back({
            msgs.poses[i].pose.position.x, 
            msgs.poses[i].pose.position.y,
            msgs.poses[i].pose.position.z
        });
    }
    return line;
}

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy