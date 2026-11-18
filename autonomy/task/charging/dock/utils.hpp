/*
 * Copyright 2026 The Openbot Authors
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
 * @file utils.hpp
 * @brief YAML / proto dock-database parsers (OpenNav layout, no ROS).
 */

#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/charging/dock/geometry.hpp"
#include "autonomy/task/charging/dock/types.hpp"

namespace autonomy {
namespace task {
namespace charging {

inline bool ParseDockFile(const std::string& yaml_filepath, DockMap* dock_db) {
    if (dock_db == nullptr) {
        return false;
    }
    YAML::Node yaml_file;
    try {
        yaml_file = YAML::LoadFile(yaml_filepath);
    } catch (const std::exception& ex) {
        AERROR << "dock database parse fail path=" << yaml_filepath << " "
               << ex.what();
        return false;
    }
    if (!yaml_file["docks"]) {
        AERROR << "dock database missing 'docks' path=" << yaml_filepath;
        return false;
    }
    for (const auto& yaml_dock : yaml_file["docks"]) {
        Dock curr;
        curr.frame = "map";
        const std::string dock_name = yaml_dock.first.as<std::string>();
        const YAML::Node& attribs = yaml_dock.second;
        if (attribs["frame"]) {
            curr.frame = attribs["frame"].as<std::string>();
        }
        if (!attribs["type"] || !attribs["pose"]) {
            AERROR << "dock " << dock_name << " missing type/pose";
            return false;
        }
        curr.type = attribs["type"].as<std::string>();
        const auto pose_arr = attribs["pose"].as<std::vector<double>>();
        if (pose_arr.size() != 3u) {
            AERROR << "dock " << dock_name << " pose size != 3";
            return false;
        }
        curr.pose.mutable_position()->set_x(pose_arr[0]);
        curr.pose.mutable_position()->set_y(pose_arr[1]);
        geom::SetYaw(&curr.pose, pose_arr[2]);
        if (attribs["id"]) {
            curr.id = attribs["id"].as<std::string>();
        }
        dock_db->emplace(dock_name, std::move(curr));
    }
    return true;
}

inline bool LoadDockInstances(
    const ::autonomy::task::proto::DockingOptions& options, DockMap* dock_db) {
    if (dock_db == nullptr) {
        return false;
    }
    if (!options.dock_database().empty()) {
        return ParseDockFile(options.dock_database(), dock_db);
    }
    for (const auto& inst : options.docks()) {
        Dock curr;
        curr.frame = inst.frame().empty() ? "map" : inst.frame();
        curr.type = inst.type();
        curr.id = inst.id();
        curr.pose.mutable_position()->set_x(inst.x());
        curr.pose.mutable_position()->set_y(inst.y());
        geom::SetYaw(&curr.pose, inst.yaw());
        const std::string name = inst.name().empty() ? curr.id : inst.name();
        if (name.empty()) {
            continue;
        }
        dock_db->emplace(name, std::move(curr));
    }
    return true;
}

inline ::automsgs::msgs::geometry_msgs::PoseStamped DockPoseStamped(
    const Dock& dock) {
    ::automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id(dock.frame);
    *pose.mutable_pose() = dock.pose;
    return pose;
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy
