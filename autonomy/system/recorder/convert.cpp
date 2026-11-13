/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file convert.cpp
 * @brief MCAP to LeRobot v2.1 JSONL (odom + cmd_vel).
 */

#include "autonomy/system/recorder/convert.hpp"

#include <filesystem>
#include <fstream>
#include <optional>

#include "nlohmann/json.hpp"

#include "autonomy/system/recorder/mcap_writer.hpp"
#include "autonomy/system/recorder/signals.hpp"

namespace autonomy {
namespace system {
namespace recorder {

namespace fs = std::filesystem;

bool ConvertMcapToLeRobot(const std::string& mcap_path,
                          const std::string& out_dir, double fps) {
    const auto msgs = RecoverMcap(mcap_path);
    if (msgs.empty()) {
        return false;
    }
    std::vector<std::pair<int64_t, automsgs::msgs::geometry_msgs::Pose2D>> odoms;
    std::vector<std::pair<int64_t, automsgs::msgs::geometry_msgs::Twist>> actions;
    for (const auto& m : msgs) {
        Sample s;
        s.channel = m.topic;
        s.payload = m.payload;
        s.clock.set_monotonic_ns(static_cast<int64_t>(m.log_time_ns));
        if (m.topic.find("odom") != std::string::npos) {
            if (auto p = ParseOdom(s)) {
                odoms.push_back({s.clock.monotonic_ns(), *p});
            }
        }
        if (m.topic.find("cmd_vel") != std::string::npos) {
            if (auto tw = ParseTwist(s)) {
                actions.push_back({s.clock.monotonic_ns(), *tw});
            }
        }
    }
    if (actions.empty()) {
        return false;
    }

    fs::create_directories(fs::path(out_dir) / "meta");
    fs::create_directories(fs::path(out_dir) / "data" / "chunk-000");

    nlohmann::json info = {
        {"codebase_version", "v2.1"},
        {"robot_type", "autonomy"},
        {"note",
         "state is odom+cmd_vel; camera/BEV/policy tensors stay in the MCAP"},
        {"fps", fps > 0 ? fps : 10.0},
        {"total_episodes", 1},
        {"total_frames", static_cast<int>(actions.size())},
        {"features",
         {{"observation.state", {{"dtype", "float32"}, {"shape", {5}}}},
          {"action", {{"dtype", "float32"}, {"shape", {2}}}},
          {"timestamp", {{"dtype", "float32"}, {"shape", {1}}}},
          {"episode_index", {{"dtype", "int64"}, {"shape", {1}}}},
          {"frame_index", {{"dtype", "int64"}, {"shape", {1}}}},
          {"index", {{"dtype", "int64"}, {"shape", {1}}}},
          {"next.done", {{"dtype", "bool"}, {"shape", {1}}}}}},
    };
    {
        std::ofstream out(fs::path(out_dir) / "meta" / "info.json");
        out << info.dump(2) << '\n';
    }
    {
        nlohmann::json ep = {{"episode_index", 0},
                             {"length", static_cast<int>(actions.size())},
                             {"tasks", nlohmann::json::array({"navigate"})}};
        std::ofstream out(fs::path(out_dir) / "meta" / "episodes.jsonl");
        out << ep.dump() << '\n';
    }
    {
        std::ofstream tasks(fs::path(out_dir) / "meta" / "tasks.jsonl");
        tasks << nlohmann::json({{"task_index", 0}, {"task", "navigate"}}).dump()
              << '\n';
    }

    std::ofstream frames(fs::path(out_dir) / "data" / "chunk-000" /
                         "episode_000000.jsonl");
    size_t oi = 0;
    const int64_t t0 = actions.front().first;
    for (size_t i = 0; i < actions.size(); ++i) {
        const auto& [t, tw] = actions[i];
        while (oi + 1 < odoms.size() && odoms[oi + 1].first <= t) {
            ++oi;
        }
        automsgs::msgs::geometry_msgs::Pose2D st;
        if (!odoms.empty()) {
            st = odoms[std::min(oi, odoms.size() - 1)].second;
        }
        const double ts =
            fps > 0 ? static_cast<double>(t - t0) / 1e9 : static_cast<double>(i) / 10.0;
        nlohmann::json row = {
            {"timestamp", ts},
            {"frame_index", static_cast<int>(i)},
            {"episode_index", 0},
            {"index", static_cast<int>(i)},
            {"task_index", 0},
            {"next.done", i + 1 == actions.size()},
            {"observation.state",
             {st.x(), st.y(), st.theta(), tw.linear().x(), tw.angular().z()}},
            {"action", {tw.linear().x(), tw.angular().z()}},
        };
        frames << row.dump() << '\n';
    }
    return true;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
