// Copyright 2026 The Openbot Authors
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sys/resource.h>

#include <CLI/CLI.hpp>
#include <glog/logging.h>
#include <nlohmann/json.hpp>
#include "autolink/record/record_reader.hpp"
#include "autonomy/localization/cartographer/mapping/map_builder.hpp"
#include "autonomy/localization/cartographer/mapping/internal/2d/pose_graph_2d.hpp"
#include "autonomy/localization/cartographer/node/map_builder_bridge.hpp"
#include "autonomy/localization/cartographer/node/node_constants.hpp"
#include "autonomy/localization/cartographer/node/node_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/cartographer/node/time_conversion.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace fs = std::filesystem;
namespace carto = ::cartographer;
namespace native = autonomy::localization::cartographer::node;
using Clock = std::chrono::steady_clock;

int main(int argc, char** argv) {
    CLI::App app{"Offline 2D Cartographer record evaluation; creates no Autolink nodes or topics"};
    std::string record, configuration_directory, configuration_basename, output_dir, static_transforms;
    double seconds = 0.;
    app.add_option("--record", record)->required()->check(CLI::ExistingFile);
    app.add_option("--configuration_directory", configuration_directory)->required()->check(CLI::ExistingDirectory);
    app.add_option("--configuration_basename", configuration_basename)->required();
    app.add_option("--output_dir", output_dir)->required();
    app.add_option("--static_transforms", static_transforms)->check(CLI::ExistingFile);
    app.add_option("--seconds", seconds, "0 = whole record")->check(CLI::NonNegativeNumber);
    CLI11_PARSE(app, argc, argv);
    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);
    try {
        if (!std::isfinite(seconds)) throw std::invalid_argument("Invalid duration");
        const fs::path output = fs::absolute(output_dir);
        if (fs::exists(output) && !fs::is_empty(output)) throw std::runtime_error("Output directory must be new or empty");
        fs::create_directories(output);
        auto [node_options, trajectory_options] = native::LoadOptions(configuration_directory, configuration_basename);
        if (!node_options.map_builder_options.use_trajectory_builder_2d() ||
            node_options.map_builder_options.use_trajectory_builder_3d() ||
            trajectory_options.num_point_clouds != 0 || trajectory_options.use_nav_sat || trajectory_options.use_landmarks)
            throw std::runtime_error("This runner supports 2D LaserScan/MultiEchoLaserScan + IMU/odometry");
        autolink::record::RecordReader reader(record);
        if (!reader.IsValid()) throw std::runtime_error("Invalid record");
        using SensorId = carto::mapping::TrajectoryBuilderInterface::SensorId;
        using SensorType = SensorId::SensorType;
        std::set<SensorId> sensors;
        const auto laser_topics = native::ResolveLaserScanTopics(trajectory_options);
        const auto echo_topics = native::ResolveMultiEchoLaserScanTopics(trajectory_options);
        for (const auto& topic : laser_topics) sensors.insert({SensorType::RANGE, topic});
        for (const auto& topic : echo_topics) sensors.insert({SensorType::RANGE, topic});
        const bool use_imu = trajectory_options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data();
        if (use_imu) sensors.insert({SensorType::IMU, trajectory_options.imu_topic});
        if (trajectory_options.use_odometry) sensors.insert({SensorType::ODOMETRY, trajectory_options.odometry_topic});
        const auto channels = reader.GetChannelList();
        for (const auto& sensor : sensors)
            if (!channels.count(sensor.id)) throw std::runtime_error("Record missing configured sensor: " + sensor.id);
        auto require_type = [&](const std::string& topic, const std::string& type) {
            if (reader.GetMessageType(topic) != type) throw std::runtime_error("Unexpected record message type on " + topic);
        };
        for (const auto& topic : laser_topics) require_type(topic, "automsgs.msgs.sensor_msgs.LaserScan");
        for (const auto& topic : echo_topics) require_type(topic, "automsgs.msgs.sensor_msgs.MultiEchoLaserScan");
        if (use_imu) require_type(trajectory_options.imu_topic, "automsgs.msgs.sensor_msgs.Imu");
        if (trajectory_options.use_odometry) require_type(trajectory_options.odometry_topic, "automsgs.msgs.nav_msgs.Odometry");
        auto* buffer = autonomy::transform::Buffer::Instance();
        // Do not call Buffer::Init or autolink::Init: this process must not join a running robot.
        buffer->clear();
        if (static_transforms.empty()) static_transforms = native::ResolveStaticTransformYamlPath(configuration_directory, configuration_basename);
        if (fs::exists(static_transforms) && autonomy::transform::LoadStaticTransformsFromFile(buffer, static_transforms, "benchmark") == 0)
            throw std::runtime_error("Static transform file contains no valid transforms");
        // No published-frame TF is needed for offline local-pose collection.
        trajectory_options.published_frame = trajectory_options.tracking_frame;
        native::MapBuilderBridge bridge(node_options, carto::mapping::CreateMapBuilder(node_options.map_builder_options), buffer);
        const int trajectory = bridge.AddTrajectory(sensors, trajectory_options);
        auto* sensor_bridge = bridge.sensor_bridge(trajectory);
        std::ofstream local(output / "local.tum");
        local << std::fixed << std::setprecision(9);
        if (!local) throw std::runtime_error("Cannot create local trajectory");
        autolink::record::RecordMessage message;
        uint64_t first_time = 0, last_time = 0;
        bool started = false;
        size_t ranges = 0, imus = 0, odometry = 0, local_samples = 0;
        carto::common::Time last_local_time = carto::common::Time::min();
        std::vector<double> latencies;
        const auto start = Clock::now();
        while (reader.ReadMessage(&message)) {
            if (!started) { first_time = message.time; started = true; }
            if (seconds > 0. && message.time >= first_time && (message.time - first_time) * 1e-9 > seconds) break;
            last_time = std::max(last_time, message.time);
            const auto step = Clock::now();
            bool range = false;
            if (message.channel_name == "/tf" || message.channel_name == "/tf_static") {
                automsgs::msgs::tf2_msgs::TFMessage msg;
                if (!msg.ParseFromString(message.content)) throw std::runtime_error("Invalid TF payload");
                autonomy::transform::ApplyTfMessageToBuffer(buffer, msg, "record", message.channel_name == "/tf_static");
            } else if (use_imu && message.channel_name == trajectory_options.imu_topic) {
                automsgs::msgs::sensor_msgs::Imu msg;
                if (!msg.ParseFromString(message.content)) throw std::runtime_error("Invalid IMU payload");
                sensor_bridge->HandleImuMessage(message.channel_name, msg); ++imus;
            } else if (trajectory_options.use_odometry && message.channel_name == trajectory_options.odometry_topic) {
                automsgs::msgs::nav_msgs::Odometry msg;
                if (!msg.ParseFromString(message.content)) throw std::runtime_error("Invalid odometry payload");
                sensor_bridge->HandleOdometryMessage(message.channel_name, msg); ++odometry;
            } else if (std::find(laser_topics.begin(), laser_topics.end(), message.channel_name) != laser_topics.end()) {
                automsgs::msgs::sensor_msgs::LaserScan msg;
                if (!msg.ParseFromString(message.content)) throw std::runtime_error("Invalid scan payload");
                sensor_bridge->HandleLaserScanMessage(message.channel_name, msg); range = true;
            } else if (std::find(echo_topics.begin(), echo_topics.end(), message.channel_name) != echo_topics.end()) {
                automsgs::msgs::sensor_msgs::MultiEchoLaserScan msg;
                if (!msg.ParseFromString(message.content)) throw std::runtime_error("Invalid multi-echo scan payload");
                sensor_bridge->HandleMultiEchoLaserScanMessage(message.channel_name, msg); range = true;
            }
            if (!range) continue;
            ++ranges;
            const auto data = bridge.GetLocalTrajectoryData();
            const auto it = data.find(trajectory);
            if (it == data.end() || it->second.local_slam_data->time == last_local_time) continue;
            const auto& sample = *it->second.local_slam_data;
            last_local_time = sample.time;
            const auto stamp = native::ToCommsgs(sample.time);
            const auto& p = sample.local_pose.translation(); const auto& q = sample.local_pose.rotation();
            local << stamp.sec() + stamp.nanosec() * 1e-9 << ' ' << p.x() << ' ' << p.y() << ' ' << p.z()
                  << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
            ++local_samples;
            latencies.push_back(std::chrono::duration<double, std::milli>(Clock::now() - step).count());
        }
        bridge.FinishTrajectory(trajectory);
        const auto finish = Clock::now();
        const bool global = node_options.map_builder_options.pose_graph_options().optimize_every_n_nodes() > 0;
        if (global) bridge.RunFinalOptimization();
        else dynamic_cast<carto::mapping::PoseGraph2D&>(*bridge.map_builder()->pose_graph()).WaitForAllComputations();
        const auto optimized = Clock::now();
        const auto nodes = bridge.map_builder()->pose_graph()->GetTrajectoryNodePoses();
        std::ofstream path(output / "trajectory.tum"); path << std::fixed << std::setprecision(9);
        for (const auto& node : nodes) {
            if (!node.data.constant_pose_data) continue;
            const auto stamp = native::ToCommsgs(node.data.constant_pose_data->time);
            const auto& p = node.data.global_pose.translation(); const auto& q = node.data.global_pose.rotation();
            path << stamp.sec() + stamp.nanosec() * 1e-9 << ' ' << p.x() << ' ' << p.y() << ' ' << p.z()
                 << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        }
        if (!local || !path || local_samples < 2) throw std::runtime_error("Insufficient valid SLAM results; check sensor topics, TF and timestamps");
        if (!bridge.SerializeState((output / "state.pbstream").string(), true)) throw std::runtime_error("Failed to save state");
        std::sort(latencies.begin(), latencies.end());
        auto percentile = [&](double fraction) { return latencies[static_cast<size_t>((latencies.size() - 1) * fraction)]; };
        const double processing = std::chrono::duration<double>(finish - start).count();
        const double slam_wall = std::chrono::duration<double>(optimized - start).count();
        const double duration = (last_time - first_time) * 1e-9;
        struct rusage usage{}; getrusage(RUSAGE_SELF, &usage);
        const auto& frontend = trajectory_options.trajectory_builder_options.trajectory_builder_2d_options();
        nlohmann::json summary = {
            {"record", fs::absolute(record).string()}, {"tracking_frame", trajectory_options.tracking_frame},
            {"sensor_duration_sec", duration}, {"processing_wall_sec", processing},
            {"final_optimization_wall_sec", global ? std::chrono::duration<double>(optimized - finish).count() : 0.},
            {"background_drain_wall_sec", global ? 0. : std::chrono::duration<double>(optimized - finish).count()},
            {"slam_wall_sec", slam_wall},
            {"realtime_factor", duration / std::max(slam_wall, 1e-9)},
            {"processing_realtime_factor", duration / std::max(processing, 1e-9)},
            {"peak_rss_mib", usage.ru_maxrss / 1024.},
            {"range_messages", ranges}, {"imu_messages", imus}, {"odometry_messages", odometry},
            {"local_slam_samples", local_samples}, {"trajectory_nodes", nodes.size()}, {"global_optimization", global},
            {"frontend_step_p50_ms", percentile(.5)}, {"frontend_step_p95_ms", percentile(.95)},
            {"frontend_step_max_ms", latencies.back()},
            {"voxel_filter_size", frontend.voxel_filter_size()},
            {"translation_weight", frontend.ceres_scan_matcher_options().translation_weight()},
            {"rotation_weight", frontend.ceres_scan_matcher_options().rotation_weight()}};
        std::ofstream result(output / "summary.json"); result << summary.dump(2) << '\n';
        if (!result) throw std::runtime_error("Cannot write summary");
        std::cout << summary.dump() << '\n';
        return 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Offline benchmark failed: " << e.what();
        return 1;
    }
}
