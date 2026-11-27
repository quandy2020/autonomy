// Copyright 2026 The Openbot Authors
#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <variant>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include "autolink/autolink.hpp"
#include "autolink/timer/timer.hpp"
#include "autonomy/localization/fusion/planar_ekf.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy::localization::fusion {

struct OdometryFusionConfig {
    std::string wheel_topic = "/wheel/odometry";
    std::string imu_topic = "/imu";
    std::string output_topic = "/odometry/filtered";
    std::string tf_topic = "/tf";
    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    bool publish_tf = true;
    double reorder_delay = .05;
    double gyro_bias = 0.;
    double gyro_variance = .0001;
    Eigen::Vector3d wheel_variance = Eigen::Vector3d::Constant(.0025);
    PlanarEkf::Options filter;
};

OdometryFusionConfig LoadOdometryFusionConfig(const std::string& path);

class OdometryFusionNode : public std::enable_shared_from_this<OdometryFusionNode> {
public:
    using Odometry = automsgs::msgs::nav_msgs::Odometry;
    using Imu = automsgs::msgs::sensor_msgs::Imu;
    OdometryFusionNode(OdometryFusionConfig config, transform::Buffer* buffer);
    ~OdometryFusionNode();
    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

private:
    using Message = std::variant<std::shared_ptr<Odometry>, std::shared_ptr<Imu>>;
    struct Event { Message message; std::chrono::steady_clock::time_point arrived; };
    void Enqueue(double time, Message message);
    void Flush();
    bool Process(const Odometry& message);
    bool Process(const Imu& message);
    void Publish();
    OdometryFusionConfig config_;
    transform::Buffer* buffer_;
    PlanarEkf filter_;
    std::mutex mutex_;
    std::map<std::pair<double, uint64_t>, Event> pending_;
    uint64_t sequence_ = 0;
    double published_time_ = -1.;
    automsgs::msgs::builtin_interfaces::Time latest_stamp_;
    std::shared_ptr<autolink::Reader<Odometry>> wheel_reader_;
    std::shared_ptr<autolink::Reader<Imu>> imu_reader_;
    std::shared_ptr<autolink::Writer<Odometry>> writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>> tf_writer_;
    std::unique_ptr<autolink::Timer> timer_;
};

}  // namespace autonomy::localization::fusion
