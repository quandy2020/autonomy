// Copyright 2026 The Openbot Authors
#include "autonomy/localization/fusion/odometry_fusion_node.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>
#include <type_traits>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include "autonomy/transform/tf2/convert.h"

namespace autonomy::localization::fusion {
namespace {
double Seconds(const automsgs::msgs::builtin_interfaces::Time& stamp) {
    if (stamp.sec() < 0 || stamp.nanosec() >= 1000000000) return -1.;
    return stamp.sec() + stamp.nanosec() * 1e-9;
}
}

OdometryFusionConfig LoadOdometryFusionConfig(const std::string& path) {
    const auto yaml = YAML::LoadFile(path);
    OdometryFusionConfig c;
    auto read = [&](const char* key, auto& value) {
        if (yaml[key]) value = yaml[key].as<std::decay_t<decltype(value)>>();
    };
    read("wheel_topic", c.wheel_topic); read("imu_topic", c.imu_topic);
    read("output_topic", c.output_topic); read("tf_topic", c.tf_topic);
    read("odom_frame", c.odom_frame); read("base_frame", c.base_frame);
    read("publish_tf", c.publish_tf); read("reorder_delay", c.reorder_delay);
    read("gyro_bias", c.gyro_bias); read("gyro_variance", c.gyro_variance);
    read("acceleration_noise", c.filter.acceleration_noise);
    read("angular_acceleration_noise", c.filter.angular_acceleration_noise);
    read("wheel_timeout", c.filter.wheel_timeout);
    if (yaml["wheel_velocity_variance"]) {
        const auto values = yaml["wheel_velocity_variance"].as<std::vector<double>>();
        if (values.size() != 3) throw std::invalid_argument("wheel_velocity_variance requires [vx, vy, yaw_rate]");
        c.wheel_variance = Eigen::Map<const Eigen::Vector3d>(values.data());
    }
    return c;
}

OdometryFusionNode::OdometryFusionNode(OdometryFusionConfig config, transform::Buffer* buffer)
    : config_(std::move(config)), buffer_(buffer), filter_(config_.filter) {
    for (const auto& topic : {config_.wheel_topic, config_.imu_topic, config_.output_topic, config_.tf_topic})
        if (topic.empty() || topic.front() != '/') throw std::invalid_argument("EKF topics require a leading /");
    if (config_.wheel_topic == config_.output_topic || config_.imu_topic == config_.output_topic ||
        config_.wheel_topic == config_.imu_topic || config_.tf_topic == config_.output_topic ||
        config_.tf_topic == config_.wheel_topic || config_.tf_topic == config_.imu_topic)
        throw std::invalid_argument("EKF input/output topics must be distinct");
    for (const auto& frame : {config_.odom_frame, config_.base_frame})
        if (frame.empty() || frame.front() == '/') throw std::invalid_argument("Invalid EKF frame name");
    if (config_.odom_frame == config_.base_frame || !std::isfinite(config_.reorder_delay) ||
        config_.reorder_delay < 0. || config_.reorder_delay >= config_.filter.wheel_timeout ||
        !std::isfinite(config_.gyro_bias) || !std::isfinite(config_.gyro_variance) ||
        config_.gyro_variance <= 0. || !config_.wheel_variance.allFinite() ||
        (config_.wheel_variance.array() <= 0.).any()) throw std::invalid_argument("Invalid EKF frames/noise/timing");
}

OdometryFusionNode::~OdometryFusionNode() { Stop(); }

bool OdometryFusionNode::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!node || timer_ || weak_from_this().expired()) return false;
    writer_ = node->CreateWriter<Odometry>(config_.output_topic);
    if (config_.publish_tf) tf_writer_ = node->CreateWriter<automsgs::msgs::tf2_msgs::TFMessage>(config_.tf_topic);
    if (!writer_ || (config_.publish_tf && !tf_writer_)) return false;
    const auto weak = weak_from_this();
    autolink::ReaderConfig reader_config;
    // The default one-message queue drops sensor bursts before timestamp reordering.
    reader_config.pending_queue_size = 256;
    reader_config.channel_name = config_.wheel_topic;
    wheel_reader_ = node->CreateReader<Odometry>(reader_config, [weak](const std::shared_ptr<Odometry>& msg) {
        if (auto self = weak.lock(); self && msg) self->Enqueue(Seconds(msg->header().stamp()), msg);
    });
    reader_config.channel_name = config_.imu_topic;
    imu_reader_ = node->CreateReader<Imu>(reader_config, [weak](const std::shared_ptr<Imu>& msg) {
        if (auto self = weak.lock(); self && msg) self->Enqueue(Seconds(msg->header().stamp()), msg);
    });
    if (!wheel_reader_ || !imu_reader_) { Stop(); return false; }
    timer_ = std::make_unique<autolink::Timer>(10, [weak] {
        if (auto self = weak.lock()) self->Flush();
    }, false);
    timer_->Start();
    LOG(INFO) << "Wheel/IMU EKF: " << config_.wheel_topic << " + " << config_.imu_topic << " -> " << config_.output_topic;
    return true;
}

void OdometryFusionNode::Stop() {
    timer_.reset();
    if (wheel_reader_) wheel_reader_->Shutdown();
    if (imu_reader_) imu_reader_->Shutdown();
    wheel_reader_.reset(); imu_reader_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear(); writer_.reset(); tf_writer_.reset();
}

void OdometryFusionNode::Enqueue(double time, Message message) {
    if (time < 0.) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.size() >= 4096) {
        LOG_EVERY_N(WARNING, 100) << "EKF sensor queue overflow; dropping oldest measurement";
        pending_.erase(pending_.begin());
    }
    pending_.emplace(std::make_pair(time, sequence_++), Event{std::move(message), std::chrono::steady_clock::now()});
}

void OdometryFusionNode::Flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    bool updated = false;
    while (!pending_.empty()) {
        const auto it = pending_.begin();
        if (std::chrono::duration<double>(now - it->second.arrived).count() < config_.reorder_delay) break;
        const bool accepted = std::visit([&](const auto& msg) {
            if (!Process(*msg)) {
                if (filter_.initialized()) {
                    LOG_EVERY_N(WARNING, 100) << "EKF discarded invalid, stale or out-of-order measurement";
                }
                return false;
            }
            latest_stamp_ = msg->header().stamp();
            return true;
        }, it->second.message);
        updated |= accepted;
        pending_.erase(it);
    }
    if (updated && filter_.time() > published_time_) Publish();
}

bool OdometryFusionNode::Process(const Odometry& msg) {
    if (msg.child_frame_id() != config_.base_frame || msg.header().frame_id() != config_.odom_frame) {
        LOG_EVERY_N(WARNING, 100) << "EKF wheel frames must match configured odom_frame/base_frame";
        return false;
    }
    const auto& twist = msg.twist().twist();
    Eigen::Vector3d velocity(twist.linear().x(), twist.linear().y(), twist.angular().z());
    Eigen::Matrix3d covariance = config_.wheel_variance.asDiagonal();
    if (msg.twist().covariance_size() != 0 && msg.twist().covariance_size() != 36) return false;
    if (msg.twist().covariance_size() == 36) {
        const int indices[] = {0, 1, 5};
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
            const double value = msg.twist().covariance(indices[r] * 6 + indices[c]);
            if (!std::isfinite(value) || (r == c && value < 0.)) return false;
            covariance(r, c) = r == c && value == 0. ? config_.wheel_variance(r) : value;
        }
    }
    return filter_.AddWheel(Seconds(msg.header().stamp()), velocity, covariance);
}

bool OdometryFusionNode::Process(const Imu& msg) {
    if (msg.header().frame_id().empty() || msg.header().frame_id().front() == '/') return false;
    const auto& w = msg.angular_velocity();
    const Eigen::Vector3d angular(w.x(), w.y(), w.z());
    if (!angular.allFinite() || (msg.angular_velocity_covariance_size() != 0 &&
        msg.angular_velocity_covariance_size() != 9)) return false;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    if (msg.header().frame_id() != config_.base_frame) {
        try {
            if (!buffer_) throw std::runtime_error("No TF buffer");
            const auto tf = buffer_->lookupTransform(config_.base_frame, msg.header().frame_id(), msg.header().stamp(), 0.f);
            const auto& q = tf.transform().rotation();
            const Eigen::Quaterniond quaternion(q.w(), q.x(), q.y(), q.z());
            if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-6) return false;
            rotation = quaternion.normalized().toRotationMatrix();
        } catch (...) {
            LOG_EVERY_N(WARNING, 100) << "EKF waiting for IMU-to-base TF";
            return false;
        }
    }
    double variance = config_.gyro_variance;
    if (msg.angular_velocity_covariance_size() == 9) {
        Eigen::Matrix3d covariance;
        for (int i = 0; i < 9; ++i) covariance(i / 3, i % 3) = msg.angular_velocity_covariance(i);
        if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1e-9) ||
            (covariance.diagonal().array() < 0.).any()) return false;
        if (!covariance.isZero()) {
            Eigen::LDLT<Eigen::Matrix3d> decomposition(covariance);
            if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) return false;
            variance = (rotation * covariance * rotation.transpose())(2, 2);
            if (variance <= 0.) variance = config_.gyro_variance;
        }
    }
    return filter_.AddGyro(Seconds(msg.header().stamp()), (rotation * angular).z() - config_.gyro_bias, variance);
}

void OdometryFusionNode::Publish() {
    if (!writer_) return;
    const auto x = filter_.state();
    const auto p = filter_.covariance();
    if (!x.allFinite() || !p.allFinite()) return;
    Odometry msg;
    msg.mutable_header()->set_frame_id(config_.odom_frame);
    *msg.mutable_header()->mutable_stamp() = latest_stamp_;
    msg.set_child_frame_id(config_.base_frame);
    auto* pose = msg.mutable_pose()->mutable_pose();
    *pose->mutable_header() = msg.header();
    pose->mutable_pose()->mutable_position()->set_x(x(0));
    pose->mutable_pose()->mutable_position()->set_y(x(1));
    pose->mutable_pose()->mutable_orientation()->set_z(std::sin(x(2) * .5));
    pose->mutable_pose()->mutable_orientation()->set_w(std::cos(x(2) * .5));
    msg.mutable_twist()->mutable_twist()->mutable_linear()->set_x(x(3));
    msg.mutable_twist()->mutable_twist()->mutable_linear()->set_y(x(4));
    msg.mutable_twist()->mutable_twist()->mutable_angular()->set_z(x(5));
    const int indices[] = {0, 1, 5};
    for (int i = 0; i < 36; ++i) { msg.mutable_pose()->add_covariance(0.); msg.mutable_twist()->add_covariance(0.); }
    for (int i : {2, 3, 4}) {
        msg.mutable_pose()->set_covariance(i * 6 + i, 1e6);
        msg.mutable_twist()->set_covariance(i * 6 + i, 1e6);
    }
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
        msg.mutable_pose()->set_covariance(indices[r] * 6 + indices[c], p(r, c));
        msg.mutable_twist()->set_covariance(indices[r] * 6 + indices[c], p(r + 3, c + 3));
    }
    if (!writer_->Write(msg)) { LOG(ERROR) << "EKF odometry publish failed"; return; }
    if (tf_writer_) {
        automsgs::msgs::tf2_msgs::TFMessage tf;
        auto* transform = tf.add_transforms();
        *transform->mutable_header() = msg.header();
        transform->set_child_frame_id(config_.base_frame);
        transform->mutable_transform()->mutable_translation()->set_x(x(0));
        transform->mutable_transform()->mutable_translation()->set_y(x(1));
        *transform->mutable_transform()->mutable_rotation() = pose->pose().orientation();
        if (!tf_writer_->Write(tf)) LOG(ERROR) << "EKF TF publish failed";
    }
    published_time_ = filter_.time();
}

}  // namespace autonomy::localization::fusion
