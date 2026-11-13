/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "autonomy/control/controller/policy_controller/observation/depth_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

std::size_t ShapeProduct(const proto::ObservationInput& input,
                         std::size_t fallback) {
    if (input.shape_size() == 0) {
        return fallback;
    }
    std::size_t count = 1;
    bool any = false;
    for (int i = 0; i < input.shape_size(); ++i) {
        if (input.shape(i) > 0) {
            count *= static_cast<std::size_t>(input.shape(i));
            any = true;
        }
    }
    return any ? count : fallback;
}

bool ImageToMetres(const automsgs::msgs::sensor_msgs::Image& image,
                   cv::Mat* metres, std::string* error) {
    if (metres == nullptr || image.width() == 0 || image.height() == 0 ||
        image.data().empty()) {
        if (error != nullptr) {
            *error = "Empty depth image";
        }
        return false;
    }
    const int width = static_cast<int>(image.width());
    const int height = static_cast<int>(image.height());
    const std::string& encoding = image.encoding();
    auto* raw = reinterpret_cast<unsigned char*>(
        const_cast<char*>(image.data().data()));
    if (encoding == "32FC1" || encoding == "32FC") {
        cv::Mat depth(height, width, CV_32FC1, raw,
                      image.step() > 0 ? image.step()
                                       : static_cast<size_t>(width) * 4);
        depth.convertTo(*metres, CV_32FC1);
        return true;
    }
    if (encoding == "16UC1" || encoding == "mono16" || encoding == "16UC") {
        cv::Mat depth(height, width, CV_16UC1, raw,
                      image.step() > 0 ? image.step()
                                       : static_cast<size_t>(width) * 2);
        depth.convertTo(*metres, CV_32FC1, 0.001);
        return true;
    }
    if (error != nullptr) {
        *error = "Depth modality requires 32FC1 or 16UC1, got " + encoding;
    }
    return false;
}

Eigen::Vector3d PathGoalRobot(const ObservationSnapshot& snapshot) {
    if (!snapshot.pose.has_value() || snapshot.path == nullptr ||
        snapshot.path->poses().empty()) {
        return Eigen::Vector3d::Zero();
    }
    const auto& pose = snapshot.pose->pose();
    const auto& goal =
        snapshot.path->poses(snapshot.path->poses_size() - 1).pose().position();
    const double yaw = std::atan2(
        2.0 * (pose.orientation().w() * pose.orientation().z() +
               pose.orientation().x() * pose.orientation().y()),
        1.0 - 2.0 * (pose.orientation().y() * pose.orientation().y() +
                     pose.orientation().z() * pose.orientation().z()));
    const double dx = goal.x() - pose.position().x();
    const double dy = goal.y() - pose.position().y();
    const double dz = goal.z() - pose.position().z();
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    return {c * dx - s * dy, s * dx + c * dy, dz};
}

}  // namespace

bool EncodeDepth(const ObservationSnapshot& snapshot,
                 const proto::ObservationInput& input,
                 const proto::SandPlanner& options, std::vector<float>* output,
                 std::string* error) {
    if (output == nullptr) {
        return false;
    }
    std::vector<std::shared_ptr<const automsgs::msgs::sensor_msgs::Image>>
        frames = snapshot.depth_history;
    if (frames.empty() && snapshot.image != nullptr) {
        frames.push_back(snapshot.image);
    }
    if (frames.empty()) {
        if (error != nullptr) {
            *error = "Depth modality requires a depth image";
        }
        return false;
    }
    int history = options.depth_history() > 0 ? options.depth_history() : 4;
    int height = options.depth_height() > 0 ? options.depth_height() : 168;
    int width = options.depth_width() > 0 ? options.depth_width() : 224;
    if (input.shape_size() >= 4) {
        if (input.shape(1) > 0) {
            history = static_cast<int>(input.shape(1));
        }
        if (input.shape(2) > 0) {
            height = static_cast<int>(input.shape(2));
        }
        if (input.shape(3) > 0) {
            width = static_cast<int>(input.shape(3));
        }
    }
    const std::size_t need = static_cast<std::size_t>(history) *
                             static_cast<std::size_t>(height) *
                             static_cast<std::size_t>(width);
    output->assign(ShapeProduct(input, need), 0.0f);
    if (output->size() < need) {
        output->assign(need, 0.0f);
    }
    while (static_cast<int>(frames.size()) < history) {
        frames.insert(frames.begin(), frames.front());
    }
    if (static_cast<int>(frames.size()) > history) {
        frames.erase(frames.begin(),
                     frames.begin() +
                         (static_cast<int>(frames.size()) - history));
    }
    const float max_depth = options.max_depth() > 0.0f ? options.max_depth()
                                                       : 10.0f;
    for (int t = 0; t < history; ++t) {
        cv::Mat metres;
        if (!ImageToMetres(*frames[static_cast<std::size_t>(t)], &metres,
                           error)) {
            return false;
        }
        cv::Mat resized;
        cv::resize(metres, resized, cv::Size(width, height), 0, 0,
                   cv::INTER_NEAREST);
        const std::size_t frame_offset =
            static_cast<std::size_t>(t) * static_cast<std::size_t>(height) *
            static_cast<std::size_t>(width);
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                float value = resized.at<float>(row, col);
                if (!std::isfinite(value) || value < 0.0f) {
                    value = 0.0f;
                }
                value = std::min(value, max_depth);
                (*output)[frame_offset +
                          static_cast<std::size_t>(row * width + col)] = value;
            }
        }
    }
    return true;
}

bool EncodeGoal(const ObservationSnapshot& snapshot,
                const proto::ObservationInput& input, std::vector<float>* output,
                std::string* error) {
    if (output == nullptr) {
        return false;
    }
    if (!snapshot.pose.has_value() || snapshot.path == nullptr ||
        snapshot.path->poses().empty()) {
        if (error != nullptr) {
            *error = "Goal modality requires pose and a non-empty path";
        }
        return false;
    }
    const Eigen::Vector3d goal = PathGoalRobot(snapshot);
    const std::size_t need = ShapeProduct(input, 3);
    output->assign(need, 0.0f);
    if (output->size() >= 3) {
        (*output)[0] = static_cast<float>(goal.x());
        (*output)[1] = static_cast<float>(goal.y());
        (*output)[2] = static_cast<float>(goal.z());
    }
    return true;
}

bool EncodePreviousVelocity(const ObservationEncodeContext& context,
                            const proto::ObservationInput& input,
                            std::vector<float>* output, std::string* error) {
    (void)error;
    if (output == nullptr) {
        return false;
    }
    const std::size_t need = ShapeProduct(input, 3);
    output->assign(need, 0.0f);
    if (context.has_previous_plan_heading && output->size() >= 3) {
        (*output)[0] = static_cast<float>(context.previous_plan_heading.x());
        (*output)[1] = static_cast<float>(context.previous_plan_heading.y());
        (*output)[2] = static_cast<float>(context.previous_plan_heading.z());
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
