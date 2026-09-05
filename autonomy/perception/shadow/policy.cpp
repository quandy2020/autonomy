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
 * @file policy.cpp
 * @brief Shadow ground policy preprocessing, inference, and path decoding.
 *
 * The fixed candidate batch and ordered motion-primitive decoding are adapted
 * conceptually from YOPO-Simple commit
 * 58dda5b2d124aa3998593dd62798f1496c93b26d, distributed under the MIT
 * License. This repository-authored integration does not copy an upstream
 * implementation fragment and retains only the ground `(x, y, yaw)` contract;
 * no quadrotor attitude, thrust, differential-flatness, or aerial-dynamics
 * output is used.
 */

#include "autonomy/perception/shadow/policy.hpp"

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/common/network/common/options.hpp"
#include "autonomy/perception/shadow/options.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

constexpr char kDepthName[] = "depth";
constexpr char kRobotStateName[] = "robot_state";
constexpr char kTargetStateName[] = "target_state";
constexpr char kTrajectoriesName[] = "trajectories";
constexpr char kScoresName[] = "scores";

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Shadow policy: " + message;
    }
}

const common::network::ModelTensorInfo* FindTensorInfo(
    const std::vector<common::network::ModelTensorInfo>& infos,
    const char* name) {
    for (const auto& info : infos) {
        if (info.name == name) {
            return &info;
        }
    }
    return nullptr;
}

bool ValidateTensor(const common::network::ModelTensorInfo& info,
                    const char* label, const std::vector<int64_t>& expected,
                    std::string* error) {
    if (info.element_type != common::network::ElementType::kFloat32) {
        SetError(error, std::string(label) + " must use float32 elements.");
        return false;
    }
    if (info.shape.Rank() != expected.size() || info.shape.Dims() != expected) {
        SetError(error,
                 std::string(label) + " has an unexpected static shape.");
        return false;
    }
    return true;
}

bool ValidateModelContract(
    const std::vector<common::network::ModelTensorInfo>& inputs,
    const std::vector<common::network::ModelTensorInfo>& outputs,
    const proto::ShadowOptions& options, std::string* error) {
    const auto* depth = FindTensorInfo(inputs, kDepthName);
    const auto* robot_state = FindTensorInfo(inputs, kRobotStateName);
    const auto* target_state = FindTensorInfo(inputs, kTargetStateName);
    const auto* trajectories = FindTensorInfo(outputs, kTrajectoriesName);
    const auto* scores = FindTensorInfo(outputs, kScoresName);
    if (inputs.size() != 3 || depth == nullptr || robot_state == nullptr ||
        target_state == nullptr) {
        SetError(error,
                 "model inputs must be exactly 'depth', 'robot_state', and "
                 "'target_state'.");
        return false;
    }
    if (outputs.size() != 2 || trajectories == nullptr || scores == nullptr) {
        SetError(error,
                 "model outputs must be exactly 'trajectories' and 'scores'.");
        return false;
    }

    const int64_t width = static_cast<int64_t>(options.policy_width());
    const int64_t height = static_cast<int64_t>(options.policy_height());
    const int64_t candidates = static_cast<int64_t>(options.candidate_count());
    const int64_t steps = static_cast<int64_t>(options.trajectory_steps());
    return ValidateTensor(*depth, "model input 'depth'", {1, 1, height, width},
                          error) &&
           ValidateTensor(*robot_state, "model input 'robot_state'", {1, 2},
                          error) &&
           ValidateTensor(*target_state, "model input 'target_state'", {1, 4},
                          error) &&
           ValidateTensor(*trajectories, "model output 'trajectories'",
                          {1, candidates, steps, 3}, error) &&
           ValidateTensor(*scores, "model output 'scores'", {1, candidates},
                          error);
}

class EnginePolicyRunner final : public PolicyRunner
{
public:
    explicit EnginePolicyRunner(std::unique_ptr<common::network::Engine> engine)
        : PolicyRunner(engine->GetInputInfos(), engine->GetOutputInfos()),
          engine_(std::move(engine)) {}

    bool Run(const common::network::TensorMap& inputs,
             common::network::TensorMap* outputs, std::string* error) override {
        if (error != nullptr) {
            error->clear();
        }
        if (outputs == nullptr) {
            if (error != nullptr) {
                *error = "model output map is null.";
            }
            return false;
        }
        outputs->clear();
        if (!engine_->Run(inputs, outputs)) {
            if (error != nullptr) {
                *error = engine_->GetLastError();
            }
            outputs->clear();
            return false;
        }
        return true;
    }

private:
    std::unique_ptr<common::network::Engine> engine_;
};

bool ValidateDepth(const automsgs::msgs::sensor_msgs::Image& depth,
                   std::string* error) {
    if (depth.encoding() != "32FC1") {
        SetError(error, "depth encoding must be metric '32FC1'.");
        return false;
    }
    if (depth.width() == 0 || depth.height() == 0) {
        SetError(error, "depth dimensions must be positive.");
        return false;
    }
    if (depth.is_bigendian()) {
        SetError(error, "big-endian depth images are unsupported.");
        return false;
    }
    if (depth.width() >
            static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        depth.height() >
            static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetError(error, "depth dimensions exceed OpenCV limits.");
        return false;
    }
    const size_t minimum_step =
        static_cast<size_t>(depth.width()) * sizeof(float);
    if (depth.step() < minimum_step) {
        SetError(error, "depth step is too small.");
        return false;
    }
    if (static_cast<size_t>(depth.height()) >
        std::numeric_limits<size_t>::max() /
            static_cast<size_t>(depth.step())) {
        SetError(error, "depth byte size overflows the host size type.");
        return false;
    }
    const size_t required_data =
        static_cast<size_t>(depth.height()) * depth.step();
    if (depth.data().size() < required_data) {
        SetError(error, "depth data is too small.");
        return false;
    }
    return true;
}

bool PrepareDepth(const proto::ShadowOptions& options,
                  const automsgs::msgs::sensor_msgs::Image& depth,
                  common::network::Tensor* tensor, std::string* error) {
    if (tensor == nullptr) {
        SetError(error, "depth tensor output is null.");
        return false;
    }
    if (!ValidateDepth(depth, error)) {
        return false;
    }
    if (options.policy_width() >
            static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        options.policy_height() >
            static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetError(error, "policy dimensions exceed OpenCV limits.");
        return false;
    }

    cv::Mat source(static_cast<int>(depth.height()),
                   static_cast<int>(depth.width()), CV_32FC1);
    for (uint32_t row = 0; row < depth.height(); ++row) {
        std::memcpy(
            source.ptr<float>(static_cast<int>(row)),
            depth.data().data() + static_cast<size_t>(row) * depth.step(),
            static_cast<size_t>(depth.width()) * sizeof(float));
        float* values = source.ptr<float>(static_cast<int>(row));
        for (uint32_t column = 0; column < depth.width(); ++column) {
            if (!std::isfinite(values[column]) ||
                values[column] < options.min_depth_m() ||
                values[column] > options.max_depth_m()) {
                values[column] = 0.0F;
            }
        }
    }

    cv::Mat resized;
    cv::resize(source, resized,
               cv::Size(static_cast<int>(options.policy_width()),
                        static_cast<int>(options.policy_height())),
               0.0, 0.0, cv::INTER_NEAREST);
    const size_t element_count =
        static_cast<size_t>(options.policy_width()) * options.policy_height();
    std::vector<float> values(element_count);
    for (uint32_t row = 0; row < options.policy_height(); ++row) {
        std::memcpy(
            values.data() + static_cast<size_t>(row) * options.policy_width(),
            resized.ptr<float>(static_cast<int>(row)),
            static_cast<size_t>(options.policy_width()) * sizeof(float));
    }
    *tensor = common::network::Tensor::FromFloat32(std::move(values));
    return true;
}

bool Float32Value(double value, const char* label, float* result,
                  std::string* error) {
    if (!std::isfinite(value)) {
        SetError(error, std::string(label) + " is non-finite.");
        return false;
    }
    const float converted = static_cast<float>(value);
    if (!std::isfinite(converted)) {
        SetError(error, std::string(label) + " exceeds float32 range.");
        return false;
    }
    *result = converted;
    return true;
}

bool PrepareState(
    const proto::ShadowOptions& options,
    const automsgs::msgs::nav_msgs::Odometry& odometry,
    const automsgs::msgs::geometry_msgs::PoseStamped& target,
    const automsgs::msgs::geometry_msgs::TwistStamped& target_velocity,
    common::network::Tensor* robot_tensor,
    common::network::Tensor* target_tensor, std::string* error) {
    if (robot_tensor == nullptr || target_tensor == nullptr) {
        SetError(error, "state tensor output is null.");
        return false;
    }
    if (odometry.header().frame_id() != options.map_frame()) {
        SetError(error, "odometry pose must use the configured map frame.");
        return false;
    }
    if (odometry.child_frame_id() != options.base_frame()) {
        SetError(error, "odometry twist must use the configured base frame.");
        return false;
    }
    if (target.header().frame_id() != options.map_frame() ||
        target_velocity.header().frame_id() != options.map_frame()) {
        SetError(error, "target pose and velocity must use the map frame.");
        return false;
    }

    const auto& robot_pose = odometry.pose().pose().pose();
    const auto& orientation = robot_pose.orientation();
    const double qx = orientation.x();
    const double qy = orientation.y();
    const double qz = orientation.z();
    const double qw = orientation.w();
    if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) ||
        !std::isfinite(qw)) {
        SetError(error, "odometry orientation is non-finite.");
        return false;
    }
    const double norm_squared = qx * qx + qy * qy + qz * qz + qw * qw;
    if (!std::isfinite(norm_squared) ||
        norm_squared <= std::numeric_limits<double>::epsilon()) {
        SetError(error, "odometry orientation is invalid.");
        return false;
    }
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    const double nx = qx * inverse_norm;
    const double ny = qy * inverse_norm;
    const double nz = qz * inverse_norm;
    const double nw = qw * inverse_norm;
    const double yaw =
        std::atan2(2.0 * (nw * nz + nx * ny), 1.0 - 2.0 * (ny * ny + nz * nz));
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);

    const auto& robot_position = robot_pose.position();
    const auto& target_position = target.pose().position();
    const auto& target_linear = target_velocity.twist().linear();
    const double delta_x = target_position.x() - robot_position.x();
    const double delta_y = target_position.y() - robot_position.y();
    const double relative_x = cosine * delta_x + sine * delta_y;
    const double relative_y = -sine * delta_x + cosine * delta_y;
    const double relative_vx =
        cosine * target_linear.x() + sine * target_linear.y();
    const double relative_vy =
        -sine * target_linear.x() + cosine * target_linear.y();

    std::vector<float> robot_state(2);
    std::vector<float> target_state(4);
    if (!Float32Value(odometry.twist().twist().linear().x(),
                      "robot linear velocity", &robot_state[0], error) ||
        !Float32Value(odometry.twist().twist().angular().z(),
                      "robot angular velocity", &robot_state[1], error) ||
        !Float32Value(relative_x, "relative target x", &target_state[0],
                      error) ||
        !Float32Value(relative_y, "relative target y", &target_state[1],
                      error) ||
        !Float32Value(relative_vx, "relative target velocity x",
                      &target_state[2], error) ||
        !Float32Value(relative_vy, "relative target velocity y",
                      &target_state[3], error)) {
        return false;
    }

    *robot_tensor =
        common::network::Tensor::FromFloat32(std::move(robot_state));
    *target_tensor =
        common::network::Tensor::FromFloat32(std::move(target_state));
    return true;
}

bool CheckedMultiply(size_t left, size_t right, size_t* result) {
    if (left != 0 && right > std::numeric_limits<size_t>::max() / left) {
        return false;
    }
    *result = left * right;
    return true;
}

bool FindFloatOutput(const common::network::TensorMap& outputs,
                     const char* name, size_t expected_count,
                     const float** values, std::string* error) {
    const auto output = outputs.find(name);
    if (output == outputs.end()) {
        SetError(error, std::string("model output '") + name + "' is missing.");
        return false;
    }
    std::string detail;
    size_t count = 0;
    if (!output->second.TryViewFloat32(values, &count, &detail)) {
        SetError(error, std::string("model output '") + name +
                            "' must be float32: " + detail);
        return false;
    }
    if (count != expected_count) {
        SetError(error, std::string("model output '") + name +
                            "' has an unexpected tensor size.");
        return false;
    }
    return true;
}

bool DecodeOutputs(const proto::ShadowOptions& options,
                   const automsgs::msgs::sensor_msgs::Image& depth,
                   const common::network::TensorMap& outputs,
                   std::vector<automsgs::msgs::nav_msgs::Path>* paths,
                   std::vector<float>* scores, std::string* error) {
    if (outputs.size() != 2) {
        SetError(error,
                 "inference must return exactly 'trajectories' and 'scores'.");
        return false;
    }
    const size_t candidate_count = options.candidate_count();
    const size_t trajectory_steps = options.trajectory_steps();
    size_t points = 0;
    size_t trajectory_values = 0;
    if (!CheckedMultiply(candidate_count, trajectory_steps, &points) ||
        !CheckedMultiply(points, size_t{3}, &trajectory_values)) {
        SetError(error, "configured trajectory tensor size overflows.");
        return false;
    }

    const float* trajectory_data = nullptr;
    const float* score_data = nullptr;
    if (!FindFloatOutput(outputs, kTrajectoriesName, trajectory_values,
                         &trajectory_data, error) ||
        !FindFloatOutput(outputs, kScoresName, candidate_count, &score_data,
                         error)) {
        return false;
    }
    for (size_t index = 0; index < trajectory_values; ++index) {
        if (!std::isfinite(trajectory_data[index])) {
            SetError(error, "model trajectories contain a non-finite value.");
            return false;
        }
    }
    for (size_t index = 0; index < candidate_count; ++index) {
        if (!std::isfinite(score_data[index])) {
            SetError(error, "model scores contain a non-finite value.");
            return false;
        }
    }

    std::vector<automsgs::msgs::nav_msgs::Path> decoded_paths;
    decoded_paths.reserve(candidate_count);
    for (size_t candidate = 0; candidate < candidate_count; ++candidate) {
        automsgs::msgs::nav_msgs::Path path;
        *path.mutable_header() = depth.header();
        path.mutable_header()->set_frame_id(options.base_frame());
        for (size_t step = 0; step < trajectory_steps; ++step) {
            const size_t offset = (candidate * trajectory_steps + step) * 3;
            auto* pose = path.add_poses();
            *pose->mutable_header() = path.header();
            pose->mutable_pose()->mutable_position()->set_x(
                trajectory_data[offset]);
            pose->mutable_pose()->mutable_position()->set_y(
                trajectory_data[offset + 1]);
            pose->mutable_pose()->mutable_position()->set_z(0.0);
            const double half_yaw =
                static_cast<double>(trajectory_data[offset + 2]) * 0.5;
            pose->mutable_pose()->mutable_orientation()->set_x(0.0);
            pose->mutable_pose()->mutable_orientation()->set_y(0.0);
            pose->mutable_pose()->mutable_orientation()->set_z(
                std::sin(half_yaw));
            pose->mutable_pose()->mutable_orientation()->set_w(
                std::cos(half_yaw));
        }
        decoded_paths.push_back(std::move(path));
    }

    *paths = std::move(decoded_paths);
    scores->assign(score_data, score_data + candidate_count);
    return true;
}

}  // namespace

PolicyRunner::PolicyRunner(
    std::vector<common::network::ModelTensorInfo> input_infos,
    std::vector<common::network::ModelTensorInfo> output_infos)
    : input_infos_(std::move(input_infos)),
      output_infos_(std::move(output_infos)) {}

YopoPolicy::YopoPolicy(proto::ShadowOptions options,
                       std::unique_ptr<PolicyRunner> runner)
    : options_(std::move(options)), runner_(std::move(runner)) {}

std::unique_ptr<YopoPolicy> YopoPolicy::Create(
    const proto::ShadowOptions& options, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (!ValidateShadowOptions(options, error)) {
        return nullptr;
    }

    common::network::InferenceOptions inference_options;
    inference_options.backend_id = options.policy_backend();
    inference_options.model_path = options.policy_model_path();
    std::string engine_error;
    auto engine =
        common::network::Engine::CreateEngine(inference_options, &engine_error);
    if (engine == nullptr) {
        SetError(error, engine_error);
        return nullptr;
    }
    return Create(options,
                  std::make_unique<EnginePolicyRunner>(std::move(engine)),
                  error);
}

std::unique_ptr<YopoPolicy> YopoPolicy::Create(
    const proto::ShadowOptions& options, std::unique_ptr<PolicyRunner> runner,
    std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (!ValidateShadowOptions(options, error)) {
        return nullptr;
    }
    if (runner == nullptr) {
        SetError(error, "policy runner is null.");
        return nullptr;
    }
    if (!ValidateModelContract(runner->input_infos_, runner->output_infos_,
                               options, error)) {
        return nullptr;
    }
    return std::unique_ptr<YopoPolicy>(
        new YopoPolicy(options, std::move(runner)));
}

bool YopoPolicy::Generate(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::nav_msgs::Odometry& odometry,
    const automsgs::msgs::geometry_msgs::PoseStamped& target,
    const automsgs::msgs::geometry_msgs::TwistStamped& target_velocity,
    std::vector<automsgs::msgs::nav_msgs::Path>* paths,
    std::vector<float>* scores, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (paths != nullptr) {
        paths->clear();
    }
    if (scores != nullptr) {
        scores->clear();
    }
    if (paths == nullptr || scores == nullptr) {
        SetError(error, "path and score outputs must not be null.");
        return false;
    }

    common::network::Tensor depth_tensor;
    common::network::Tensor robot_tensor;
    common::network::Tensor target_tensor;
    if (!PrepareDepth(options_, depth, &depth_tensor, error) ||
        !PrepareState(options_, odometry, target, target_velocity,
                      &robot_tensor, &target_tensor, error)) {
        return false;
    }
    common::network::TensorMap inputs;
    inputs.emplace(kDepthName, std::move(depth_tensor));
    inputs.emplace(kRobotStateName, std::move(robot_tensor));
    inputs.emplace(kTargetStateName, std::move(target_tensor));

    common::network::TensorMap outputs;
    std::string detail;
    if (!runner_->Run(inputs, &outputs, &detail)) {
        SetError(error, detail);
        return false;
    }
    return DecodeOutputs(options_, depth, outputs, paths, scores, error);
}

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
