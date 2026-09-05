/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file refiner.hpp
 * @brief Backend-independent RGB-D refinement over automsgs sensor messages.
 */

#ifndef AUTONOMY_PERCEPTION_FATHOM_DEPTH_REFINER_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_DEPTH_REFINER_HPP_

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/perception/fathom/config.hpp"

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @brief Backend-independent inference contract used by DepthRefiner.
 *
 * Concrete runtime
 * adapters, such as FathomEngine, implement this contract outside the
 * refiner so fake-runner processing remains available without ONNX Runtime.
 */
class FathomModelRunner
{
public:
    virtual ~FathomModelRunner() = default;

    /**
     * @brief Executes one inference request.
     * @param inputs Named model input tensors.
     * @param outputs Named model output tensors, populated on success.
     * @param error Optional diagnostic output.
     * @return True when inference succeeds.
     */
    virtual bool Run(const common::network::TensorMap& inputs,
                     common::network::TensorMap* outputs,
                     std::string* error = nullptr) = 0;
};

/**
 * @brief Converts aligned RGB-D messages to model inputs and automsgs outputs.
 *
 * Construction accepts the model-runner contract so this facade remains
 * independent from any concrete inference backend.
 */
class DepthRefiner
{
public:
    /**
     * @brief Creates a refiner from validated options and an inference runner.
     * @param config Fixed deployment profile.
     * @param runner Concrete or test inference runner; ownership transfers.
     * @param error Optional diagnostic output, cleared on entry.
     * @return A ready refiner, or nullptr when validation fails.
     */
    static std::unique_ptr<DepthRefiner> Create(
        const FathomConfig& config, std::unique_ptr<FathomModelRunner> runner,
        std::string* error = nullptr);

    DepthRefiner(const DepthRefiner&) = delete;
    DepthRefiner& operator=(const DepthRefiner&) = delete;
    DepthRefiner(DepthRefiner&&) = delete;
    DepthRefiner& operator=(DepthRefiner&&) = delete;

    /**
     * @brief Refines one aligned RGB-D frame and projects valid metric depth.
     *
     * Output messages are cleared before work begins and assigned only after
     * all processing, inference, and projection steps succeed.
     *
     * @param rgb Aligned `rgb8` or `bgr8` image.
     * @param raw_depth Aligned `16UC1` or `32FC1` depth image.
     * @param camera_info Intrinsics matching the original image dimensions.
     * @param refined_depth Output `32FC1` metric depth image.
     * @param point_cloud Output organized XYZ point cloud.
     * @param error Optional diagnostic output, cleared on entry.
     * @return True when preprocessing, inference, and projection all succeed.
     */
    bool Refine(const automsgs::msgs::sensor_msgs::Image& rgb,
                const automsgs::msgs::sensor_msgs::Image& raw_depth,
                const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                automsgs::msgs::sensor_msgs::Image* refined_depth,
                automsgs::msgs::sensor_msgs::PointCloud2* point_cloud,
                std::string* error = nullptr);

private:
    DepthRefiner(FathomConfig config,
                 std::unique_ptr<FathomModelRunner> runner);

    FathomConfig config_;
    std::unique_ptr<FathomModelRunner> runner_;
};

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_DEPTH_REFINER_HPP_
