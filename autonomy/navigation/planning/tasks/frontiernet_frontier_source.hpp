/*
 * Copyright 2026 The Navigation Authors (navigation)
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
 * @file frontiernet_frontier_source.hpp
 * @brief Depth-only frontier waypoints from a FrontierNet prediction.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/perception/depth_preprocessor.hpp"
#include "navigation/perception/frontiernet_model.hpp"
#include "navigation/planning/base/waypoint.hpp"
#include "navigation/proto/config.pb.h"
#include "navigation/proto/observation.pb.h"

namespace navigation
{
namespace planning
{

/** @brief Decoding and selection knobs for the FrontierNet frontier source. */
struct FrontierNetFrontierOptions
{
    /** @brief Distance-field normalizer used by the checkpoint (metres). */
    double df_normalizer{10.0};
    /** @brief Frontier distance threshold in metres. */
    double df_threshold{0.5};
    /** @brief Waypoints farther than this are dropped (<=0 disables). */
    double max_range{6.0};
    /** @brief Minimum predicted gain in m^3 for a pixel to count. */
    double min_gain{0.0};
    /** @brief Smallest image-space cluster worth a trip. */
    std::uint32_t min_cluster_pixels{12};
    /** @brief Waypoints are clamped to this distance ahead of the robot. */
    double target_distance{2.0};
    /** @brief Depth range that can be back-projected. */
    double min_depth{0.15};
    /** @brief Upper bound of back-projected depth. */
    double max_depth{8.0};
    /** @brief Clusters closer than this are ignored (metres). */
    double min_forward_m{0.4};
    /** @brief Print one line per inference to stderr. */
    bool log_frontiers{false};
};

/** @brief Per-frame diagnostics of one detection. */
struct FrontierNetFrontierStats
{
    /** @brief True when the ONNX graph is loaded. */
    bool model_ready{false};
    /** @brief True when the depth frame was usable. */
    bool depth_ok{false};
    /** @brief Pixels the distance field marked as frontier. */
    std::size_t frontier_pixels{0};
    /** @brief Image-space clusters kept as waypoints. */
    std::size_t clusters{0};
    /** @brief Wall time of the forward pass in milliseconds. */
    double inference_ms{0.0};
};

/** @brief Pinhole intrinsics already scaled to the model resolution. */
struct FrontierCameraModel
{
    /** @brief Focal length along x in model pixels. */
    double fx{0.0};
    /** @brief Focal length along y in model pixels. */
    double fy{0.0};
    /** @brief Principal point along x in model pixels. */
    double cx{0.0};
    /** @brief Principal point along y in model pixels. */
    double cy{0.0};
};

/**
 * @brief Binary frontier region from the predicted distance field.
 *
 * @param[in] df ``size * size`` values of ``-log(d / normalizer)``.
 * @param[in] normalizer Distance-field normalizer in metres.
 * @param[in] threshold Frontier distance threshold in metres.
 */
std::vector<std::uint8_t> RegionFromDf(const std::vector<float>& df,
                                       double normalizer, double threshold);

/**
 * @brief Per-pixel gain in m^3 from the class logits.
 *
 * Label 0 means "no frontier" and maps to zero gain; every other label maps to
 * the centre of its log1p gain bin, exactly like the official decoder.
 */
std::vector<float> GainFromMasks(const std::vector<float>& masks,
                                 std::uint32_t classes, std::uint32_t size);

/**
 * @brief Back-project one model pixel to the map frame.
 *
 * The camera is treated as an optical frame (x right, y down, z forward) rigidly
 * attached to the base link, which matches the autosim depth camera.
 *
 * @return false when the depth is outside the configured range.
 */
bool ProjectPixelToMap(std::uint32_t u, std::uint32_t v, float depth_m,
                       const FrontierCameraModel& camera,
                       const common::Pose2D& robot_pose, double min_depth,
                       double max_depth, double* map_x, double* map_y);

/**
 * @brief Contract the exploration policy needs from a depth frontier detector.
 *
 * @details The policy only asks whether a detector is usable and for the
 * waypoints of one observation, so unit tests can substitute a fake detector
 * and exercise the map-then-model fallback without an ONNX graph.
 */
class FrontierSourceInterface
{
public:
    virtual ~FrontierSourceInterface() = default;

    /** @brief True when the detector can produce waypoints. */
    [[nodiscard]] virtual bool Ready() const = 0;

    /** @brief Frontier waypoints for one observation, nearest first. */
    virtual std::vector<Waypoint> Detect(
        const observation::Observation& observation,
        const common::Pose2D& robot_pose) = 0;
};

/**
 * @brief FrontierNet depth -> frontier waypoints in the map frame.
 *
 * @details One forward pass per call: DeFM preprocessing, ONNX inference,
 * distance-field thresholding, gain decoding, image-space clustering and
 * back-projection. Waypoints are sorted by distance, nearest first.
 */
class FrontierNetFrontierSource : public FrontierSourceInterface
{
public:
    /** @brief Construct with decoding options. */
    explicit FrontierNetFrontierSource(FrontierNetFrontierOptions options = {});

    /**
     * @brief Load the ONNX graph and adopt the manifest's preprocessing stats.
     *
     * @param[in] manifest Model manifest (path, sha256, shapes, mean, std).
     * @param[out] error Filled on failure when non-null.
     */
    bool LoadModel(const config::ModelManifest& manifest, std::string* error);

    /** @brief True when the graph is loaded. */
    [[nodiscard]] bool Ready() const override;

    /**
     * @brief Detect frontier waypoints for one observation.
     *
     * @param[in] observation Depth, camera info and odometry.
     * @param[in] robot_pose Current planar pose in the map frame.
     * @param[out] stats Optional per-frame diagnostics.
     */
    std::vector<Waypoint> Detect(const observation::Observation& observation,
                                 const common::Pose2D& robot_pose) override;

    /**
     * @brief Detect with per-frame diagnostics.
     *
     * @param[in] observation Depth, camera info and odometry.
     * @param[in] robot_pose Current planar pose in the map frame.
     * @param[out] stats Optional per-frame diagnostics.
     */
    std::vector<Waypoint> Detect(const observation::Observation& observation,
                                 const common::Pose2D& robot_pose,
                                 FrontierNetFrontierStats* stats);

    /** @brief Decoding options in effect. */
    [[nodiscard]] const FrontierNetFrontierOptions& Options() const;

private:
    /** @brief Decoding and selection knobs. */
    FrontierNetFrontierOptions options_;
    /** @brief DeFM normalization matching the exported graph. */
    perception::DepthPreprocessor preprocessor_;
    /** @brief ONNX FrontierNet graph. */
    std::unique_ptr<perception::FrontierNetModel> model_;
};

}  // namespace planning
}  // namespace navigation
