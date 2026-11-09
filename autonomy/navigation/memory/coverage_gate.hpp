/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file coverage_gate.hpp
 * @brief Paper coverage keyframe gate: keep a frame iff eta patches are unseen.
 */

#pragma once

#include <cstdint>
#include <unordered_set>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/proto/observation.pb.h"

namespace navigation
{
namespace memory
{

/** @brief One frame's lookup keys (unseen test) and claim keys (if kept). */
struct CoverageFrame
{
    /** @brief Per-patch neighbourhood keys; -1 means unused. */
    std::vector<std::int64_t> lookup;
    /** @brief Number of valid patches. */
    int patches{0};
    /** @brief Neighbour keys stored per patch. */
    int neighbors{0};
    /** @brief Cells the frame claims when kept. */
    std::vector<std::uint64_t> claims;
};

/**
 * @brief Coverage cells from a depth image and ROS planar camera pose.
 *
 * @param[in] observation Depth + optional CameraInfo.
 * @param[in] ros_pose Map-frame robot/camera planar pose.
 * @param[in] camera_height_m Camera height used to build Habitat c2w.
 */
CoverageFrame FrameCells(const observation::Observation& observation,
                         const common::Pose2D& ros_pose,
                         double camera_height_m = 0.6);

/**
 * @brief Claimed-cell set for one spatial context, matching VNT-PA eta.
 */
class CoverageGate
{
public:
    /** @brief Construct with the paper keep threshold (default 10 patches). */
    explicit CoverageGate(int eta = 10);

    /**
     * @brief Keep the frame when at least eta patches are unseen.
     *
     * @param[in] frame Coverage cells from FrameCells.
     * @param[in] force Keep unconditionally (first frame of a map).
     */
    bool Offer(const CoverageFrame& frame, bool force = false);

    /** @brief Drop every claimed cell. */
    void Reset();

    /** @brief Paper eta (unseen patch count). */
    [[nodiscard]] int Eta() const { return eta_; }

private:
    int eta_{10};
    std::unordered_set<std::uint64_t> claimed_;
};

}  // namespace memory
}  // namespace navigation
