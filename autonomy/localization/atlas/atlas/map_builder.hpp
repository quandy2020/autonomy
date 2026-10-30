/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "atlas/sensor/sensor_types.hpp"

namespace autonomy::localization::atlas {

namespace internal {

using MappingPoseCache =
    std::unordered_map<std::uint64_t, Eigen::Isometry3d>;

struct MappingCandidateEvictionResult {
    std::size_t unprotected_candidates = 0;
    bool saturated = false;
};

struct MappingCandidateSlotResult {
    MappingCandidateEvictionResult eviction;
    bool have_slot = false;
    bool incoming_is_keyframe = false;
};

struct MappingCandidateBufferResult {
    MappingCandidateEvictionResult eviction;
    bool soft_capacity_exceeded = false;
};

std::unordered_set<std::int64_t> UnsubmittedMappingKeyframeKeys(
    const std::vector<TrajectoryPose>& trajectory,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids);

void ProtectPendingMappingKeyframes(
    const std::vector<double>& pending_timestamps,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids,
    std::unordered_set<std::int64_t>* protected_keys);

std::size_t DiscardStaleMappingCandidates(
    double oldest_useful_time,
    const std::unordered_set<std::int64_t>& protected_keys,
    std::deque<map::MappingFrame>* pending_frames);

MappingCandidateSlotResult PrepareMappingCandidateSlot(
    std::size_t maximum_pending, double incoming_timestamp,
    const std::unordered_set<std::int64_t>& protected_keys,
    std::deque<map::MappingFrame>* pending_frames);

/**
 * Buffers one candidate without blocking the tracking thread. The configured
 * capacity is a soft limit only while every resident candidate is protected;
 * the next keyframe snapshot reclassifies speculative overflow candidates.
 */
MappingCandidateBufferResult BufferMappingCandidateWithSoftCapacity(
    std::size_t maximum_pending,
    const std::unordered_set<std::int64_t>& protected_keys,
    map::MappingFrame frame,
    std::deque<map::MappingFrame>* pending_frames);

MappingCandidateEvictionResult EnforceMappingCandidateCapacity(
    std::size_t maximum_pending,
    const std::unordered_set<std::int64_t>& protected_keys,
    std::deque<map::MappingFrame>* pending_frames);

/** Selects cumulative Local-BA pose changes without advancing state. */
std::vector<map::TimedPose> SelectChangedMappingPoses(
    const std::vector<TrajectoryPose>& trajectory,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids,
    const MappingPoseCache& last_sent_poses,
    double translation_threshold, double rotation_threshold_radians,
    bool force);

/** Advances pose-sync state only after MappingPipeline accepted the batch. */
void RecordSentMappingPoses(const std::vector<map::TimedPose>& poses,
                            MappingPoseCache* last_sent_poses);

struct MapIdentitySnapshot {
    bool valid = false;
    std::uint64_t map_id = 0;
    std::uint64_t coordinate_epoch = 0;
    std::uint64_t destructive_epoch = 0;
};

/** True only for an in-flight non-destructive coordinate rebase. */
bool CrossedScaledCoordinateEpoch(const MapIdentitySnapshot& before,
                                  const MapIdentitySnapshot& after);

/** Reports a valid map-id or destructive coordinate-epoch transition. */
bool ObserveMappingMap(bool map_id_valid, std::uint64_t map_id,
                       std::uint64_t destructive_epoch,
                       bool* observed_map_id_valid,
                       std::uint64_t* observed_map_id,
                       std::uint64_t* observed_destructive_epoch);

struct CulledMappingFrame {
    std::int64_t timestamp_key = 0;
    std::uint64_t frame_id = 0;
};

/** Selects frames absent from both committed and tentative KF snapshots. */
std::vector<CulledMappingFrame> SelectCulledMappingFrames(
    const std::vector<TrajectoryPose>& committed_trajectory,
    const std::vector<double>& pending_timestamps,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids,
    std::size_t missing_refresh_threshold,
    std::unordered_map<std::int64_t, std::size_t>* missing_refresh_counts);

}  // namespace internal

/**
 * @brief High-level owner and façade for an ORB-SLAM3 tracking session.
 *
 * MapBuilder validates sensor ordering, owns the SLAM worker threads, and
 * exposes poses in the world-from-camera convention. Unless stated otherwise,
 * members are safe to call only from the thread driving the sensor stream.
 */
class MapBuilder
{
public:
    /** @brief Creates a stopped session from the supplied configuration. */
    explicit MapBuilder(SystemConfig config);
    ~MapBuilder();

    MapBuilder(const MapBuilder&) = delete;
    MapBuilder& operator=(const MapBuilder&) = delete;

    /** @brief Starts the SLAM runtime. @return true when ready for input. */
    bool Start();
    /** @brief Stops workers; repeated calls are harmless. */
    void Stop();
    /** @return Whether the runtime has been started and not stopped. */
    bool started() const;

    /** @brief Queues one IMU sample. @return false for invalid ordering/data.
     */
    bool AddImu(const ImuSample& sample);
    /** @brief Tracks a monocular frame. */
    bool TrackMonocular(const ImageFrame& image,
                        TrackingResult* result = nullptr);
    /** @brief Tracks a timestamp-aligned stereo pair. */
    bool TrackStereo(const ImageFrame& left, const ImageFrame& right,
                     TrackingResult* result = nullptr);
    /** @brief Tracks a timestamp-aligned color and depth pair. */
    bool TrackRgbd(const ImageFrame& color, const ImageFrame& depth,
                   TrackingResult* result = nullptr);

    /** @brief Copies the most recent tracking result into @p result. */
    bool LatestResult(TrackingResult* result) const;
    /** @return Current non-null tracked map points in world coordinates. */
    std::vector<Eigen::Vector3f> TrackedMapPoints() const;
    /** @return Annotated latest camera image with detected/matched features. */
    cv::Mat TrackingImage() const;
    /** @return Previous/current images joined by temporal feature matches. */
    cv::Mat FeatureMatchesImage() const;
    /** @return Current optimized keyframe trajectory. */
    std::vector<TrajectoryPose> OptimizedKeyFrameTrajectory() const;
    /** @return Unique loop-closure constraints in world coordinates. */
    std::vector<LoopClosureEdge> LoopClosureEdges() const;
    /** @return Latest immutable dense-cloud and occupancy-grid snapshot. */
    std::shared_ptr<const map::MapSnapshot> LatestMappingSnapshot() const;

    /** @brief Stops map growth while retaining localization. */
    void ActivateLocalizationMode();
    /** @brief Re-enables local mapping. */
    void DeactivateLocalizationMode();
    /** @brief Clears tracker state and starts a fresh active map. */
    void Reset();

    /** @brief Stops the runtime and writes a TUM-format camera trajectory. */
    bool ShutdownAndSaveTumTrajectory(const std::string& path);
    /** @brief Stops the runtime and writes a EuRoC-format camera trajectory. */
    bool ShutdownAndSaveEurocTrajectory(const std::string& path);
    /** @brief Stops the runtime and writes a KITTI-format camera trajectory. */
    bool ShutdownAndSaveKittiTrajectory(const std::string& path);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy::localization::atlas
