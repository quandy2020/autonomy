/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "atlas/map/dense_map.hpp"
#include "atlas/map/ground_classifier.hpp"
#include "atlas/map/map_types.hpp"
#include "atlas/map/occupancy_grid_map.hpp"

namespace autonomy::localization::atlas::map {

namespace internal {

enum class MappingWorkerWork {
    kNone,
    kIdleSnapshot,
    kReset,
    kForcedRebuild,
    kFrame,
    kFrameRemoval,
    kPoseUpdate,
    kFlush,
};

/** Selects work without mutating state; order is part of the scheduler API. */
MappingWorkerWork SelectMappingWorkerWork(
    bool idle_snapshot, bool reset_requested, bool rebuild_requested,
    bool have_queued_frame_removals, bool have_queued_frames,
    bool have_frame_removals, bool have_pose_updates, bool flush_pending);

}  // namespace internal

/**
 * Non-blocking producer/background-consumer pipeline for visual dense maps.
 *
 * Submit only moves ref-counted cv::Mat storage into a bounded queue. Image
 * projection, stereo matching, ray tracing, rebuilding and snapshot copies all
 * run on the worker thread.
 */
class MappingPipeline {
public:
    enum class SubmitResult {
        kAccepted,
        kQueueFull,
        kInvalid,
        kStopped,
    };

    explicit MappingPipeline(MappingPipelineOptions options);
    ~MappingPipeline();

    MappingPipeline(const MappingPipeline&) = delete;
    MappingPipeline& operator=(const MappingPipeline&) = delete;

    bool Start();
    void Stop();
    bool running() const;

    /**
     * Convenience ownership-transferring submission. Returns false instead of
     * dropping an already accepted frame when the bounded queue is full.
     * Call TrySubmit when the caller must retain a frame for retry.
     */
    bool Submit(MappingFrame frame,
                std::uint64_t* assigned_frame_id = nullptr);
    /**
     * Non-blocking, lossless bounded submission. kAccepted moves *frame into
     * the worker queue and optionally returns its stable id. kQueueFull leaves
     * *frame and its id unchanged for retry. kInvalid is permanent for the
     * supplied frame, while kStopped means the worker cannot accept work.
     * An accepted frame is never displaced by a later submission.
     */
    SubmitResult TrySubmit(MappingFrame* frame,
                           std::uint64_t* assigned_frame_id = nullptr);

    /**
     * Supplies loop-closure/BA-corrected poses. A material pose change causes
     * a background rebuild from cached camera-coordinate local clouds.
     */
    bool UpdatePoses(std::vector<TimedPose> poses);
    /**
     * Queues a bounded idempotent culling batch without blocking tracking.
     * The returned ids were admitted; omitted ids may be retried later.
     */
    std::vector<std::uint64_t> RemoveFrames(
        std::vector<std::uint64_t> frame_ids);
    /**
     * Drains worker completion records. Callers must retain their frame-id
     * metadata until an id appears in removed_frame_ids. A retained id was
     * deliberately left in the last valid map and may be retried later.
     */
    MappingFrameRemovalResults TakeFrameRemovalResults();
    /** Forces a rebuild even when pose changes are below the threshold. */
    bool RequestRebuild();
    /** Clears queued input, local-cloud history and both global maps. */
    void Reset();
    /**
     * Blocks until work already visible to the worker has been consumed and
     * its final immutable snapshot has been published. Returns false when the
     * worker is not running or shutdown starts before the barrier completes.
     */
    bool Flush();

    std::shared_ptr<const MapSnapshot> LatestSnapshot() const;
    MappingPipelineStats stats() const;

    /** Projects to camera coordinates; exposed for deterministic unit tests. */
    bool ProjectFrame(
        const MappingFrame& frame,
        std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>* points)
        const;

private:
    struct CachedFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::uint64_t frame_id = 0;
        double timestamp = 0.0;
        Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
        std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>
            camera_points;
        /** Retains image references needed to re-triangulate monocular data. */
        std::shared_ptr<MappingFrame> source_frame;
        /** True while point/image payloads are resident instead of archived. */
        bool payload_resident = true;
        /** Exact per-frame archive created inside this instance's directory. */
        bool archived = false;
        std::string archive_path;
        std::uint64_t archive_bytes = 0;
    };

    using CachedFrameList =
        std::list<CachedFrame, Eigen::aligned_allocator<CachedFrame>>;
    using CachedFrameIterator = CachedFrameList::iterator;
    using PointVector =
        std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>;

    bool ValidateFrame(const MappingFrame& frame) const;
    bool AssignFrameId(MappingFrame* frame);
    bool ProjectAndFilter(
        const MappingFrame& frame,
        std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>* points)
        const;
    bool PassesMotionGate(const MappingFrame& frame) const;
    void ProcessFrame(MappingFrame frame);
    bool EnsureHistoryCacheDirectory();
    bool WriteArchivedPayload(CachedFrame* cached, const PointVector& points,
                              const MappingFrame* source_frame);
    bool ReadArchivedPayload(const CachedFrame& cached, PointVector* points,
                             MappingFrame* source_frame) const;
    void DropResidentPayload(CachedFrame* cached);
    void RemoveArchivedPayload(CachedFrame* cached);
    void EnforceResidentCacheLimit();
    void CleanupHistoryCache();
    void MarkHistoryIncomplete();
    void ApplyPoseUpdates(std::vector<TimedPose> poses, bool force_rebuild);
    MappingFrameRemovalResults ApplyFrameRemovals(
        std::vector<std::uint64_t> frame_ids,
        std::vector<std::uint64_t> removed_queued_frame_ids);
    void StoreOptimizedPose(std::uint64_t frame_id,
                            const Eigen::Isometry3d& world_from_camera);
    void PruneOptimizedPoses();
    /** Clears all worker-owned mapping state and publishes an empty snapshot. */
    void ResetWorkerState();
    bool Rebuild(
        double timestamp,
        const std::unordered_set<std::uint64_t>& excluded_frame_ids = {});
    void PublishSnapshot(double timestamp, bool force);
    void WorkerLoop();
    PointVector TransformPoints(
        const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
            camera_points,
        const Eigen::Isometry3d& world_from_camera) const;

    MappingPipelineOptions options_;
    DenseMap dense_map_;
    DenseMap navigation_map_;
    GroundClassifier ground_classifier_;
    OccupancyGridMap occupancy_grid_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<MappingFrame> queue_;
    std::vector<TimedPose> pending_poses_;
    std::unordered_set<std::uint64_t> pending_frame_removals_;
    std::unordered_set<std::uint64_t> processing_frame_removals_;
    std::unordered_set<std::uint64_t> completed_frame_removals_;
    std::unordered_set<std::uint64_t> retained_frame_removals_;
    bool stop_requested_ = false;
    bool reset_requested_ = false;
    bool rebuild_requested_ = false;
    bool running_ = false;
    std::uint64_t reset_request_id_ = 0;
    std::uint64_t reset_completed_id_ = 0;
    std::uint64_t flush_request_id_ = 0;
    std::uint64_t flush_completed_id_ = 0;
    std::thread worker_;

    mutable std::mutex snapshot_mutex_;
    std::shared_ptr<const MapSnapshot> snapshot_;

    CachedFrameList cache_;
    std::unordered_map<std::uint64_t, CachedFrameIterator> cached_frames_by_id_;
    std::deque<std::uint64_t> resident_frame_order_;
    std::unordered_map<std::uint64_t, Eigen::Isometry3d> optimized_poses_;
    std::deque<std::uint64_t> optimized_pose_order_;
    std::unordered_multimap<std::uint64_t, std::uint64_t>
        monocular_frames_by_reference_pose_id_;
    std::string history_cache_directory_;
    std::uint64_t history_cache_bytes_ = 0;
    std::uint64_t next_archive_sequence_ = 0;
    bool history_cache_directory_created_ = false;
    bool cache_contains_complete_history_ = true;
    bool have_last_pose_ = false;
    Eigen::Isometry3d last_pose_ = Eigen::Isometry3d::Identity();
    bool have_navigation_center_ = false;
    Eigen::Vector3f navigation_center_ = Eigen::Vector3f::Zero();
    double last_frame_time_ = 0.0;
    double last_snapshot_time_ = -1.0;
    std::uint64_t generation_ = 0;
    std::uint64_t last_published_generation_ = 0;

    std::atomic<std::uint64_t> next_frame_id_{1};
    std::atomic<std::uint64_t> submitted_frames_{0};
    std::atomic<std::uint64_t> dropped_frames_{0};
    std::atomic<std::uint64_t> backpressured_frames_{0};
    std::atomic<std::uint64_t> reset_discarded_frames_{0};
    std::atomic<std::uint64_t> gated_frames_{0};
    std::atomic<std::uint64_t> processed_frames_{0};
    std::atomic<std::uint64_t> rejected_frames_{0};
    std::atomic<std::uint64_t> removed_frames_{0};
    std::atomic<std::uint64_t> removal_rebuild_failures_{0};
    std::atomic<std::uint64_t> duplicate_frames_{0};
    std::atomic<std::uint64_t> retriangulated_frames_{0};
    std::atomic<std::uint64_t> skipped_incomplete_rebuilds_{0};
    std::atomic<std::uint64_t> history_archive_failures_{0};
};

}  // namespace autonomy::localization::atlas::map
