/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/mapping_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

namespace autonomy::localization::atlas::map {
namespace {
namespace fs = std::filesystem;

class ScopedTestDirectory {
public:
    explicit ScopedTestDirectory(const std::string& label) {
        const auto token =
            std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = fs::temp_directory_path() /
                ("atlas_mapping_pipeline_" + label + "_" +
                 std::to_string(token));
        std::error_code error;
        EXPECT_TRUE(fs::create_directory(path_, error));
        EXPECT_FALSE(error);
    }

    ~ScopedTestDirectory() {
        std::error_code error;
        fs::remove_all(path_, error);
    }

    const fs::path& path() const { return path_; }

    std::size_t EntryCount() const {
        std::error_code error;
        const auto count = static_cast<std::size_t>(
            std::distance(fs::directory_iterator(path_, error),
                          fs::directory_iterator()));
        EXPECT_FALSE(error);
        return count;
    }

private:
    fs::path path_;
};

MappingFrame MakeRgbdFrame(std::uint64_t id, double timestamp) {
    MappingFrame frame;
    frame.frame_id = id;
    frame.timestamp = timestamp;
    frame.camera.fx = 1.0;
    frame.camera.fy = 1.0;
    frame.camera.cx = 0.0;
    frame.camera.cy = 0.0;
    frame.camera.depth_scale = 1000.0;
    frame.color = cv::Mat(1, 1, CV_8UC3, cv::Scalar(3, 2, 1)).clone();
    frame.depth = cv::Mat(1, 1, CV_16UC1, cv::Scalar(1000)).clone();
    return frame;
}

MappingFrame MakeTemporalFrame(double baseline, int disparity_pixels) {
    constexpr int kWidth = 192;
    constexpr int kHeight = 96;
    MappingFrame frame;
    frame.timestamp = 1.0;
    frame.frame_id = 2;
    frame.reference_frame_id = 1;
    frame.depth_source = DepthSource::kMonocularTemporal;
    frame.color_encoding = ColorEncoding::kMono;
    frame.camera.fx = 80.0;
    frame.camera.fy = 80.0;
    frame.camera.cx = kWidth / 2.0;
    frame.camera.cy = kHeight / 2.0;
    frame.camera.depth_scale = 1.0;
    frame.reference_from_camera.translation().x() = -baseline;
    frame.color.create(kHeight, kWidth, CV_8UC1);
    cv::RNG random(17);
    random.fill(frame.color, cv::RNG::UNIFORM, 0, 255);
    frame.reference = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
    if (disparity_pixels > 0 && disparity_pixels < kWidth) {
        frame.color(cv::Rect(disparity_pixels, 0,
                             kWidth - disparity_pixels, kHeight))
            .copyTo(frame.reference(cv::Rect(0, 0,
                                             kWidth - disparity_pixels,
                                             kHeight)));
    }
    return frame;
}

MappingFrame MakeStereoFrame(CameraProjection projection) {
    MappingFrame frame = MakeTemporalFrame(0.1, 8);
    frame.depth_source = DepthSource::kStereo;
    frame.right = frame.reference;
    frame.reference.release();
    frame.reference_frame_id = 0;
    frame.camera.projection = projection;
    frame.secondary_camera = frame.camera;
    frame.secondary_from_camera = frame.reference_from_camera;
    frame.camera.stereo_baseline =
        projection == CameraProjection::kRectified ? 0.1 : 0.0;
    return frame;
}

std::shared_ptr<const MapSnapshot> WaitForGeneration(
    const MappingPipeline& pipeline, std::uint64_t minimum_generation) {
    for (int attempt = 0; attempt < 600; ++attempt) {
        auto snapshot = pipeline.LatestSnapshot();
        if (snapshot && snapshot->dense_map.generation >= minimum_generation)
            return snapshot;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return nullptr;
}

bool WaitForHandledFrames(const MappingPipeline& pipeline,
                          std::uint64_t expected) {
    for (int attempt = 0; attempt < 600; ++attempt) {
        const MappingPipelineStats stats = pipeline.stats();
        if (stats.processed_frames + stats.gated_frames +
                stats.rejected_frames >=
            expected) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

TEST(MappingPipelineTest,
     WorkerPrioritizesExplicitRebuildAndFramesOverMaintenance) {
    using Work = internal::MappingWorkerWork;
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  true, true, true, true, true, true, true, true),
              Work::kReset);
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  false, false, true, true, true, true, true, true),
              Work::kForcedRebuild);
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  false, false, false, true, true, true, true, true),
              Work::kFrameRemoval);
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  false, false, false, false, true, true, true, true),
              Work::kFrame);
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  false, false, false, false, false, true, true, true),
              Work::kFrameRemoval);
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  false, false, false, false, false, false, true, true),
              Work::kPoseUpdate);
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  false, false, false, false, false, false, false, true),
              Work::kFlush);
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  false, false, false, false, false, false, false, false),
              Work::kNone);
}

TEST(MappingPipelineTest, IdleSnapshotWinsOnlyWhenWaitTimedOut) {
    using Work = internal::MappingWorkerWork;
    EXPECT_EQ(internal::SelectMappingWorkerWork(
                  true, false, false, false, false, false, false, false),
              Work::kIdleSnapshot);
}

TEST(MappingPipelineTest, ProjectsMetricRgbdAndBgrColor) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    MappingPipeline pipeline(options);
    MappingFrame frame = MakeRgbdFrame(1, 1.0);
    frame.color =
        cv::Mat(2, 2, CV_8UC3, cv::Scalar(30, 20, 10)).clone();
    frame.depth = cv::Mat(2, 2, CV_16UC1, cv::Scalar(1000)).clone();

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    ASSERT_TRUE(pipeline.ProjectFrame(frame, &points));
    ASSERT_EQ(points.size(), 4U);
    EXPECT_TRUE(points[3].position.isApprox(Eigen::Vector3f(1, 1, 1)));
    EXPECT_EQ(points[0].red, 10);
    EXPECT_EQ(points[0].green, 20);
    EXPECT_EQ(points[0].blue, 30);
}

TEST(MappingPipelineTest, RejectsDegenerateTemporalBaseline) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    MappingPipeline pipeline(options);
    MappingFrame frame = MakeTemporalFrame(0.0, 8);
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    EXPECT_FALSE(pipeline.ProjectFrame(frame, &points));
}

TEST(MappingPipelineTest, ProjectsRectifiedTemporalMonocularPair) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 2;
    options.projection.minimum_depth = 0.3;
    options.projection.maximum_depth = 3.0;
    options.projection.stereo_number_of_disparities = 16;
    options.projection.stereo_block_size = 3;
    MappingPipeline pipeline(options);
    MappingFrame frame = MakeTemporalFrame(0.1, 8);
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    ASSERT_TRUE(pipeline.ProjectFrame(frame, &points));
    ASSERT_GT(points.size(), 100U);
    std::vector<float> depths;
    depths.reserve(points.size());
    for (const ColorPoint& point : points)
        depths.push_back(point.position.z());
    const auto median = depths.begin() + depths.size() / 2;
    std::nth_element(depths.begin(), median, depths.end());
    // fx * baseline / disparity = 80 * 0.1 / 8 = 1 metre.
    EXPECT_NEAR(*median, 1.0F, 0.15F);
}

TEST(MappingPipelineTest, RectifiesRawPinholeStereoPair) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 2;
    options.projection.minimum_depth = 0.3;
    options.projection.maximum_depth = 3.0;
    options.projection.stereo_number_of_disparities = 16;
    options.projection.stereo_block_size = 3;
    MappingPipeline pipeline(options);
    MappingFrame frame = MakeStereoFrame(CameraProjection::kPinhole);
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    ASSERT_TRUE(pipeline.ProjectFrame(frame, &points));
    ASSERT_GT(points.size(), 100U);
    std::vector<float> depths;
    for (const ColorPoint& point : points)
        depths.push_back(point.position.z());
    const auto median = depths.begin() + depths.size() / 2;
    std::nth_element(depths.begin(), median, depths.end());
    EXPECT_NEAR(*median, 1.0F, 0.15F);
}

TEST(MappingPipelineTest, SupportsKb8AndRejectsUnknownProjection) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 2;
    options.projection.minimum_depth = 0.3;
    options.projection.maximum_depth = 3.0;
    options.projection.stereo_number_of_disparities = 16;
    options.projection.stereo_block_size = 3;
    MappingPipeline pipeline(options);
    MappingFrame kb8 = MakeStereoFrame(CameraProjection::kKannalaBrandt8);
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    EXPECT_TRUE(pipeline.ProjectFrame(kb8, &points));
    EXPECT_GT(points.size(), 100U);

    MappingFrame unknown = MakeStereoFrame(CameraProjection::kUnknown);
    EXPECT_FALSE(pipeline.ProjectFrame(unknown, &points));
}

TEST(MappingPipelineTest, ProjectsRectifiedStereoPair) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 2;
    options.projection.minimum_depth = 0.3;
    options.projection.maximum_depth = 3.0;
    options.projection.stereo_number_of_disparities = 16;
    options.projection.stereo_block_size = 3;
    MappingPipeline pipeline(options);
    MappingFrame frame = MakeTemporalFrame(0.1, 8);
    frame.depth_source = DepthSource::kStereo;
    frame.camera.stereo_baseline = 0.1;
    frame.right = frame.reference;
    frame.reference.release();
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    ASSERT_TRUE(pipeline.ProjectFrame(frame, &points));
    ASSERT_GT(points.size(), 100U);
    std::vector<float> depths;
    depths.reserve(points.size());
    for (const ColorPoint& point : points)
        depths.push_back(point.position.z());
    const auto median = depths.begin() + depths.size() / 2;
    std::nth_element(depths.begin(), median, depths.end());
    EXPECT_NEAR(*median, 1.0F, 0.15F);
}

TEST(MappingPipelineTest, RebuildsCachedCloudAtOptimizedPoseAndResets) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.resolution = 0.5;
    options.occupancy_grid.maximum_extent = 10.0;
    options.occupancy_grid.maximum_cells = 1000;
    options.occupancy_grid.maximum_snapshot_cells = 1000;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(7, 1.0)));
    auto first = WaitForGeneration(pipeline, 1);
    ASSERT_NE(first, nullptr);
    ASSERT_EQ(first->dense_map.points.size(), 1U);
    EXPECT_NEAR(first->dense_map.points[0].position.x(), 0.0, 1e-5);

    TimedPose optimized;
    optimized.frame_id = 7;
    optimized.timestamp = 2.0;
    optimized.world_from_camera.translation().x() = 2.0;
    ASSERT_TRUE(pipeline.UpdatePoses({optimized}));
    auto rebuilt = WaitForGeneration(pipeline,
                                     first->dense_map.generation + 1U);
    ASSERT_NE(rebuilt, nullptr);
    ASSERT_EQ(rebuilt->dense_map.points.size(), 1U);
    EXPECT_NEAR(rebuilt->dense_map.points[0].position.x(), 2.0, 1e-5);

    pipeline.Reset();
    auto reset = WaitForGeneration(pipeline,
                                   rebuilt->dense_map.generation + 1U);
    ASSERT_NE(reset, nullptr);
    EXPECT_TRUE(reset->dense_map.points.empty());
    EXPECT_TRUE(reset->occupancy_grid.data.empty());
    pipeline.Stop();
}

TEST(MappingPipelineTest,
     ResetWaitsForInFlightFrameAndExhaustionPrecedesQueuePressure) {
    MappingPipelineOptions options;
    options.queue_capacity = 1;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 10000.0;
    options.dense_map.maximum_voxels = 16;
    options.maximum_navigation_voxels = 16;
    options.occupancy_grid.maximum_extent = 10.0;
    options.occupancy_grid.maximum_rays_per_frame = 1;
    options.occupancy_grid.ray_angle_bins = 1;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    MappingFrame in_flight = MakeRgbdFrame(
        std::numeric_limits<std::uint64_t>::max(), 1.0);
    constexpr int kImageSize = 768;
    in_flight.color = cv::Mat(kImageSize, kImageSize, CV_8UC3,
                              cv::Scalar(3, 2, 1))
                          .clone();
    in_flight.depth = cv::Mat(kImageSize, kImageSize, CV_16UC1,
                              cv::Scalar(1000))
                          .clone();
    ASSERT_TRUE(pipeline.Submit(std::move(in_flight)));

    // Once this explicit-id frame is accepted into the one-element queue,
    // the large first frame has been popped and is still being processed.
    MappingFrame queued = MakeRgbdFrame(1, 2.0);
    MappingPipeline::SubmitResult queued_result =
        MappingPipeline::SubmitResult::kQueueFull;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(5);
    while (queued_result == MappingPipeline::SubmitResult::kQueueFull &&
           std::chrono::steady_clock::now() < deadline) {
        queued_result = pipeline.TrySubmit(&queued);
        if (queued_result == MappingPipeline::SubmitResult::kQueueFull)
            std::this_thread::yield();
    }
    ASSERT_EQ(queued_result, MappingPipeline::SubmitResult::kAccepted);
    ASSERT_EQ(pipeline.stats().processed_frames, 0U);

    // Automatic ids are permanently exhausted. That must be reported as
    // invalid even though the bounded queue is simultaneously full.
    MappingFrame exhausted = MakeRgbdFrame(0, 3.0);
    std::uint64_t assigned = 123;
    EXPECT_EQ(pipeline.TrySubmit(&exhausted, &assigned),
              MappingPipeline::SubmitResult::kInvalid);
    EXPECT_EQ(exhausted.frame_id, 0U);
    EXPECT_EQ(assigned, 123U);

    pipeline.Reset();
    const auto reset = pipeline.LatestSnapshot();
    ASSERT_NE(reset, nullptr);
    EXPECT_TRUE(reset->dense_map.points.empty());
    EXPECT_TRUE(reset->occupancy_grid.data.empty());
    const std::uint64_t reset_generation = reset->dense_map.generation;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto stable = pipeline.LatestSnapshot();
    ASSERT_NE(stable, nullptr);
    EXPECT_EQ(stable->dense_map.generation, reset_generation);
    EXPECT_TRUE(stable->dense_map.points.empty());

    const MappingPipelineStats stats = pipeline.stats();
    EXPECT_EQ(stats.rejected_frames, 1U);
    EXPECT_EQ(stats.reset_discarded_frames, 1U);
    EXPECT_EQ(stats.dropped_frames, 0U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, AccumulatesSmallPoseUpdatesFromLastFusedPose) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.rebuild_translation_threshold = 0.05;
    options.rebuild_rotation_threshold_radians = 1.0;
    options.dense_map.voxel_size = 0.01;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(7, 1.0)));
    ASSERT_TRUE(pipeline.Flush());
    const auto initial = pipeline.LatestSnapshot();
    ASSERT_NE(initial, nullptr);

    TimedPose optimized;
    optimized.frame_id = 7;
    optimized.timestamp = 2.0;
    optimized.world_from_camera.translation().x() = 0.03;
    ASSERT_TRUE(pipeline.UpdatePoses({optimized}));
    ASSERT_TRUE(pipeline.Flush());
    const auto below_threshold = pipeline.LatestSnapshot();
    ASSERT_NE(below_threshold, nullptr);
    EXPECT_EQ(below_threshold->dense_map.generation,
              initial->dense_map.generation);
    ASSERT_EQ(below_threshold->dense_map.points.size(), 1U);
    EXPECT_NEAR(below_threshold->dense_map.points.front().position.x(), 0.0F,
                1e-5F);

    optimized.timestamp = 3.0;
    optimized.world_from_camera.translation().x() = 0.06;
    ASSERT_TRUE(pipeline.UpdatePoses({optimized}));
    ASSERT_TRUE(pipeline.Flush());
    const auto rebuilt = pipeline.LatestSnapshot();
    ASSERT_NE(rebuilt, nullptr);
    EXPECT_GT(rebuilt->dense_map.generation,
              initial->dense_map.generation);
    ASSERT_EQ(rebuilt->dense_map.points.size(), 1U);
    EXPECT_NEAR(rebuilt->dense_map.points.front().position.x(), 0.06F, 1e-5F);
    pipeline.Stop();
}

TEST(MappingPipelineTest, RejectsDuplicateFrameIdWithoutLeavingGhostVoxels) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(5, 1.0)));
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 1U);

    MappingFrame duplicate = MakeRgbdFrame(5, 2.0);
    duplicate.world_from_camera.translation().x() = 2.0;
    ASSERT_TRUE(pipeline.Submit(std::move(duplicate)));
    ASSERT_TRUE(pipeline.Flush());
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(after->dense_map.generation, before->dense_map.generation);
    ASSERT_EQ(after->dense_map.points.size(), 1U);
    EXPECT_NEAR(after->dense_map.points.front().position.x(), 0.0F, 1e-5F);
    const MappingPipelineStats stats = pipeline.stats();
    EXPECT_EQ(stats.processed_frames, 1U);
    EXPECT_EQ(stats.rejected_frames, 1U);
    EXPECT_EQ(stats.duplicate_frames, 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, ExplicitIdsAdvanceAutomaticIdsWithoutOverflow) {
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.dense_map.maximum_voxels = 16;
    options.maximum_navigation_voxels = 16;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    std::uint64_t assigned = 0;
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(41, 1.0), &assigned));
    EXPECT_EQ(assigned, 41U);
    MappingFrame automatic = MakeRgbdFrame(0, 2.0);
    automatic.world_from_camera.translation().x() = 1.0;
    ASSERT_TRUE(pipeline.Submit(std::move(automatic), &assigned));
    EXPECT_EQ(assigned, 42U);

    constexpr std::uint64_t kPenultimateFrameId =
        std::numeric_limits<std::uint64_t>::max() - 1;
    MappingFrame penultimate = MakeRgbdFrame(kPenultimateFrameId, 3.0);
    penultimate.world_from_camera.translation().x() = 2.0;
    ASSERT_TRUE(pipeline.Submit(std::move(penultimate), &assigned));
    EXPECT_EQ(assigned, kPenultimateFrameId);
    MappingFrame maximum = MakeRgbdFrame(0, 4.0);
    maximum.world_from_camera.translation().x() = 3.0;
    ASSERT_TRUE(pipeline.Submit(std::move(maximum), &assigned));
    EXPECT_EQ(assigned, std::numeric_limits<std::uint64_t>::max());

    assigned = 123;
    MappingFrame exhausted = MakeRgbdFrame(0, 5.0);
    exhausted.world_from_camera.translation().x() = 4.0;
    EXPECT_FALSE(pipeline.Submit(std::move(exhausted), &assigned));
    EXPECT_EQ(assigned, 123U);
    ASSERT_TRUE(pipeline.Flush());

    const MappingPipelineStats stats = pipeline.stats();
    EXPECT_EQ(stats.processed_frames, 4U);
    EXPECT_EQ(stats.duplicate_frames, 0U);
    EXPECT_EQ(stats.rejected_frames, 1U);
    const auto snapshot = pipeline.LatestSnapshot();
    ASSERT_NE(snapshot, nullptr);
    EXPECT_EQ(snapshot->dense_map.points.size(), 4U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, FlushDrainsQueueAndPublishesLatestGeneration) {
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 3600.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    EXPECT_FALSE(pipeline.Flush());
    ASSERT_TRUE(pipeline.Start());

    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    MappingFrame second = MakeRgbdFrame(2, 1.1);
    second.world_from_camera.translation().x() = 1.0;
    ASSERT_TRUE(pipeline.Submit(std::move(second)));
    ASSERT_TRUE(pipeline.Flush());

    const auto snapshot = pipeline.LatestSnapshot();
    ASSERT_NE(snapshot, nullptr);
    EXPECT_EQ(snapshot->dense_map.generation, 2U);
    EXPECT_EQ(snapshot->dense_map.points.size(), 2U);
    EXPECT_EQ(pipeline.stats().processed_frames, 2U);
    pipeline.Stop();
    EXPECT_FALSE(pipeline.Flush());
}

TEST(MappingPipelineTest, FlushAccountsForEveryAcceptedFrameWithoutDrops) {
    MappingPipelineOptions options;
    options.queue_capacity = 8;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    for (std::uint64_t id = 1; id <= 4; ++id) {
        MappingFrame frame = MakeRgbdFrame(id, static_cast<double>(id));
        frame.world_from_camera.translation().x() =
            static_cast<double>(id - 1);
        ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    }
    ASSERT_TRUE(pipeline.Flush());

    const MappingPipelineStats stats = pipeline.stats();
    EXPECT_EQ(stats.submitted_frames,
              stats.processed_frames + stats.gated_frames +
                  stats.rejected_frames + stats.dropped_frames);
    EXPECT_EQ(stats.dropped_frames, 0U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, QueueBackpressureNeverDropsAcceptedFrame) {
    MappingPipelineOptions options;
    options.queue_capacity = 1;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.maximum_extent = 100.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    constexpr std::uint64_t kFrameCount = 60;
    std::deque<MappingFrame> pending;
    for (std::uint64_t id = 1; id <= kFrameCount; ++id) {
        MappingFrame frame = MakeRgbdFrame(0, static_cast<double>(id));
        frame.world_from_camera.translation().x() =
            static_cast<double>(id - 1);
        pending.push_back(std::move(frame));
    }

    std::size_t retries = 0;
    while (!pending.empty()) {
        const MappingPipeline::SubmitResult result =
            pipeline.TrySubmit(&pending.front());
        if (result == MappingPipeline::SubmitResult::kAccepted) {
            pending.pop_front();
            continue;
        }
        // Queue pressure must preserve the caller-owned payload for retry.
        ASSERT_EQ(result, MappingPipeline::SubmitResult::kQueueFull);
        EXPECT_FALSE(pending.front().color.empty());
        EXPECT_EQ(pending.front().frame_id, 0U);
        ++retries;
        ASSERT_LT(retries, 1000000U);
        std::this_thread::yield();
    }
    ASSERT_TRUE(pipeline.Flush());

    const MappingPipelineStats stats = pipeline.stats();
    EXPECT_GT(retries, 0U);
    EXPECT_EQ(stats.backpressured_frames, retries);
    EXPECT_EQ(stats.submitted_frames, kFrameCount);
    EXPECT_EQ(stats.processed_frames, kFrameCount);
    EXPECT_EQ(stats.gated_frames, 0U);
    EXPECT_EQ(stats.rejected_frames, 0U);
    EXPECT_EQ(stats.dropped_frames, 0U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, TrySubmitDistinguishesInvalidAndStoppedFrames) {
    MappingPipelineOptions options;
    MappingPipeline pipeline(options);

    MappingFrame stopped = MakeRgbdFrame(0, 1.0);
    std::uint64_t assigned = 77;
    EXPECT_EQ(pipeline.TrySubmit(&stopped, &assigned),
              MappingPipeline::SubmitResult::kStopped);
    EXPECT_EQ(stopped.frame_id, 0U);
    EXPECT_FALSE(stopped.color.empty());
    EXPECT_EQ(assigned, 77U);

    MappingFrame invalid = MakeRgbdFrame(0, 2.0);
    invalid.depth.release();
    EXPECT_EQ(pipeline.TrySubmit(&invalid, &assigned),
              MappingPipeline::SubmitResult::kInvalid);
    EXPECT_EQ(invalid.frame_id, 0U);
    EXPECT_EQ(assigned, 77U);
    EXPECT_EQ(pipeline.stats().rejected_frames, 1U);
}

TEST(MappingPipelineTest, RemovalAdmissionIsBoundedAndRetryable) {
    MappingPipelineOptions options;
    options.queue_capacity = 1;
    options.maximum_cached_frames = 2;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    EXPECT_EQ(pipeline.RemoveFrames({3U, 2U, 2U, 1U, 1U}),
              std::vector<std::uint64_t>({1U, 2U}));
    ASSERT_TRUE(pipeline.Flush());
    // Reset is a completion barrier and must discard acknowledgements from
    // the old map before ids from the new map can be admitted.
    pipeline.Reset();
    const MappingFrameRemovalResults cleared =
        pipeline.TakeFrameRemovalResults();
    EXPECT_TRUE(cleared.removed_frame_ids.empty());
    EXPECT_TRUE(cleared.retained_frame_ids.empty());

    EXPECT_EQ(pipeline.RemoveFrames({3U}),
              std::vector<std::uint64_t>({3U}));
    ASSERT_TRUE(pipeline.Flush());
    const MappingFrameRemovalResults second =
        pipeline.TakeFrameRemovalResults();
    EXPECT_EQ(second.removed_frame_ids,
              std::vector<std::uint64_t>({3U}));
    pipeline.Stop();
}

TEST(MappingPipelineTest, RemovingFusedFrameRebuildsCloudAndOccupancy) {
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.resolution = 0.1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    MappingFrame second = MakeRgbdFrame(2, 2.0);
    second.world_from_camera.translation().x() = 1.0;
    ASSERT_TRUE(pipeline.Submit(std::move(second)));
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 2U);
    const std::uint64_t generation = before->dense_map.generation;

    EXPECT_EQ(pipeline.RemoveFrames({1U}),
              std::vector<std::uint64_t>({1U}));
    ASSERT_TRUE(pipeline.Flush());
    const MappingFrameRemovalResults removal =
        pipeline.TakeFrameRemovalResults();
    EXPECT_EQ(removal.removed_frame_ids,
              std::vector<std::uint64_t>({1U}));
    EXPECT_TRUE(removal.retained_frame_ids.empty());
    EXPECT_TRUE(pipeline.TakeFrameRemovalResults()
                    .removed_frame_ids.empty());
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    ASSERT_EQ(after->dense_map.points.size(), 1U);
    EXPECT_GT(after->dense_map.generation, generation);
    EXPECT_EQ(pipeline.stats().removed_frames, 1U);
    EXPECT_EQ(pipeline.stats().removal_rebuild_failures, 0U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, RemovingArchivedFrameDeletesItAndRebuilds) {
    ScopedTestDirectory directory("remove_archived");
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 1;
    options.maximum_history_cache_bytes = 16U * 1024U * 1024U;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    for (std::uint64_t id = 1; id <= 3; ++id) {
        MappingFrame frame = MakeRgbdFrame(id, static_cast<double>(id));
        frame.world_from_camera.translation().x() =
            static_cast<double>(id - 1);
        ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    }
    ASSERT_TRUE(pipeline.Flush());
    EXPECT_EQ(pipeline.RemoveFrames({1U}),
              std::vector<std::uint64_t>({1U}));
    ASSERT_TRUE(pipeline.Flush());
    const MappingFrameRemovalResults removal =
        pipeline.TakeFrameRemovalResults();
    EXPECT_EQ(removal.removed_frame_ids,
              std::vector<std::uint64_t>({1U}));
    EXPECT_TRUE(removal.retained_frame_ids.empty());
    const auto snapshot = pipeline.LatestSnapshot();
    ASSERT_NE(snapshot, nullptr);
    EXPECT_EQ(snapshot->dense_map.points.size(), 2U);
    EXPECT_EQ(pipeline.stats().removed_frames, 1U);
    EXPECT_EQ(pipeline.stats().removal_rebuild_failures, 0U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, IncompleteHistoryRefusesDestructiveRemoval) {
    ScopedTestDirectory directory("remove_incomplete");
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 1;
    options.maximum_history_cache_bytes = 1;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    MappingFrame second = MakeRgbdFrame(2, 2.0);
    second.world_from_camera.translation().x() = 1.0;
    ASSERT_TRUE(pipeline.Submit(std::move(second)));
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 2U);

    EXPECT_EQ(pipeline.RemoveFrames({2U}),
              std::vector<std::uint64_t>({2U}));
    ASSERT_TRUE(pipeline.Flush());
    const MappingFrameRemovalResults removal =
        pipeline.TakeFrameRemovalResults();
    EXPECT_TRUE(removal.removed_frame_ids.empty());
    EXPECT_EQ(removal.retained_frame_ids,
              std::vector<std::uint64_t>({2U}));
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(after->dense_map.generation, before->dense_map.generation);
    EXPECT_EQ(after->dense_map.points.size(), 2U);
    EXPECT_EQ(pipeline.stats().removed_frames, 0U);
    EXPECT_EQ(pipeline.stats().removal_rebuild_failures, 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest,
     QueuedDuplicateCannotDeleteCachedFrameWhenRebuildIsUnsafe) {
    ScopedTestDirectory directory("remove_queued_duplicate");
    MappingPipelineOptions options;
    options.queue_capacity = 1;
    options.maximum_cached_frames = 1;
    options.maximum_history_cache_bytes = 1;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1U, 1.0)));
    ASSERT_TRUE(pipeline.Flush());
    MappingFrame second = MakeRgbdFrame(2U, 2.0);
    second.world_from_camera.translation().x() = 1.0;
    ASSERT_TRUE(pipeline.Submit(std::move(second)));
    ASSERT_TRUE(pipeline.Flush());
    ASSERT_GE(pipeline.stats().history_archive_failures, 1U);
    ASSERT_EQ(pipeline.stats().processed_frames, 2U);

    MappingFrame in_flight = MakeRgbdFrame(3U, 3.0);
    constexpr int kImageSize = 768;
    in_flight.color = cv::Mat(kImageSize, kImageSize, CV_8UC3,
                              cv::Scalar(3, 2, 1))
                          .clone();
    in_flight.depth = cv::Mat(kImageSize, kImageSize, CV_16UC1,
                              cv::Scalar(1000))
                          .clone();
    in_flight.world_from_camera.translation().x() = 2.0;
    ASSERT_TRUE(pipeline.Submit(std::move(in_flight)));

    MappingFrame duplicate = MakeRgbdFrame(1U, 4.0);
    MappingPipeline::SubmitResult queued =
        MappingPipeline::SubmitResult::kQueueFull;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(5);
    while (queued == MappingPipeline::SubmitResult::kQueueFull &&
           std::chrono::steady_clock::now() < deadline) {
        queued = pipeline.TrySubmit(&duplicate);
        if (queued == MappingPipeline::SubmitResult::kQueueFull)
            std::this_thread::yield();
    }
    ASSERT_EQ(queued, MappingPipeline::SubmitResult::kAccepted);
    ASSERT_EQ(pipeline.stats().processed_frames, 2U);

    EXPECT_EQ(pipeline.RemoveFrames({1U}),
              std::vector<std::uint64_t>({1U}));
    ASSERT_TRUE(pipeline.Flush());
    const MappingFrameRemovalResults removal =
        pipeline.TakeFrameRemovalResults();
    EXPECT_TRUE(removal.removed_frame_ids.empty());
    EXPECT_EQ(removal.retained_frame_ids,
              std::vector<std::uint64_t>({1U}));
    EXPECT_EQ(pipeline.stats().removed_frames, 1U);
    EXPECT_EQ(pipeline.stats().removal_rebuild_failures, 1U);
    EXPECT_EQ(pipeline.stats().duplicate_frames, 0U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, NavigationMapHasIndependentVoxelLimit) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.dense_map.maximum_voxels = 16;
    options.maximum_navigation_voxels = 1;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    MappingFrame frame = MakeRgbdFrame(1, 1.0);
    frame.color =
        cv::Mat(1, 3, CV_8UC3, cv::Scalar(30, 20, 10)).clone();
    frame.depth = cv::Mat(1, 3, CV_16UC1, cv::Scalar(1000)).clone();
    ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    ASSERT_TRUE(pipeline.Flush());

    const auto snapshot = pipeline.LatestSnapshot();
    ASSERT_NE(snapshot, nullptr);
    EXPECT_EQ(snapshot->dense_map.points.size(), 3U);
    EXPECT_EQ(snapshot->navigation_map.points.size(), 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, RollingNavigationMapKeepsCurrentWindowAcrossRebuild) {
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.dense_map.maximum_voxels = 16;
    options.maximum_navigation_voxels = 16;
    options.navigation_radius = 2.0;
    options.navigation_recenter_distance = 1.0;
    options.occupancy_grid.maximum_extent = 20.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    MappingFrame current = MakeRgbdFrame(2, 2.0);
    current.world_from_camera.translation().x() = 10.0;
    ASSERT_TRUE(pipeline.Submit(std::move(current)));
    ASSERT_TRUE(pipeline.Flush());

    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    EXPECT_EQ(before->dense_map.points.size(), 2U);
    ASSERT_EQ(before->navigation_map.points.size(), 1U);
    EXPECT_NEAR(before->navigation_map.points.front().position.x(), 10.0F,
                1e-5F);

    ASSERT_TRUE(pipeline.RequestRebuild());
    ASSERT_TRUE(pipeline.Flush());
    const auto rebuilt = pipeline.LatestSnapshot();
    ASSERT_NE(rebuilt, nullptr);
    EXPECT_GT(rebuilt->dense_map.generation,
              before->dense_map.generation);
    EXPECT_EQ(rebuilt->dense_map.points.size(), 2U);
    ASSERT_EQ(rebuilt->navigation_map.points.size(), 1U);
    EXPECT_NEAR(rebuilt->navigation_map.points.front().position.x(), 10.0F,
                1e-5F);
    pipeline.Stop();
}

TEST(MappingPipelineTest, RejectsInvalidRollingNavigationWindow) {
    MappingPipelineOptions options;
    options.navigation_radius = 2.0;
    options.navigation_recenter_distance = 3.0;
    EXPECT_THROW(MappingPipeline(std::move(options)), std::invalid_argument);

    options = MappingPipelineOptions{};
    options.navigation_radius = -1.0;
    EXPECT_THROW(MappingPipeline(std::move(options)), std::invalid_argument);
}

TEST(MappingPipelineTest, IncompleteHistoryNeverDestroysOlderMapData) {
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 2;
    options.maximum_history_cache_bytes = 1;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.dense_map.maximum_voxels = 16;
    options.maximum_navigation_voxels = 16;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    for (std::uint64_t id = 1; id <= 3; ++id) {
        MappingFrame frame = MakeRgbdFrame(id, static_cast<double>(id));
        frame.world_from_camera.translation().x() =
            static_cast<double>(id - 1);
        ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    }
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 3U);

    TimedPose optimized;
    optimized.frame_id = 3;
    optimized.timestamp = 4.0;
    optimized.world_from_camera.translation().x() = 4.0;
    ASSERT_TRUE(pipeline.UpdatePoses({optimized}));
    ASSERT_TRUE(pipeline.Flush());
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(after->dense_map.points.size(), 3U);
    EXPECT_EQ(after->dense_map.generation, before->dense_map.generation);
    EXPECT_EQ(pipeline.stats().skipped_incomplete_rebuilds, 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, DiskBackedHistoryRebuildsEveryEvictedFrame) {
    ScopedTestDirectory directory("complete_history");
    MappingPipelineOptions options;
    options.queue_capacity = 8;
    options.maximum_cached_frames = 2;
    options.maximum_history_cache_bytes = 16U * 1024U * 1024U;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.dense_map.maximum_voxels = 32;
    options.maximum_navigation_voxels = 32;
    options.occupancy_grid.maximum_extent = 20.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    for (std::uint64_t id = 1; id <= 5; ++id) {
        MappingFrame frame = MakeRgbdFrame(id, static_cast<double>(id));
        frame.world_from_camera.translation().x() =
            static_cast<double>(id - 1);
        ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    }
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 5U);
    EXPECT_EQ(directory.EntryCount(), 1U);

    std::vector<TimedPose> optimized;
    for (std::uint64_t id = 1; id <= 5; ++id) {
        TimedPose pose;
        pose.frame_id = id;
        pose.timestamp = 10.0 + static_cast<double>(id);
        pose.world_from_camera.translation().x() =
            9.0 + static_cast<double>(id);
        optimized.push_back(std::move(pose));
    }
    ASSERT_TRUE(pipeline.UpdatePoses(std::move(optimized)));
    ASSERT_TRUE(pipeline.Flush());
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    ASSERT_EQ(after->dense_map.points.size(), 5U);
    std::vector<float> x_coordinates;
    for (const ColorPoint& point : after->dense_map.points)
        x_coordinates.push_back(point.position.x());
    std::sort(x_coordinates.begin(), x_coordinates.end());
    for (std::size_t index = 0; index < x_coordinates.size(); ++index) {
        EXPECT_NEAR(x_coordinates[index], 10.0F + index, 1e-5F);
    }
    EXPECT_EQ(pipeline.stats().history_archive_failures, 0U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, ArchivedMonocularFrameRetriangulatesLosslessly) {
    ScopedTestDirectory directory("monocular_history");
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 1;
    options.maximum_history_cache_bytes = 16U * 1024U * 1024U;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 2;
    options.projection.minimum_depth = 0.3;
    options.projection.maximum_depth = 3.0;
    options.projection.stereo_number_of_disparities = 16;
    options.projection.stereo_block_size = 3;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.02;
    options.dense_map.maximum_voxels = 100000;
    options.maximum_navigation_voxels = 100000;
    options.occupancy_grid.maximum_extent = 20.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeTemporalFrame(0.1, 8)));
    ASSERT_TRUE(pipeline.Flush());

    MappingFrame filler = MakeRgbdFrame(3, 2.0);
    filler.world_from_camera.translation().x() = 10.0;
    ASSERT_TRUE(pipeline.Submit(std::move(filler)));
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);

    TimedPose reference;
    reference.frame_id = 1;
    reference.timestamp = 3.0;
    reference.world_from_camera.translation().x() = 0.2;
    TimedPose current;
    current.frame_id = 2;
    current.timestamp = 3.0;
    ASSERT_TRUE(pipeline.UpdatePoses({reference, current}));
    ASSERT_TRUE(pipeline.Flush());
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    EXPECT_GT(after->dense_map.generation, before->dense_map.generation);
    EXPECT_EQ(pipeline.stats().history_archive_failures, 0U);

    std::vector<float> depths;
    for (const ColorPoint& point : after->dense_map.points) {
        if (point.position.x() < 5.0F)
            depths.push_back(point.position.z());
    }
    ASSERT_GT(depths.size(), 100U);
    const auto median = depths.begin() + depths.size() / 2;
    std::nth_element(depths.begin(), median, depths.end());
    EXPECT_NEAR(*median, 2.0F, 0.25F);
    pipeline.Stop();
}

TEST(MappingPipelineTest, ArchiveQuotaFailureKeepsExistingMap) {
    ScopedTestDirectory directory("archive_failure");
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 2;
    options.maximum_history_cache_bytes = 1;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.dense_map.maximum_voxels = 16;
    options.maximum_navigation_voxels = 16;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    for (std::uint64_t id = 1; id <= 3; ++id) {
        MappingFrame frame = MakeRgbdFrame(id, static_cast<double>(id));
        frame.world_from_camera.translation().x() =
            static_cast<double>(id - 1);
        ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    }
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 3U);

    TimedPose optimized;
    optimized.frame_id = 3;
    optimized.timestamp = 4.0;
    optimized.world_from_camera.translation().x() = 4.0;
    ASSERT_TRUE(pipeline.UpdatePoses({optimized}));
    ASSERT_TRUE(pipeline.Flush());
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(after->dense_map.points.size(), 3U);
    EXPECT_EQ(after->dense_map.generation, before->dense_map.generation);
    EXPECT_GE(pipeline.stats().history_archive_failures, 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, CorruptArchiveHeaderNeverReplacesExistingMap) {
    ScopedTestDirectory directory("corrupt_archive");
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 1;
    options.maximum_history_cache_bytes = 16U * 1024U * 1024U;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    MappingFrame second = MakeRgbdFrame(2, 2.0);
    second.world_from_camera.translation().x() = 1.0;
    ASSERT_TRUE(pipeline.Submit(std::move(second)));
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 2U);

    fs::path archive;
    for (const auto& child : fs::directory_iterator(directory.path())) {
        if (!child.is_directory())
            continue;
        for (const auto& file : fs::directory_iterator(child.path())) {
            if (file.path().extension() == ".bin") {
                archive = file.path();
                break;
            }
        }
    }
    ASSERT_FALSE(archive.empty());
    std::fstream damaged(archive,
                          std::ios::binary | std::ios::in | std::ios::out);
    ASSERT_TRUE(damaged.good());
    constexpr std::streamoff kColorBytesOffset =
        8 + sizeof(std::uint32_t) + sizeof(std::uint64_t);
    damaged.seekp(kColorBytesOffset);
    const std::uint64_t impossible_size =
        std::numeric_limits<std::uint64_t>::max();
    damaged.write(reinterpret_cast<const char*>(&impossible_size),
                  sizeof(impossible_size));
    damaged.close();
    ASSERT_FALSE(damaged.fail());

    ASSERT_TRUE(pipeline.RequestRebuild());
    ASSERT_TRUE(pipeline.Flush());
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(after->dense_map.points.size(), 2U);
    EXPECT_EQ(after->dense_map.generation, before->dense_map.generation);
    EXPECT_GE(pipeline.stats().history_archive_failures, 1U);
    EXPECT_GE(pipeline.stats().skipped_incomplete_rebuilds, 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest,
     RemovalPreflightRetainsMetadataWhenSurvivorArchiveIsCorrupt) {
    ScopedTestDirectory directory("remove_corrupt_survivor");
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 1;
    options.maximum_history_cache_bytes = 16U * 1024U * 1024U;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    for (std::uint64_t id = 1; id <= 3; ++id) {
        MappingFrame frame = MakeRgbdFrame(id, static_cast<double>(id));
        frame.world_from_camera.translation().x() =
            static_cast<double>(id - 1);
        ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    }
    ASSERT_TRUE(pipeline.Flush());
    const auto before = pipeline.LatestSnapshot();
    ASSERT_NE(before, nullptr);
    ASSERT_EQ(before->dense_map.points.size(), 3U);

    fs::path archive;
    for (const auto& child : fs::directory_iterator(directory.path())) {
        if (!child.is_directory())
            continue;
        for (const auto& file : fs::directory_iterator(child.path())) {
            if (file.path().extension() == ".bin") {
                archive = file.path();
                break;
            }
        }
        if (!archive.empty())
            break;
    }
    ASSERT_FALSE(archive.empty());
    std::fstream damaged(archive,
                         std::ios::binary | std::ios::in | std::ios::out);
    ASSERT_TRUE(damaged.good());
    damaged.seekp(0);
    const std::uint64_t invalid_magic = 0;
    damaged.write(reinterpret_cast<const char*>(&invalid_magic),
                  sizeof(invalid_magic));
    damaged.close();
    ASSERT_FALSE(damaged.fail());

    EXPECT_EQ(pipeline.RemoveFrames({3U}),
              std::vector<std::uint64_t>({3U}));
    ASSERT_TRUE(pipeline.Flush());
    const MappingFrameRemovalResults removal =
        pipeline.TakeFrameRemovalResults();
    EXPECT_TRUE(removal.removed_frame_ids.empty());
    EXPECT_EQ(removal.retained_frame_ids,
              std::vector<std::uint64_t>({3U}));
    const auto after = pipeline.LatestSnapshot();
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(after->dense_map.generation, before->dense_map.generation);
    EXPECT_EQ(after->dense_map.points.size(), 3U);
    EXPECT_EQ(pipeline.stats().removed_frames, 0U);
    EXPECT_EQ(pipeline.stats().removal_rebuild_failures, 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, ResetRemovesOnlyItsUniqueHistoryDirectory) {
    ScopedTestDirectory directory("reset_cleanup");
    const fs::path unrelated = directory.path() / "keep.txt";
    {
        std::ofstream output(unrelated);
        output << "keep";
    }
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.maximum_cached_frames = 1;
    options.maximum_history_cache_bytes = 16U * 1024U * 1024U;
    options.history_cache_directory = directory.path().string();
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(2, 2.0)));
    ASSERT_TRUE(pipeline.Flush());
    EXPECT_EQ(directory.EntryCount(), 2U);

    pipeline.Reset();
    ASSERT_TRUE(pipeline.Flush());
    EXPECT_TRUE(fs::exists(unrelated));
    EXPECT_EQ(directory.EntryCount(), 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, DestructorRemovesOnlyItsUniqueHistoryDirectory) {
    ScopedTestDirectory directory("destructor_cleanup");
    const fs::path unrelated = directory.path() / "keep.txt";
    {
        std::ofstream output(unrelated);
        output << "keep";
    }
    {
        MappingPipelineOptions options;
        options.queue_capacity = 4;
        options.maximum_cached_frames = 1;
        options.maximum_history_cache_bytes = 16U * 1024U * 1024U;
        options.history_cache_directory = directory.path().string();
        options.projection.pixel_stride = 1;
        options.projection.minimum_depth = 0.1;
        options.projection.maximum_depth = 2.0;
        options.minimum_translation = 0.0;
        options.minimum_rotation_radians = 0.0;
        options.maximum_frame_interval = 0.0;
        options.snapshot_period = 0.0;
        options.occupancy_grid.maximum_extent = 10.0;
        MappingPipeline pipeline(options);
        ASSERT_TRUE(pipeline.Start());
        ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
        ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(2, 2.0)));
        ASSERT_TRUE(pipeline.Flush());
        EXPECT_EQ(directory.EntryCount(), 2U);
        pipeline.Stop();
    }
    EXPECT_TRUE(fs::exists(unrelated));
    EXPECT_EQ(directory.EntryCount(), 1U);
}

TEST(MappingPipelineTest, PreVoxelFiltersCachedCameraCloud) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.1;
    options.occupancy_grid.resolution = 0.5;
    options.occupancy_grid.maximum_extent = 10.0;
    options.occupancy_grid.maximum_cells = 1000;
    options.occupancy_grid.maximum_snapshot_cells = 1000;
    MappingPipeline pipeline(options);
    MappingFrame frame = MakeRgbdFrame(9, 1.0);
    frame.camera.fx = 1000.0;
    frame.camera.fy = 1000.0;
    frame.color = cv::Mat(2, 2, CV_8UC3, cv::Scalar(3, 2, 1)).clone();
    frame.depth = cv::Mat(2, 2, CV_16UC1, cv::Scalar(1000)).clone();
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(std::move(frame)));
    auto snapshot = WaitForGeneration(pipeline, 1);
    ASSERT_NE(snapshot, nullptr);
    EXPECT_EQ(snapshot->dense_map.points.size(), 1U);
    ASSERT_TRUE(pipeline.RequestRebuild());
    auto rebuilt = WaitForGeneration(
        pipeline, snapshot->dense_map.generation + 1U);
    ASSERT_NE(rebuilt, nullptr);
    EXPECT_EQ(rebuilt->dense_map.points.size(), 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, PublishesLastGenerationAfterIdlePeriod) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.05;
    options.dense_map.voxel_size = 0.05;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    auto first = WaitForGeneration(pipeline, 1);
    ASSERT_NE(first, nullptr);

    MappingFrame second = MakeRgbdFrame(2, 1.01);
    second.world_from_camera.translation().x() = 0.2;
    ASSERT_TRUE(pipeline.Submit(std::move(second)));
    auto final = WaitForGeneration(pipeline, 2);
    ASSERT_NE(final, nullptr);
    EXPECT_EQ(final->dense_map.generation, 2U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, QueuedFrameUsesNewerPendingOptimizedPose) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.05;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    TimedPose optimized;
    optimized.frame_id = 7;
    optimized.timestamp = 1.0;
    optimized.world_from_camera.translation().x() = 2.0;
    ASSERT_TRUE(pipeline.UpdatePoses({optimized}));
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(7, 1.0)));
    auto snapshot = WaitForGeneration(pipeline, 1);
    ASSERT_NE(snapshot, nullptr);
    ASSERT_EQ(snapshot->dense_map.points.size(), 1U);
    EXPECT_NEAR(snapshot->dense_map.points[0].position.x(), 2.0F, 1e-5F);
    pipeline.Stop();
}

TEST(MappingPipelineTest, StopDrainsQueuedFrameAndPublishesFinalSnapshot) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.snapshot_period = 10.0;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(1, 1.0)));
    pipeline.Stop();
    auto snapshot = pipeline.LatestSnapshot();
    ASSERT_NE(snapshot, nullptr);
    EXPECT_EQ(snapshot->dense_map.generation, 1U);
    EXPECT_FALSE(snapshot->dense_map.points.empty());
}

TEST(MappingPipelineTest, RetriangulatesMonocularCloudAfterScaleChange) {
    MappingPipelineOptions options;
    options.projection.pixel_stride = 2;
    options.projection.minimum_depth = 0.3;
    options.projection.maximum_depth = 3.0;
    options.projection.stereo_number_of_disparities = 16;
    options.projection.stereo_block_size = 3;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.02;
    options.occupancy_grid.maximum_extent = 10.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeTemporalFrame(0.1, 8)));
    auto first = WaitForGeneration(pipeline, 1);
    ASSERT_NE(first, nullptr);

    TimedPose reference;
    reference.frame_id = 1;
    reference.timestamp = 2.0;
    reference.world_from_camera.translation().x() = 0.2;
    TimedPose current;
    current.frame_id = 2;
    current.timestamp = 2.0;
    ASSERT_TRUE(pipeline.UpdatePoses({reference, current}));
    auto scaled = WaitForGeneration(
        pipeline, first->dense_map.generation + 1U);
    ASSERT_NE(scaled, nullptr);
    ASSERT_FALSE(scaled->dense_map.points.empty());
    std::vector<float> depths;
    for (const ColorPoint& point : scaled->dense_map.points)
        depths.push_back(point.position.z());
    const auto median = depths.begin() + depths.size() / 2;
    std::nth_element(depths.begin(), median, depths.end());
    EXPECT_NEAR(*median, 2.0F, 0.25F);
    pipeline.Stop();
}

TEST(MappingPipelineTest, RetriangulatesOnlyAffectedMonocularFrames) {
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.projection.pixel_stride = 2;
    options.projection.minimum_depth = 0.3;
    options.projection.maximum_depth = 3.0;
    options.projection.stereo_number_of_disparities = 16;
    options.projection.stereo_block_size = 3;
    options.minimum_translation = 0.0;
    options.minimum_rotation_radians = 0.0;
    options.maximum_frame_interval = 0.0;
    options.snapshot_period = 0.0;
    options.dense_map.voxel_size = 0.02;
    options.dense_map.maximum_voxels = 200000;
    options.maximum_navigation_voxels = 200000;
    options.occupancy_grid.maximum_extent = 20.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());

    ASSERT_TRUE(pipeline.Submit(MakeTemporalFrame(0.1, 8)));
    MappingFrame unaffected = MakeTemporalFrame(0.1, 8);
    unaffected.frame_id = 4;
    unaffected.reference_frame_id = 3;
    unaffected.timestamp = 2.0;
    unaffected.world_from_camera.translation().x() = 10.0;
    ASSERT_TRUE(pipeline.Submit(std::move(unaffected)));
    ASSERT_TRUE(pipeline.Flush());

    TimedPose reference;
    reference.frame_id = 1;
    reference.timestamp = 3.0;
    reference.world_from_camera.translation().x() = 0.2;
    ASSERT_TRUE(pipeline.UpdatePoses({reference}));
    ASSERT_TRUE(pipeline.Flush());
    EXPECT_EQ(pipeline.stats().retriangulated_frames, 1U);
    pipeline.Stop();
}

TEST(MappingPipelineTest, MotionGateSkipsRedundantKeyframe) {
    MappingPipelineOptions options;
    options.queue_capacity = 4;
    options.projection.pixel_stride = 1;
    options.projection.minimum_depth = 0.1;
    options.projection.maximum_depth = 2.0;
    options.minimum_translation = 0.5;
    options.minimum_rotation_radians = 0.5;
    options.maximum_frame_interval = 10.0;
    options.snapshot_period = 0.0;
    MappingPipeline pipeline(options);
    ASSERT_TRUE(pipeline.Start());
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(11, 1.0)));
    ASSERT_TRUE(pipeline.Submit(MakeRgbdFrame(12, 1.1)));
    ASSERT_TRUE(WaitForHandledFrames(pipeline, 2));
    const MappingPipelineStats stats = pipeline.stats();
    EXPECT_EQ(stats.processed_frames, 1U);
    EXPECT_EQ(stats.gated_frames, 1U);
    pipeline.Stop();
}

}  // namespace
}  // namespace autonomy::localization::atlas::map
