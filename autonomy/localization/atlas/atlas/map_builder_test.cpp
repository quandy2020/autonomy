#include "atlas/map_builder.hpp"

#include <cmath>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(MapBuilderConfigTest, UsesBoundedKeyframeRefreshBatchByDefault) {
    const SystemConfig config;
    EXPECT_EQ(config.maximum_mapping_keyframes_per_refresh, 4U);
}

TEST(MapBuilderCandidateBufferTest,
     ProtectsOnlyUnsubmittedKeyframesFromStalePruning) {
    std::deque<map::MappingFrame> pending_frames;
    for (const double timestamp : {1.0, 2.0, 3.0, 4.0, 8.0}) {
        map::MappingFrame frame;
        frame.timestamp = timestamp;
        pending_frames.push_back(std::move(frame));
    }

    std::vector<TrajectoryPose> trajectory(2);
    trajectory[0].time = 2.0;
    trajectory[1].time = 3.0;
    std::unordered_map<std::int64_t, std::uint64_t> submitted_frame_ids;
    submitted_frame_ids.emplace(3000000000LL, 30U);
    std::unordered_set<std::int64_t> protected_keys =
        internal::UnsubmittedMappingKeyframeKeys(
            trajectory, submitted_frame_ids);

    EXPECT_EQ(internal::DiscardStaleMappingCandidates(
                  5.0, protected_keys, &pending_frames),
              3U);
    ASSERT_EQ(pending_frames.size(), 2U);
    EXPECT_DOUBLE_EQ(pending_frames[0].timestamp, 2.0);
    EXPECT_DOUBLE_EQ(pending_frames[1].timestamp, 8.0);

    submitted_frame_ids.emplace(2000000000LL, 20U);
    protected_keys = internal::UnsubmittedMappingKeyframeKeys(
        trajectory, submitted_frame_ids);
    EXPECT_EQ(internal::DiscardStaleMappingCandidates(
                  5.0, protected_keys, &pending_frames),
              1U);
    ASSERT_EQ(pending_frames.size(), 1U);
    EXPECT_DOUBLE_EQ(pending_frames.front().timestamp, 8.0);
}

TEST(MapBuilderCandidateBufferTest,
     CapacityEvictsOrdinaryCandidatesBeforeProtectedKeyframes) {
    std::deque<map::MappingFrame> pending_frames;
    for (const double timestamp : {1.0, 2.0, 3.0, 4.0}) {
        map::MappingFrame frame;
        frame.timestamp = timestamp;
        pending_frames.push_back(std::move(frame));
    }
    std::unordered_set<std::int64_t> protected_keys{
        1000000000LL, 3000000000LL};

    internal::MappingCandidateEvictionResult evicted =
        internal::EnforceMappingCandidateCapacity(
            2U, protected_keys, &pending_frames);
    EXPECT_EQ(evicted.unprotected_candidates, 2U);
    EXPECT_FALSE(evicted.saturated);
    ASSERT_EQ(pending_frames.size(), 2U);
    EXPECT_DOUBLE_EQ(pending_frames[0].timestamp, 1.0);
    EXPECT_DOUBLE_EQ(pending_frames[1].timestamp, 3.0);
}

TEST(MapBuilderCandidateBufferTest,
     ProtectsNewPendingKeyframeBeforeCapacityEnforcement) {
    std::deque<map::MappingFrame> pending_frames;
    for (const double timestamp : {1.0, 2.0, 3.0}) {
        map::MappingFrame frame;
        frame.timestamp = timestamp;
        pending_frames.push_back(std::move(frame));
    }
    std::unordered_set<std::int64_t> protected_keys{
        1000000000LL, 3000000000LL};

    map::MappingFrame frame;
    frame.timestamp = 5.0;
    pending_frames.push_back(std::move(frame));
    EXPECT_EQ(protected_keys.count(5000000000LL), 0U);
    internal::ProtectPendingMappingKeyframes(
        {5.0}, {}, &protected_keys);
    EXPECT_EQ(protected_keys.count(5000000000LL), 1U);

    const internal::MappingCandidateEvictionResult evicted =
        internal::EnforceMappingCandidateCapacity(
            3U, protected_keys, &pending_frames);
    EXPECT_EQ(evicted.unprotected_candidates, 1U);
    EXPECT_FALSE(evicted.saturated);
    ASSERT_EQ(pending_frames.size(), 3U);
    EXPECT_DOUBLE_EQ(pending_frames[0].timestamp, 1.0);
    EXPECT_DOUBLE_EQ(pending_frames[1].timestamp, 3.0);
    EXPECT_DOUBLE_EQ(pending_frames[2].timestamp, 5.0);
}

TEST(MapBuilderCandidateBufferTest,
     AllProtectedCandidatesReportSaturationWithoutEviction) {
    std::deque<map::MappingFrame> pending_frames;
    std::unordered_set<std::int64_t> protected_keys;
    for (std::int64_t index = 0; index < 46; ++index) {
        map::MappingFrame frame;
        frame.timestamp = static_cast<double>(index);
        pending_frames.push_back(std::move(frame));
        protected_keys.insert(index * 1000000000LL);
    }

    const internal::MappingCandidateEvictionResult evicted =
        internal::EnforceMappingCandidateCapacity(
            45U, protected_keys, &pending_frames);

    EXPECT_EQ(evicted.unprotected_candidates, 0U);
    EXPECT_TRUE(evicted.saturated);
    ASSERT_EQ(pending_frames.size(), 46U);
    for (std::size_t index = 0; index < pending_frames.size(); ++index) {
        EXPECT_DOUBLE_EQ(pending_frames[index].timestamp,
                         static_cast<double>(index));
    }
}

TEST(MapBuilderCandidateBufferTest,
     RecomputesIncomingProtectionAfterCommitBeforeCapacityDecision) {
    std::deque<map::MappingFrame> pending_frames;
    std::unordered_set<std::int64_t> protected_keys;
    for (std::int64_t index = 0; index < 2; ++index) {
        map::MappingFrame frame;
        frame.timestamp = static_cast<double>(index);
        pending_frames.push_back(std::move(frame));
        protected_keys.insert(index * 1000000000LL);
    }

    const internal::MappingCandidateSlotResult before_refresh =
        internal::PrepareMappingCandidateSlot(
            2U, 2.0, protected_keys, &pending_frames);
    EXPECT_FALSE(before_refresh.have_slot);
    EXPECT_FALSE(before_refresh.incoming_is_keyframe);
    EXPECT_TRUE(before_refresh.eviction.saturated);
    ASSERT_EQ(pending_frames.size(), 2U);

    // Simulate LocalMapper committing the incoming keyframe between the
    // initial pending snapshot and the forced capacity refresh.
    protected_keys.insert(2000000000LL);
    const internal::MappingCandidateSlotResult after_refresh =
        internal::PrepareMappingCandidateSlot(
            2U, 2.0, protected_keys, &pending_frames);
    EXPECT_FALSE(after_refresh.have_slot);
    EXPECT_TRUE(after_refresh.incoming_is_keyframe);
    EXPECT_TRUE(after_refresh.eviction.saturated);
    ASSERT_EQ(pending_frames.size(), 2U);
    EXPECT_DOUBLE_EQ(pending_frames[0].timestamp, 0.0);
    EXPECT_DOUBLE_EQ(pending_frames[1].timestamp, 1.0);
}

TEST(MapBuilderCandidateBufferTest,
     SoftCapacityNeverEvictsProtectedCandidatesOrBlocksAdmission) {
    std::deque<map::MappingFrame> pending_frames;
    std::unordered_set<std::int64_t> protected_keys;
    for (std::int64_t index = 0; index < 2; ++index) {
        map::MappingFrame frame;
        frame.timestamp = static_cast<double>(index);
        pending_frames.push_back(std::move(frame));
        protected_keys.insert(index * 1000000000LL);
    }
    map::MappingFrame incoming;
    incoming.timestamp = 2.0;

    const internal::MappingCandidateBufferResult result =
        internal::BufferMappingCandidateWithSoftCapacity(
            2U, protected_keys, std::move(incoming), &pending_frames);

    EXPECT_EQ(result.eviction.unprotected_candidates, 0U);
    EXPECT_TRUE(result.eviction.saturated);
    EXPECT_TRUE(result.soft_capacity_exceeded);
    ASSERT_EQ(pending_frames.size(), 3U);
    EXPECT_DOUBLE_EQ(pending_frames[0].timestamp, 0.0);
    EXPECT_DOUBLE_EQ(pending_frames[1].timestamp, 1.0);
    EXPECT_DOUBLE_EQ(pending_frames[2].timestamp, 2.0);
}

TEST(MapBuilderCandidateBufferTest,
     SnapshotReclassifiesSpeculativeOverflowAsOrdinary) {
    std::deque<map::MappingFrame> pending_frames;
    for (const double timestamp : {0.0, 1.0, 2.0}) {
        map::MappingFrame frame;
        frame.timestamp = timestamp;
        pending_frames.push_back(std::move(frame));
    }
    const std::unordered_set<std::int64_t> protected_keys{
        0LL, 1000000000LL};

    const internal::MappingCandidateEvictionResult result =
        internal::EnforceMappingCandidateCapacity(
            2U, protected_keys, &pending_frames);

    EXPECT_EQ(result.unprotected_candidates, 1U);
    EXPECT_FALSE(result.saturated);
    ASSERT_EQ(pending_frames.size(), 2U);
    EXPECT_DOUBLE_EQ(pending_frames[0].timestamp, 0.0);
    EXPECT_DOUBLE_EQ(pending_frames[1].timestamp, 1.0);
}

TEST(MapBuilderCandidateBufferTest,
     SnapshotRetainsSpeculativeOverflowWhenItBecomesAKeyframe) {
    std::deque<map::MappingFrame> pending_frames;
    std::unordered_set<std::int64_t> protected_keys;
    for (std::int64_t index = 0; index < 3; ++index) {
        map::MappingFrame frame;
        frame.timestamp = static_cast<double>(index);
        pending_frames.push_back(std::move(frame));
        protected_keys.insert(index * 1000000000LL);
    }

    const internal::MappingCandidateEvictionResult result =
        internal::EnforceMappingCandidateCapacity(
            2U, protected_keys, &pending_frames);

    EXPECT_EQ(result.unprotected_candidates, 0U);
    EXPECT_TRUE(result.saturated);
    ASSERT_EQ(pending_frames.size(), 3U);
}

TEST(MapBuilderCullSyncTest,
     EmptyCommittedTrajectoryStillCullsAfterTwoMissingSnapshots) {
    const std::unordered_map<std::int64_t, std::uint64_t> submitted{
        {1000000000LL, 7U}, {2000000000LL, 9U}};
    std::unordered_map<std::int64_t, std::size_t> missing_counts;

    EXPECT_TRUE(internal::SelectCulledMappingFrames(
                    {}, {1.0}, submitted, 2U, &missing_counts)
                    .empty());
    EXPECT_EQ(missing_counts.count(1000000000LL), 0U);
    ASSERT_EQ(missing_counts.at(2000000000LL), 1U);

    const auto culled = internal::SelectCulledMappingFrames(
        {}, {1.0}, submitted, 2U, &missing_counts);
    ASSERT_EQ(culled.size(), 1U);
    EXPECT_EQ(culled.front().timestamp_key, 2000000000LL);
    EXPECT_EQ(culled.front().frame_id, 9U);
}

TEST(MapBuilderCullSyncTest, ReappearanceClearsMissingCount) {
    const std::unordered_map<std::int64_t, std::uint64_t> submitted{
        {3000000000LL, 11U}};
    std::unordered_map<std::int64_t, std::size_t> missing_counts;
    EXPECT_TRUE(internal::SelectCulledMappingFrames(
                    {}, {}, submitted, 2U, &missing_counts)
                    .empty());
    ASSERT_EQ(missing_counts.at(3000000000LL), 1U);

    TrajectoryPose active;
    active.time = 3.0;
    EXPECT_TRUE(internal::SelectCulledMappingFrames(
                    {active}, {}, submitted, 2U, &missing_counts)
                    .empty());
    EXPECT_TRUE(missing_counts.empty());
}

TrajectoryPose PoseAt(double time, double x, double yaw_radians = 0.0) {
    TrajectoryPose pose;
    pose.time = time;
    pose.pose_world_camera.translation = Eigen::Vector3d(x, 0.0, 0.0);
    pose.pose_world_camera.rotation = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw_radians, Eigen::Vector3d::UnitZ()));
    return pose;
}

TEST(MapBuilderPoseSyncTest, UsesCumulativeChangeFromLastAcceptedUpdate) {
    const std::unordered_map<std::int64_t, std::uint64_t> submitted{
        {1000000000LL, 7U}};
    internal::MappingPoseCache last_sent;
    last_sent.emplace(7U, Eigen::Isometry3d::Identity());

    EXPECT_TRUE(internal::SelectChangedMappingPoses(
                    {PoseAt(1.0, 0.01)}, submitted, last_sent,
                    0.02, 0.1, false)
                    .empty());
    const auto cumulative = internal::SelectChangedMappingPoses(
        {PoseAt(1.0, 0.021)}, submitted, last_sent, 0.02, 0.1, false);
    ASSERT_EQ(cumulative.size(), 1U);
    EXPECT_EQ(cumulative.front().frame_id, 7U);
    internal::RecordSentMappingPoses(cumulative, &last_sent);

    EXPECT_TRUE(internal::SelectChangedMappingPoses(
                    {PoseAt(1.0, 0.03)}, submitted, last_sent,
                    0.02, 0.1, false)
                    .empty());
}

TEST(MapBuilderPoseSyncTest, ForceResendsAndQuaternionSignIsEquivalent) {
    const std::unordered_map<std::int64_t, std::uint64_t> submitted{
        {2000000000LL, 9U}};
    internal::MappingPoseCache last_sent;
    last_sent.emplace(9U, Eigen::Isometry3d::Identity());
    TrajectoryPose equivalent = PoseAt(2.0, 0.0);
    equivalent.pose_world_camera.rotation.coeffs() *= -1.0;

    EXPECT_TRUE(internal::SelectChangedMappingPoses(
                    {equivalent}, submitted, last_sent,
                    0.02, 0.01, false)
                    .empty());
    EXPECT_EQ(internal::SelectChangedMappingPoses(
                  {equivalent}, submitted, last_sent,
                  0.02, 0.01, true)
                  .size(),
              1U);
}

TEST(MapBuilderPoseSyncTest, DoesNotAdvanceStateUntilExplicitlyRecorded) {
    const std::unordered_map<std::int64_t, std::uint64_t> submitted{
        {3000000000LL, 11U}};
    internal::MappingPoseCache last_sent;
    const auto update = internal::SelectChangedMappingPoses(
        {PoseAt(3.0, 1.0)}, submitted, last_sent, 0.02, 0.01, false);
    ASSERT_EQ(update.size(), 1U);
    EXPECT_TRUE(last_sent.empty());
    internal::RecordSentMappingPoses(update, &last_sent);
    EXPECT_EQ(last_sent.size(), 1U);
}

TEST(MapBuilderPoseSyncTest, DetectsMapOrDestructiveEpochTransitions) {
    bool valid = false;
    std::uint64_t map_id = 0;
    std::uint64_t destructive_epoch = 0;
    EXPECT_FALSE(internal::ObserveMappingMap(
        true, 4U, 7U, &valid, &map_id, &destructive_epoch));
    EXPECT_TRUE(valid);
    EXPECT_EQ(map_id, 4U);
    EXPECT_EQ(destructive_epoch, 7U);
    EXPECT_FALSE(internal::ObserveMappingMap(
        false, 5U, 8U, &valid, &map_id, &destructive_epoch));
    EXPECT_EQ(map_id, 4U);
    EXPECT_EQ(destructive_epoch, 7U);
    EXPECT_FALSE(internal::ObserveMappingMap(
        true, 4U, 7U, &valid, &map_id, &destructive_epoch));
    EXPECT_TRUE(internal::ObserveMappingMap(
        true, 4U, 8U, &valid, &map_id, &destructive_epoch));
    EXPECT_EQ(map_id, 4U);
    EXPECT_EQ(destructive_epoch, 8U);
    EXPECT_TRUE(internal::ObserveMappingMap(
        true, 5U, 8U, &valid, &map_id, &destructive_epoch));
    EXPECT_EQ(map_id, 5U);
}

TEST(MapBuilderPoseSyncTest, RejectsOnlyAmbiguousScaledEpochBoundary) {
    internal::MapIdentitySnapshot before{true, 4U, 7U, 2U};
    internal::MapIdentitySnapshot after = before;
    EXPECT_FALSE(internal::CrossedScaledCoordinateEpoch(before, after));

    after.coordinate_epoch = 8U;
    EXPECT_TRUE(internal::CrossedScaledCoordinateEpoch(before, after));

    after.destructive_epoch = 3U;
    EXPECT_FALSE(internal::CrossedScaledCoordinateEpoch(before, after));

    after = before;
    after.map_id = 5U;
    after.coordinate_epoch = 8U;
    EXPECT_FALSE(internal::CrossedScaledCoordinateEpoch(before, after));

    after = before;
    after.valid = false;
    after.coordinate_epoch = 8U;
    EXPECT_FALSE(internal::CrossedScaledCoordinateEpoch(before, after));
}

TEST(MapBuilderTest, RejectsMissingConfiguration) {
    SystemConfig config;
    config.settings_path = "/path/that/does/not/exist.yaml";
    config.vocabulary_path = "/path/that/does/not/exist.fbow";
    MapBuilder builder(std::move(config));
    EXPECT_FALSE(builder.Start());
    EXPECT_FALSE(builder.started());
}

}  // namespace
}  // namespace autonomy::localization::atlas
