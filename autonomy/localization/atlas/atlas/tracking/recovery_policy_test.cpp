/*
 * Copyright 2026 The Openbot Authors
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

#include "atlas/tracking/recovery_policy.hpp"
#include "atlas/tracking/local_map_tracking_transaction.hpp"

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::internal {
namespace {

TEST(TrackingRecoveryPolicyTest,
     TrajectoryFallbackUsesCurrentTimestampAndIsLost) {
    constexpr double kCurrentTimestamp = 42.125;
    constexpr TrajectoryRecordDecision decision =
        MakeTrajectoryRecordDecision(kCurrentTimestamp, false, false);

    EXPECT_DOUBLE_EQ(decision.timestamp, kCurrentTimestamp);
    EXPECT_TRUE(decision.reuse_previous_pose);
    EXPECT_TRUE(decision.lost);
}

TEST(TrackingRecoveryPolicyTest, ValidRecentlyLostPoseRemainsExportable) {
    constexpr double kCurrentTimestamp = 42.25;
    constexpr TrajectoryRecordDecision decision =
        MakeTrajectoryRecordDecision(kCurrentTimestamp, true, false);

    EXPECT_DOUBLE_EQ(decision.timestamp, kCurrentTimestamp);
    EXPECT_FALSE(decision.reuse_previous_pose);
    EXPECT_FALSE(decision.lost);
}

TEST(TrackingRecoveryPolicyTest, ExplicitLossOverridesCurrentPose) {
    constexpr double kCurrentTimestamp = 42.375;
    constexpr TrajectoryRecordDecision decision =
        MakeTrajectoryRecordDecision(kCurrentTimestamp, true, true);

    EXPECT_DOUBLE_EQ(decision.timestamp, kCurrentTimestamp);
    EXPECT_FALSE(decision.reuse_previous_pose);
    EXPECT_TRUE(decision.lost);
}

TEST(TrackingRecoveryPolicyTest, UsesPredictionResultOnlyForImuRgbd) {
    EXPECT_TRUE(ShouldUseImuPredictionResult(true));
    EXPECT_FALSE(ShouldUseImuPredictionResult(false));
}

TEST(TrackingRecoveryPolicyTest, MotionModelPoseSeedUsesMinimumOfFifteen) {
    EXPECT_FALSE(HasEnoughMotionModelPoseSeedMatches(14));
    EXPECT_TRUE(HasEnoughMotionModelPoseSeedMatches(15));
    EXPECT_TRUE(HasEnoughMotionModelPoseSeedMatches(19));
    EXPECT_TRUE(HasEnoughMotionModelPoseSeedMatches(20));
}

TEST(TrackingRecoveryPolicyTest, UsesVisualRecoveryUntilImuBa2Completes) {
    EXPECT_TRUE(CanStartImuRgbdVisualRecovery(true, true, false, false));
    EXPECT_TRUE(CanStartImuRgbdVisualRecovery(true, true, true, false));
    EXPECT_FALSE(CanStartImuRgbdVisualRecovery(true, true, true, true));
    EXPECT_TRUE(
        ShouldUseImuRgbdVisualRecovery(true, true, true, true, true, true));
    EXPECT_FALSE(
        ShouldUseImuRgbdVisualRecovery(true, true, true, false, true, true));
    EXPECT_FALSE(
        ShouldUseImuRgbdVisualRecovery(false, true, true, true, true, false));
    EXPECT_FALSE(
        ShouldUseImuRgbdVisualRecovery(true, false, true, true, true, false));
}

TEST(TrackingRecoveryPolicyTest, RecoveryForcesVisualPoseOptimization) {
    EXPECT_TRUE(ShouldUseVisualPoseOptimization(false, false));
    EXPECT_TRUE(ShouldUseVisualPoseOptimization(false, true));
    EXPECT_FALSE(ShouldUseVisualPoseOptimization(true, false));
    EXPECT_TRUE(ShouldUseVisualPoseOptimization(true, true));
}

TEST(TrackingRecoveryPolicyTest, ForcesVisualRecoveryOnlyBeforeImuInit) {
    EXPECT_FALSE(ShouldForceVisualRecoveryPoseOptimization(false, false));
    EXPECT_FALSE(ShouldForceVisualRecoveryPoseOptimization(false, true));
    EXPECT_TRUE(ShouldForceVisualRecoveryPoseOptimization(true, false));
    EXPECT_FALSE(ShouldForceVisualRecoveryPoseOptimization(true, true));
}

TEST(TrackingRecoveryPolicyTest, RecoveryBypassesRecentRelocationThreshold) {
    EXPECT_TRUE(FailsRecentRelocalizationInlierThreshold(true, 49, false));
    EXPECT_FALSE(FailsRecentRelocalizationInlierThreshold(true, 49, true));
    EXPECT_FALSE(FailsRecentRelocalizationInlierThreshold(true, 50, false));
    EXPECT_FALSE(FailsRecentRelocalizationInlierThreshold(false, 0, false));
}

TEST(TrackingRecoveryPolicyTest, RecoveryVotesWithCurrentFrameMatches) {
    EXPECT_TRUE(ShouldUseCurrentFrameForLocalKeyFrameVoting(false, false, false,
                                                            false, false));
    EXPECT_TRUE(ShouldUseCurrentFrameForLocalKeyFrameVoting(true, true, false,
                                                            false, false));
    EXPECT_TRUE(ShouldUseCurrentFrameForLocalKeyFrameVoting(true, false, true,
                                                            true, true));
    EXPECT_FALSE(ShouldUseCurrentFrameForLocalKeyFrameVoting(true, false, true,
                                                             true, false));
    EXPECT_FALSE(ShouldUseCurrentFrameForLocalKeyFrameVoting(false, false, true,
                                                             true, false));
    EXPECT_FALSE(ShouldUseCurrentFrameForLocalKeyFrameVoting(true, true, true,
                                                             true, false));
    EXPECT_FALSE(ShouldUseCurrentFrameForLocalKeyFrameVoting(true, false, false,
                                                             true, true));
    EXPECT_FALSE(ShouldUseCurrentFrameForLocalKeyFrameVoting(true, false, true,
                                                             false, true));
}

TEST(TrackingRecoveryPolicyTest, MergesRecoveryVotesOnlyTransactionally) {
    EXPECT_TRUE(ShouldMergeRgbdRecoveryLocalKeyFrameVotes(true, true, true,
                                                          true));
    EXPECT_FALSE(ShouldMergeRgbdRecoveryLocalKeyFrameVotes(false, true, true,
                                                           true));
    EXPECT_FALSE(ShouldMergeRgbdRecoveryLocalKeyFrameVotes(true, false, true,
                                                           true));
    EXPECT_FALSE(ShouldMergeRgbdRecoveryLocalKeyFrameVotes(true, true, false,
                                                           true));
    EXPECT_FALSE(ShouldMergeRgbdRecoveryLocalKeyFrameVotes(true, true, true,
                                                           false));
}

TEST(TrackingRecoveryPolicyTest,
     UsesStrictTemporalLocalMapProjectionOnlyDuringRecoveryTransaction) {
    EXPECT_EQ(kRgbdTemporalLocalMapProjectionRadius, 3);
    EXPECT_TRUE(ShouldUseStrictRgbdTemporalLocalMapProjection(true, true,
                                                              true));
    EXPECT_FALSE(ShouldUseStrictRgbdTemporalLocalMapProjection(false, true,
                                                               true));
    EXPECT_FALSE(ShouldUseStrictRgbdTemporalLocalMapProjection(true, false,
                                                               true));
    EXPECT_FALSE(ShouldUseStrictRgbdTemporalLocalMapProjection(true, true,
                                                               false));
}

TEST(TrackingRecoveryPolicyTest, AppliesStrictTemporalMapGeometryLimits) {
    EXPECT_DOUBLE_EQ(kRgbdTemporalMapReprojectionLimitPixels, 3.0);
    EXPECT_DOUBLE_EQ(RgbdTemporalMapXyzToleranceMeters(1.0, 2.0), 0.10);
    EXPECT_DOUBLE_EQ(RgbdTemporalMapXyzToleranceMeters(5.0, 4.0), 0.15);
    EXPECT_DOUBLE_EQ(RgbdTemporalMapXyzToleranceMeters(20.0, 18.0), 0.25);

    EXPECT_TRUE(IsRgbdTemporalMapReprojectionConsistent(3.0));
    EXPECT_FALSE(IsRgbdTemporalMapReprojectionConsistent(3.000001));
    EXPECT_FALSE(IsRgbdTemporalMapReprojectionConsistent(-0.001));
    EXPECT_FALSE(IsRgbdTemporalMapReprojectionConsistent(
        std::numeric_limits<double>::infinity()));

    EXPECT_TRUE(IsRgbdTemporalMapXyzConsistent(5.0, 4.0, 0.15));
    EXPECT_FALSE(IsRgbdTemporalMapXyzConsistent(5.0, 4.0, 0.150001));
    EXPECT_FALSE(IsRgbdTemporalMapXyzConsistent(0.0, 4.0, 0.0));
    EXPECT_FALSE(IsRgbdTemporalMapXyzConsistent(
        5.0, std::numeric_limits<double>::quiet_NaN(), 0.0));
    EXPECT_FALSE(IsRgbdTemporalMapXyzConsistent(
        5.0, 4.0, std::numeric_limits<double>::infinity()));
}

TEST(TrackingRecoveryPolicyTest, RequiresExactTemporalSeedIdentity) {
    constexpr RgbdTemporalSeedIdentity identity = {
        7, 8, 9, 9, 10, 10, 11, 11,
    };
    constexpr RgbdTemporalSeedContext context = {
        true, 7, 8, 9, 10, 11,
    };
    EXPECT_TRUE(HasMatchingRgbdTemporalSeedIdentity(identity, context));

    RgbdTemporalSeedIdentity mismatch = identity;
    mismatch.reference_frame_id = 8;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));
    mismatch = identity;
    mismatch.current_frame_id = 9;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));
    mismatch = identity;
    mismatch.reference_map_id = 12;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));
    mismatch = identity;
    mismatch.current_map_id = 12;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));
    mismatch = identity;
    mismatch.reference_coordinate_epoch = 13;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));
    mismatch = identity;
    mismatch.current_coordinate_epoch = 13;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));
    mismatch = identity;
    mismatch.reference_map_revision = 12;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));
    mismatch = identity;
    mismatch.current_map_revision = 12;
    EXPECT_FALSE(HasMatchingRgbdTemporalSeedIdentity(mismatch, context));

    RgbdTemporalSeedContext unset_last_frame = context;
    unset_last_frame.last_frame_set = false;
    EXPECT_FALSE(
        HasMatchingRgbdTemporalSeedIdentity(identity, unset_last_frame));
    RgbdTemporalSeedContext mismatched_context = context;
    mismatched_context.last_frame_id = 14;
    EXPECT_FALSE(
        HasMatchingRgbdTemporalSeedIdentity(identity, mismatched_context));
}

TEST(TrackingRecoveryPolicyTest, SeedsLocalMapWithoutCurrentFrameVotes) {
    EXPECT_TRUE(ShouldSeedImuRgbdRecoveryLocalKeyFrames(true, true, false));
    EXPECT_FALSE(ShouldSeedImuRgbdRecoveryLocalKeyFrames(true, true, true));
    EXPECT_FALSE(ShouldSeedImuRgbdRecoveryLocalKeyFrames(true, false, false));
    EXPECT_FALSE(ShouldSeedImuRgbdRecoveryLocalKeyFrames(false, true, false));
}

TEST(TrackingRecoveryPolicyTest, RecoveryUsesImuOnlyAsFailedReferenceSeed) {
    EXPECT_TRUE(
        ShouldUseImuPoseSeedAfterReferenceFailure(true, true, false, true));
    EXPECT_FALSE(
        ShouldUseImuPoseSeedAfterReferenceFailure(true, true, true, true));
    EXPECT_FALSE(
        ShouldUseImuPoseSeedAfterReferenceFailure(true, true, false, false));
    EXPECT_FALSE(
        ShouldUseImuPoseSeedAfterReferenceFailure(false, true, false, true));
    EXPECT_FALSE(
        ShouldUseImuPoseSeedAfterReferenceFailure(true, false, false, true));
}

TEST(TrackingRecoveryPolicyTest, BoundsAndDeduplicatesReferenceCandidates) {
    EXPECT_EQ(kImuRgbdRecoveryCovisibilityCandidateLimit, 5);
    EXPECT_EQ(kImuRgbdRecoveryReferenceCandidateLimit, 7);
    EXPECT_TRUE(ShouldAppendImuRgbdRecoveryReferenceCandidate(true, false, 6));
    EXPECT_FALSE(
        ShouldAppendImuRgbdRecoveryReferenceCandidate(false, false, 0));
    EXPECT_FALSE(ShouldAppendImuRgbdRecoveryReferenceCandidate(true, true, 0));
    EXPECT_FALSE(ShouldAppendImuRgbdRecoveryReferenceCandidate(true, false, 7));
    EXPECT_FALSE(
        ShouldAppendImuRgbdRecoveryReferenceCandidate(true, false, -1));
}

TEST(TrackingRecoveryPolicyTest, UsesBoundedRgbdVisualRecoveryTimeout) {
    EXPECT_DOUBLE_EQ(RecentlyLostTimeoutSeconds(true, true, 5.0), 1.0);
    EXPECT_DOUBLE_EQ(RecentlyLostTimeoutSeconds(true, false, 5.0), 5.0);
    EXPECT_DOUBLE_EQ(RecentlyLostTimeoutSeconds(false, true, 5.0), 5.0);

    EXPECT_FALSE(HasRecentlyLostTimedOut(1.0, true, true, 5.0));
    EXPECT_TRUE(HasRecentlyLostTimedOut(1.000001, true, true, 5.0));
    EXPECT_FALSE(HasRecentlyLostTimedOut(5.0, true, false, 5.0));
    EXPECT_TRUE(HasRecentlyLostTimedOut(5.000001, true, false, 5.0));
}

TEST(TrackingRecoveryPolicyTest,
     TimeoutDeclaresLossOnlyAfterVisualValidationFails) {
    EXPECT_FALSE(ShouldDeclareImuRgbdVisualRecoveryLost(false, false));
    EXPECT_FALSE(ShouldDeclareImuRgbdVisualRecoveryLost(true, true));
    EXPECT_TRUE(ShouldDeclareImuRgbdVisualRecoveryLost(true, false));
}

TEST(TrackingRecoveryPolicyTest, HoldsLastImuRgbdPoseAndKeepsTheMap) {
    EXPECT_TRUE(CanHoldLastImuRgbdPose(true, true));
    EXPECT_FALSE(CanHoldLastImuRgbdPose(true, false));
    EXPECT_FALSE(CanHoldLastImuRgbdPose(false, true));
    EXPECT_TRUE(ShouldKeepActiveMapAfterLost(true, true, true));
    EXPECT_FALSE(ShouldKeepActiveMapAfterLost(true, false, true));
    EXPECT_FALSE(ShouldKeepActiveMapAfterLost(true, true, false));
    EXPECT_FALSE(ShouldKeepActiveMapAfterLost(false, true, true));
}

TEST(TrackingRecoveryPolicyTest, AppliesSensorSpecificInlierThresholds) {
    EXPECT_FALSE(HasEnoughRecentlyLostInliers(10, false));
    EXPECT_TRUE(HasEnoughRecentlyLostInliers(11, false));
    EXPECT_FALSE(HasEnoughRecentlyLostInliers(14, true));
    EXPECT_TRUE(HasEnoughRecentlyLostInliers(15, true));
}

TEST(TrackingRecoveryPolicyTest, ClassifiesInertialTimestampJumps) {
    EXPECT_EQ(ClassifyInertialTimestampJump(1.0, 1.0),
              InertialTimestampJumpAction::kNone);
    EXPECT_EQ(ClassifyInertialTimestampJump(1.5, 1.0),
              InertialTimestampJumpAction::kNone);
    EXPECT_EQ(ClassifyInertialTimestampJump(2.000001, 1.0),
              InertialTimestampJumpAction::kSoftRecover);
    EXPECT_EQ(ClassifyInertialTimestampJump(10.0, 1.0),
              InertialTimestampJumpAction::kSoftRecover);
    EXPECT_EQ(ClassifyInertialTimestampJump(31.000001, 1.0),
              InertialTimestampJumpAction::kHardReset);
    EXPECT_EQ(ClassifyInertialTimestampJump(0.5, 1.0),
              InertialTimestampJumpAction::kNone);
}

TEST(TrackingRecoveryPolicyTest,
     EvaluatesLostKeyFrameOnlyWithFreshImuPoseSeed) {
    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, true, true, true, true, true, true, false));
    EXPECT_TRUE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, true, true, true, true, true, true, true));

    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        false, true, true, true, true, true, true, true));
    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, false, true, true, true, true, true, true));
    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, true, false, true, true, true, true, true));
    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, true, true, false, true, true, true, true));
    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, true, true, true, false, true, true, true));
    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, true, true, true, true, false, true, true));
    EXPECT_FALSE(ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
        true, true, true, true, true, true, false, true));
}

TEST(TrackingRecoveryPolicyTest, BoundsTemporalOdometryState) {
    BoundedRgbdTemporalOdometryState state;
    state.Start(10.0, 9.9, 3, 4, 5);

    EXPECT_TRUE(state.CanEstimate(10.1, 3, 4, 5));
    EXPECT_FALSE(state.CanEstimate(10.1, 3, 4, 8));
    EXPECT_FALSE(state.CanEstimate(10.151, 3, 4, 5));

    ASSERT_TRUE(state.CanAccept(true));
    state.Advance(10.0, true);
    EXPECT_FALSE(state.CanAccept(true));
    EXPECT_TRUE(state.CanAccept(false));

    state.Advance(10.1, false);
    ASSERT_TRUE(state.CanAccept(true));
    state.Advance(10.2, true);
    state.Advance(10.3, false);
    EXPECT_FALSE(state.CanAccept(true));
}

TEST(TrackingRecoveryPolicyTest,
     RebaseTemporalOdometryRevisionPreservesRecoveryBudget) {
    BoundedRgbdTemporalOdometryState state;
    state.Start(10.0, 9.9, 3, 4, 5);
    state.Advance(10.0, true);

    EXPECT_FALSE(state.RebaseMapRevision(9, 4, 5, 6));
    EXPECT_FALSE(state.RebaseMapRevision(3, 9, 5, 6));
    EXPECT_FALSE(state.RebaseMapRevision(3, 4, 4, 6));
    EXPECT_FALSE(state.RebaseMapRevision(3, 4, 5, 5));
    ASSERT_TRUE(state.RebaseMapRevision(3, 4, 5, 6));

    EXPECT_TRUE(state.active);
    EXPECT_DOUBLE_EQ(state.start_timestamp, 10.0);
    EXPECT_DOUBLE_EQ(state.reference_timestamp, 10.0);
    EXPECT_EQ(state.map_id, 3U);
    EXPECT_EQ(state.coordinate_epoch, 4U);
    EXPECT_EQ(state.map_revision, 6);
    EXPECT_EQ(state.consecutive_low_coverage_steps, 1);
    EXPECT_EQ(state.total_low_coverage_steps, 1);
    EXPECT_FALSE(state.CanAccept(true));
    EXPECT_TRUE(state.CanEstimate(10.1, 3, 4, 6));
}

TEST(TrackingRecoveryPolicyTest, UsesPhaseSpecificReanchorThreshold) {
    EXPECT_FALSE(HasEnoughRgbdTemporalReanchorInliers(false, 49));
    EXPECT_TRUE(HasEnoughRgbdTemporalReanchorInliers(false, 50));
    EXPECT_FALSE(HasEnoughRgbdTemporalReanchorInliers(true, 14));
    EXPECT_TRUE(HasEnoughRgbdTemporalReanchorInliers(true, 15));
}

TEST(TrackingRecoveryPolicyTest, RestoresOrCommitsExactMapTrackingScratch) {
    MapPoint point;
    KeyFrame keyframe;
    std::tie(point.mTrackProjX, point.mTrackProjY, point.mTrackDepth,
             point.mTrackDepthR, point.mTrackProjXR, point.mTrackProjYR,
             point.mbTrackInView, point.mbTrackInViewR, point.mnTrackScaleLevel,
             point.mnTrackScaleLevelR, point.mTrackViewCos,
             point.mTrackViewCosR, point.mnTrackReferenceForFrame,
             point.mnLastFrameSeen) =
        MapPointTrackingValues(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, true, false,
                               7, 8, 9.0F, 10.0F, 11, 12);
    keyframe.mnTrackReferenceForFrame = 13;
    const MapPointTrackingValues expected = MapPointTrackingSnapshot(&point);
    {
        LocalMapTrackingTransaction transaction;
        transaction.Reserve(1, 1);
        transaction.Capture(&point);
        transaction.Capture(&keyframe);
        point.mTrackProjX = point.mTrackDepth = point.mTrackProjXR = 20.0F;
        point.mTrackProjY = point.mTrackDepthR = point.mTrackProjYR = 21.0F;
        point.mbTrackInView = false;
        point.mbTrackInViewR = true;
        point.mnTrackScaleLevel = point.mnTrackScaleLevelR = 22;
        point.mTrackViewCos = point.mTrackViewCosR = 23.0F;
        point.mnTrackReferenceForFrame = point.mnLastFrameSeen = 24;
        keyframe.mnTrackReferenceForFrame = 26;
        transaction.Capture(&point);
        transaction.Capture(&keyframe);
    }
    EXPECT_EQ(MapPointTrackingSnapshot(&point), expected);
    EXPECT_EQ(keyframe.mnTrackReferenceForFrame, 13U);
    {
        LocalMapTrackingTransaction transaction;
        transaction.Capture(&point);
        point.mTrackProjX = 42.0F;
        transaction.Commit();
    }
    EXPECT_EQ(point.mTrackProjX, 42.0F);
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
