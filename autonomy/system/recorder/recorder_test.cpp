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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"

#include "autonomy/system/recorder/bt_attribution.hpp"
#include "autonomy/system/recorder/clock.hpp"
#include "autonomy/system/recorder/episode.hpp"
#include "autonomy/system/recorder/event_graph.hpp"
#include "autonomy/system/recorder/hash_chain.hpp"
#include "autonomy/system/recorder/mcap_writer.hpp"
#include "autonomy/system/recorder/metrics.hpp"
#include "autonomy/system/recorder/observation.hpp"
#include "autonomy/system/recorder/pipeline.hpp"
#include "autonomy/system/recorder/player.hpp"
#include "autonomy/system/recorder/replay.hpp"
#include "autonomy/system/recorder/rolling_buffer.hpp"
#include "autonomy/system/recorder/root_cause.hpp"
#include "autonomy/system/recorder/convert.hpp"
#include "autonomy/system/recorder/sidecar.hpp"
#include "autonomy/system/recorder/safety_monitor.hpp"
#include "autonomy/system/recorder/signals.hpp"
#include "autonomy/system/recorder/trigger.hpp"
#include "autonomy/system/recorder/upload.hpp"
#include "autonomy/system/recorder/value_tier.hpp"
#include "autonomy/system/recorder/version_tuple.hpp"

#include "automsgs/msgs/geometry_msgs/pose2d.pb.h"
#include "automsgs/msgs/geometry_msgs/twist.pb.h"
#include "automsgs/msgs/geometry_msgs/twist_stamped.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "automsgs/msgs/nav_msgs/path.pb.h"
#include "automsgs/msgs/vehicle_msgs/robot_task_status.pb.h"
#include "automsgs/task/localization.pb.h"
#include "automsgs/task/voice.pb.h"

namespace autonomy {
namespace system {
namespace recorder {
namespace {

proto::DualClock ClockAt(int64_t mono, int64_t wall = 0) {
    proto::DualClock c;
    c.set_monotonic_ns(mono);
    c.set_wall_utc_ns(wall == 0 ? mono : wall);
    c.set_time_source(proto::TIME_SOURCE_SOFTWARE);
    return c;
}

Sample MakeSample(const std::string& channel, int64_t t, std::string payload) {
    Sample s;
    s.channel = channel;
    s.message_type = "test.Payload";
    s.payload = std::move(payload);
    s.clock = ClockAt(t);
    return s;
}

BtTransition Bt(const std::string& name, const std::string& from,
                const std::string& to, int64_t t) {
    return BtTransition{name, from, to, ClockAt(t)};
}

TEST(RecorderClock, DualStamps) {
    const auto a = NowDualClock();
    const auto b = NowDualClock();
    EXPECT_GT(a.monotonic_ns(), 0);
    EXPECT_GT(a.wall_utc_ns(), 0);
    EXPECT_GE(b.monotonic_ns(), a.monotonic_ns());
    EXPECT_EQ(a.time_source(), proto::TIME_SOURCE_SOFTWARE);
}

TEST(VersionTuple, MissingGoesToDiagnostic) {
    unsetenv("AUTONOMY_POLICY_VERSION");
    unsetenv("AUTONOMY_SOFTWARE_VERSION");
    unsetenv("AUTONOMY_ENV_VERSION");
    unsetenv("AUTONOMY_SEED");
    unsetenv("AUTONOMY_GIT_SHA");
    proto::RecorderOptions opts;
    const auto check = MakeVersionTuple(opts);
    EXPECT_FALSE(check.admitted);
    EXPECT_FALSE(check.missing.empty());
    EXPECT_FALSE(IsAdmitted(check.tuple));
}

TEST(VersionTuple, SoftwareVersionFallback) {
    unsetenv("AUTONOMY_POLICY_VERSION");
    setenv("AUTONOMY_SOFTWARE_VERSION", "sw-9", 1);
    proto::RecorderOptions opts;
    const auto check = MakeVersionTuple(opts);
    EXPECT_EQ(check.tuple.policy_version(), "sw-9");
    unsetenv("AUTONOMY_SOFTWARE_VERSION");
}

TEST(VersionTuple, CompleteIsAdmitted) {
    proto::RecorderOptions opts;
    opts.set_policy_version("policy-1");
    opts.set_env_version("env-1");
    opts.set_seed("42");
    opts.set_git_sha("abc1234");
    const auto check = MakeVersionTuple(opts);
    EXPECT_TRUE(check.admitted);
    EXPECT_TRUE(check.missing.empty());
    EXPECT_EQ(check.tuple.git_sha(), "abc1234");
}

TEST(RollingBuffer, PrunesByWindow) {
    RollingBuffer buf(1'000'000'000);
    buf.Push(MakeSample("/odom", 1'000'000'000, "a"));
    buf.Push(MakeSample("/odom", 1'500'000'000, "b"));
    buf.Push(MakeSample("/odom", 3'000'000'000, "c"));
    const auto snap = buf.Snapshot();
    ASSERT_EQ(snap.size(), 2U);
    EXPECT_EQ(snap.front().payload, "b");
    EXPECT_EQ(snap.back().payload, "c");
}

TEST(RollingBuffer, TakeAllMovesPayloads) {
    RollingBuffer buf(10'000'000'000);
    buf.Push(MakeSample("/odom", 100, "a"));
    buf.Push(MakeSample("/scan", 200, std::string(32, 'x')));
    auto batch = buf.TakeAll();
    ASSERT_EQ(batch.size(), 2U);
    EXPECT_EQ(batch[1].payload.size(), 32U);
    EXPECT_EQ(buf.size(), 0U);
}

TEST(Signals, SampleFlagsForRoles) {
    proto::RecorderOptions opts;
    opts.set_odom_channel("/odom");
    opts.set_bt_log_channel("/behavior_tree/log");
    EXPECT_NE(SampleFlagsFor("/odom", opts) & kSampleOdom, 0);
    EXPECT_NE(SampleFlagsFor("/autonomy/task/localization/goal", opts) &
                  kSampleGoal,
              0);
    EXPECT_NE(SampleFlagsFor("/autonomy/task/localization/feedback", opts) &
                  kSampleFeedback,
              0);
    EXPECT_EQ(SampleFlagsFor("/scan", opts), 0);
}

TEST(RollingBuffer, DropThroughRemovesFlushed) {
    RollingBuffer buf(10'000'000'000);
    buf.Push(MakeSample("/odom", 100, "a"));
    buf.Push(MakeSample("/odom", 200, "b"));
    buf.Push(MakeSample("/odom", 300, "c"));
    buf.DropThrough(200);
    const auto snap = buf.Snapshot();
    ASSERT_EQ(snap.size(), 1U);
    EXPECT_EQ(snap.front().payload, "c");
}

TEST(TriggerEngine, RecoverAndEstopAreP0) {
    proto::RecorderOptions opts;
    opts.set_path_deviate_m(0.4);
    auto* ch = opts.add_channels();
    ch->set_name("/scan");
    ch->set_timeout_sec(0.2);
    TriggerEngine eng(opts);

    auto recover = eng.OnBtStatus("FollowPath/Recover", "IDLE", "RUNNING",
                                  ClockAt(10));
    ASSERT_TRUE(recover);
    EXPECT_EQ(recover->kind(), proto::TRIGGER_BT_RECOVER);

    auto estop = eng.OnSafetyLatch(true, "button", ClockAt(11));
    ASSERT_TRUE(estop);
    EXPECT_EQ(estop->kind(), proto::TRIGGER_ESTOP);
    EXPECT_FALSE(eng.OnSafetyLatch(true, "button", ClockAt(12)));

    auto path = eng.OnPathDeviation(0.5, ClockAt(13));
    ASSERT_TRUE(path);
    EXPECT_EQ(path->kind(), proto::TRIGGER_PATH_DEVIATE);

    eng.OnSample(MakeSample("/scan", 1'000'000'000, "x"));
    auto timeouts = eng.PollTimeouts(ClockAt(1'500'000'000));
    ASSERT_EQ(timeouts.size(), 1U);
    EXPECT_EQ(timeouts[0].kind(), proto::TRIGGER_CHANNEL_TIMEOUT);

    EXPECT_EQ(eng.Classify({*recover, *estop}, false), proto::PRIORITY_P0);

    auto take = eng.OnTakeover(ClockAt(14), "/autonomy/task/teleop/goal");
    ASSERT_TRUE(take);
    EXPECT_EQ(take->kind(), proto::TRIGGER_TAKEOVER);
    EXPECT_EQ(eng.Classify({*take}, true), proto::PRIORITY_P1);
}

TEST(TriggerEngine, CostSpike) {
    proto::RecorderOptions opts;
    opts.set_cost_spike_sigma(2.0);
    TriggerEngine eng(opts);
    std::optional<proto::TriggerEvent> hit;
    for (int i = 0; i < 12; ++i) {
        hit = eng.OnCost(1.0, ClockAt(i));
    }
    EXPECT_FALSE(hit);
    hit = eng.OnCost(50.0, ClockAt(20));
    ASSERT_TRUE(hit);
    EXPECT_EQ(hit->kind(), proto::TRIGGER_COST_SPIKE);
}

TEST(EpisodeTracker, OpensOnNavigateAndClosesOnRecover) {
    EpisodeTracker ep;
    EXPECT_FALSE(ep.OnBtStatus(Bt("NavigateToPose", "IDLE", "RUNNING", 1)));
    EXPECT_TRUE(ep.state().open);
    auto closed = ep.OnBtStatus(Bt("FollowPath/Recover", "IDLE", "RUNNING", 2));
    ASSERT_TRUE(closed);
    EXPECT_FALSE(closed->open);
    EXPECT_EQ(closed->transitions.size(), 2U);
}

TEST(EpisodeTracker, GoalOpensWithoutBt) {
    EpisodeTracker ep;
    ep.OnGoal(ClockAt(5));
    EXPECT_TRUE(ep.state().open);
    EXPECT_EQ(ep.state().bt_root, "goal");
}

TEST(EpisodeTracker, LocalizationRootAndSessionMode) {
    EpisodeTracker ep;
    EXPECT_FALSE(ep.OnBtStatus(Bt("Localization", "IDLE", "RUNNING", 1)));
    EXPECT_TRUE(ep.state().open);
    auto closed = ep.OnBtStatus(Bt("Localization", "RUNNING", "FAILURE", 2));
    ASSERT_TRUE(closed);
    EXPECT_FALSE(closed->open);

    EpisodeTracker session;
    session.SetSessionMode(true);
    session.OnGoal(ClockAt(1));
    EXPECT_FALSE(session.OnBtStatus(Bt("FollowPath/Recover", "IDLE", "RUNNING", 2)));
    EXPECT_TRUE(session.state().open);
}

TEST(Signals, TaskChannelsAndLocalizationFeedback) {
    EXPECT_TRUE(IsTaskGoalChannel("/autonomy/task/localization/goal"));
    EXPECT_TRUE(IsTaskGoalChannel("/goal_pose"));
    EXPECT_FALSE(IsTaskGoalChannel("/odom"));
    EXPECT_TRUE(IsTaskFeedbackChannel("/autonomy/task/localization/feedback"));

    autonomy::task::proto::LocalizationFeedback fb;
    fb.set_status(autonomy::task::proto::LOCALIZATION_STATUS_FAILED);
    fb.set_localization_quality(0.1f);
    Sample s;
    s.channel = "/autonomy/task/localization/feedback";
    s.payload = fb.SerializeAsString();
    auto parsed = ParseTaskResult(s);
    ASSERT_TRUE(parsed);
    EXPECT_EQ(parsed->status(),
              automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_FAILED);
    auto quality = ParseLocalizationQuality(s);
    ASSERT_TRUE(quality);
    EXPECT_NEAR(*quality, 0.1, 1e-5);
}

TEST(Signals, VoiceFeedbackFailed) {
    autonomy::task::proto::VoiceFeedback fb;
    fb.set_status(autonomy::task::proto::VOICE_STATUS_FAILED);
    Sample s;
    s.channel = "/autonomy/task/voice/feedback";
    s.payload = fb.SerializeAsString();
    auto parsed = ParseTaskResult(s);
    ASSERT_TRUE(parsed);
    EXPECT_EQ(parsed->status(),
              automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_FAILED);
}

TEST(TriggerEngine, LocalizationQualityAndTaskBt) {
    proto::RecorderOptions opts;
    opts.set_localization_quality_floor(0.3);
    TriggerEngine eng(opts);
    auto low = eng.OnLocalizationQuality(0.1, ClockAt(1));
    ASSERT_TRUE(low);
    EXPECT_EQ(low->kind(), proto::TRIGGER_TASK_FAILURE);
    EXPECT_FALSE(eng.OnLocalizationQuality(0.1, ClockAt(2)));
    EXPECT_FALSE(eng.OnLocalizationQuality(0.9, ClockAt(3)));
    auto again = eng.OnLocalizationQuality(0.05, ClockAt(4));
    ASSERT_TRUE(again);

    auto loc_fail =
        eng.OnBtStatus("Localization", "RUNNING", "FAILURE", ClockAt(5));
    ASSERT_TRUE(loc_fail);
    EXPECT_EQ(loc_fail->kind(), proto::TRIGGER_TASK_FAILURE);
}

TEST(McapWriter, RoundtripAndCrashRecover) {
    const auto dir = std::filesystem::temp_directory_path() / "autonomy_recorder";
    std::filesystem::create_directories(dir);
    const auto path = (dir / "ep.mcap").string();

    proto::EpisodeHeader header;
    header.set_episode_id("ep_test");
    header.mutable_version()->set_policy_version("p");
    header.mutable_version()->set_env_version("e");
    header.mutable_version()->set_git_sha("g");
    header.mutable_version()->set_seed("1");
    header.set_admitted(true);
    header.set_time_source(proto::TIME_SOURCE_SOFTWARE);

    {
        McapWriter w;
        ASSERT_TRUE(w.Open(path, header));
        ASSERT_TRUE(w.WriteSample(MakeSample("/odom", 100, std::string(16, 'x'))));
        ASSERT_TRUE(w.WriteSample(MakeSample("/cmd_vel", 200, std::string(8, 'y'))));
        ASSERT_TRUE(w.Close());
    }
    auto msgs = RecoverMcap(path);
    ASSERT_EQ(msgs.size(), 2U);
    EXPECT_EQ(msgs[0].topic, "/odom");
    EXPECT_EQ(msgs[0].payload, std::string(16, 'x'));
    EXPECT_EQ(msgs[1].topic, "/cmd_vel");
}

TEST(EventGraph, FailureSourceAndPath) {
    const std::vector<BtTransition> ts = {
        Bt("NavigateToPose", "IDLE", "RUNNING", 1),
        Bt("ComputePath", "IDLE", "SUCCESS", 2),
        Bt("CheckCollision", "RUNNING", "FAILURE", 3),
    };
    proto::TriggerEvent trg;
    trg.set_kind(proto::TRIGGER_BT_RECOVER);
    trg.set_evidence_node_id("trg_recover");
    *trg.mutable_clock() = ClockAt(4);
    trg.set_detail("recover");
    const auto graph = BuildEventGraph(ts, {trg});
    EXPECT_FALSE(graph.failure_source_id().empty());
    EXPECT_GE(graph.nodes_size(), 3);
    EXPECT_GE(graph.edges_size(), 2);
    bool context = false;
    for (const auto& e : graph.edges()) {
        if (e.relation() == "context" && e.to_id() == "trg_recover") {
            context = true;
        }
        EXPECT_NE(e.relation(), "causal");
    }
    EXPECT_TRUE(context);
}

TEST(RootCause, RankAndRerollGate) {
    proto::EventGraph graph;
    auto* a = graph.add_nodes();
    a->set_id("obs");
    a->set_kind(proto::NODE_OBSERVATION);
    a->set_label("perception");
    auto* b = graph.add_nodes();
    b->set_id("fail");
    b->set_kind(proto::NODE_FAILURE_SOURCE);
    b->set_label("collision");
    auto* e = graph.add_edges();
    e->set_from_id("obs");
    e->set_to_id("fail");
    graph.set_failure_source_id("fail");

    auto cards = RankRootCauses(graph);
    ASSERT_FALSE(cards.empty());
    EXPECT_EQ(cards.front().evidence_node_id(), "fail");
    EXPECT_GT(cards.front().confidence(), cards.back().confidence());

    std::vector<HistoricalEpisode> fails{{"s1", false}, {"s2", false}};
    std::vector<HistoricalEpisode> ok{{"s3", true}};
    auto candidate = [](const HistoricalEpisode& ep) {
        return ep.scene_id != "s3" ? true : true;
    };
    EXPECT_TRUE(RerollHypothesis(candidate, fails, ok, 0.05, &cards.front()));
    EXPECT_TRUE(cards.front().reroll_passed());

    auto bad = [](const HistoricalEpisode& ep) {
        return ep.scene_id == "never";
    };
    proto::RootCauseCard card = cards.front();
    EXPECT_FALSE(RerollHypothesis(bad, fails, ok, 0.05, &card));
}

TEST(BtAttribution, LastFailureNode) {
    const std::vector<BtTransition> ts = {
        Bt("ComputePath", "IDLE", "SUCCESS", 1),
        Bt("CheckCollision", "RUNNING", "FAILURE", 2),
        Bt("Recover", "IDLE", "RUNNING", 3),
    };
    auto fail = AttributeBtFailure(ts);
    ASSERT_TRUE(fail);
    EXPECT_EQ(fail->node_name(), "CheckCollision");
    EXPECT_EQ(fail->condition(), "post");
}

TEST(HashChain, DetectsTamper) {
    SafetyHashChain chain("robot-a");
    proto::SafetyEvent e;
    e.set_event_id("e1");
    e.set_kind("estop");
    e.set_episode_id("ep1");
    *e.mutable_clock() = ClockAt(1);
    auto a = chain.Append(e);
    e.set_event_id("e2");
    auto b = chain.Append(e);
    EXPECT_TRUE(chain.Verify({a, b}));
    b.set_kind("mutated");
    EXPECT_FALSE(chain.Verify({a, b}));
    EXPECT_EQ(chain.Seal().size(), 64U);
}

TEST(Observation, RequiresEvidenceNode) {
    proto::Observation obs;
    obs.set_description("looks wrong");
    EXPECT_FALSE(AcceptObservation(obs));
    obs.add_evidence_node_ids("bt_1");
    EXPECT_TRUE(AcceptObservation(obs));
}

TEST(Replay, ActionMatchAndFactualGate) {
    proto::ActionEvent rec;
    rec.set_type("cmd_vel");
    rec.set_target("base");
    rec.set_params("0.2");
    proto::ActionEvent rep = rec;
    EXPECT_DOUBLE_EQ(ActionMatch({rec}, {rep}, 1e-6), 1.0);
    auto report = FactualReplay("ep", true, {rec}, {rep}, 0.95);
    EXPECT_TRUE(report.valid());
    auto bad = FactualReplay("ep", false, {rec}, {rep}, 0.95);
    EXPECT_FALSE(bad.valid());
}

TEST(Metrics, MpsMpdAndCoverage) {
    std::vector<EpisodeMetric> eps(20);
    for (size_t i = 0; i < eps.size(); ++i) {
        eps[i].success = i > 1;
        eps[i].stupid = i == 0;
        eps[i].dangerous = i == 1;
        eps[i].attributed = !eps[i].success;
        eps[i].latency_sec = static_cast<double>(i) * 0.01;
    }
    const auto snap = ComputeMetrics(eps);
    EXPECT_NEAR(snap.mps, 0.05, 1e-9);
    EXPECT_NEAR(snap.mpd, 0.05, 1e-9);
    EXPECT_DOUBLE_EQ(snap.attribution_coverage, 1.0);
    EXPECT_EQ(snap.n, 20U);
}

TEST(Metrics, JsonlRoundTripAndGateFile) {
    const auto dir = std::filesystem::temp_directory_path() / "rec_metrics";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    EpisodeMetric m;
    m.success = true;
    m.latency_sec = 0.2;
    const auto path = (dir / "episodes.jsonl").string();
    ASSERT_TRUE(AppendEpisodeMetric(path, m));
    ASSERT_TRUE(AppendEpisodeMetric(path, m));
    const auto loaded = LoadEpisodeMetrics(path, 256);
    ASSERT_EQ(loaded.size(), 2U);
    proto::PublishGate gate;
    gate.set_passed(true);
    MetricSnapshot snap;
    snap.n = 2;
    const auto gpath = (dir / "latest.json").string();
    ASSERT_TRUE(WriteGateSnapshot(gpath, gate, snap, "ep1"));
    std::ifstream in(gpath);
    auto j = nlohmann::json::parse(in);
    EXPECT_TRUE(j["publish_allowed"].get<bool>());
    EXPECT_EQ(j["episode_id"], "ep1");
}

TEST(Player, ReplayProfileSkips) {
    EXPECT_FALSE(SkipReplayTopic(ReplayProfile::kOpenLoop, "/cmd_vel"));
    EXPECT_TRUE(SkipReplayTopic(ReplayProfile::kMidStack, "/cmd_vel"));
    EXPECT_FALSE(SkipReplayTopic(ReplayProfile::kMidStack, "/odom"));
    EXPECT_TRUE(SkipReplayTopic(ReplayProfile::kSensorIn, "/plan"));
    EXPECT_TRUE(
        SkipReplayTopic(ReplayProfile::kSensorIn, "/perception/lidar_bev"));
    EXPECT_FALSE(SkipReplayTopic(ReplayProfile::kSensorIn, "/odom"));
    EXPECT_EQ(ParseReplayProfile("sensor_in"), ReplayProfile::kSensorIn);
    EXPECT_EQ(ParseReplayProfile("mid_stack"), ReplayProfile::kMidStack);
}

TEST(Sidecar, HazardPointer) {
    const auto root = std::filesystem::temp_directory_path() / "log_root";
    std::filesystem::remove_all(root);
    ASSERT_TRUE(WriteHazardPointer(root.string(), "ep42", "/tmp/ep.mcap",
                                   "/tmp/ep.json"));
    EXPECT_TRUE(std::filesystem::exists(root / "events" / "recorder_ep42" /
                                        "manifest.json"));
}

TEST(Pipeline, EvidenceChainWalkthrough) {
    EpisodeBundle bundle;
    bundle.header.set_episode_id("ep_walk");
    bundle.header.mutable_version()->set_policy_version("p1");
    bundle.header.mutable_version()->set_env_version("e1");
    bundle.header.mutable_version()->set_git_sha("deadbeef");
    bundle.header.mutable_version()->set_seed("7");
    bundle.header.set_admitted(true);
    bundle.transitions = {
        Bt("NavigateToPose", "IDLE", "RUNNING", 1),
        Bt("CheckCollision", "RUNNING", "FAILURE", 2),
        Bt("Recover", "IDLE", "RUNNING", 3),
    };
    proto::TriggerEvent trg;
    trg.set_kind(proto::TRIGGER_BT_RECOVER);
    trg.set_evidence_node_id("trg_recover");
    *trg.mutable_clock() = ClockAt(3);
    trg.set_detail("recover");
    bundle.triggers = {trg};

    proto::Observation obs;
    obs.set_observation_id("o1");
    obs.set_description("collision at corner");
    obs.add_evidence_node_ids("trg_recover");
    bundle.observations = {obs};

    proto::ActionEvent act;
    act.set_sequence(1);
    act.set_type("cmd_vel");
    act.set_target("base");
    act.set_params("0.0");
    bundle.recorded_actions = {act};
    bundle.replayed_actions = {act};
    bundle.bit_exact = true;
    bundle.fail_set = {{"s1", false}};
    bundle.success_set = {{"s2", true}};
    bundle.reroll = [](const HistoricalEpisode&) { return true; };

    proto::SafetyEvent sev;
    sev.set_event_id("estop1");
    sev.set_kind("estop");
    sev.set_episode_id("ep_walk");
    *sev.mutable_clock() = ClockAt(4);
    *sev.mutable_version() = bundle.header.version();
    bundle.safety_events = {sev};

    const auto sidecar = RunEvidencePipeline(bundle, 0.95, 0.5);
    EXPECT_TRUE(sidecar.header().admitted());
    EXPECT_FALSE(sidecar.graph().failure_source_id().empty());
    EXPECT_EQ(sidecar.bt_failure().node_name(), "CheckCollision");
    EXPECT_EQ(sidecar.observations_size(), 1);
    ASSERT_GT(sidecar.root_causes_size(), 0);
    EXPECT_TRUE(sidecar.root_causes(0).reroll_passed());
    EXPECT_TRUE(sidecar.replay().valid());
    EXPECT_TRUE(sidecar.gate().passed());
    EXPECT_EQ(sidecar.safety_events_size(), 1);

    bundle.replayed_actions.clear();
    const auto pending = RunEvidencePipeline(bundle, 0.95, 0.5);
    EXPECT_EQ(pending.replay().reason(), "pending_replay");
    EXPECT_FALSE(pending.replay().valid());
    EXPECT_FALSE(pending.gate().passed());
}

TEST(Pipeline, FleetHistoryAffectsCoverageGate) {
    EpisodeBundle bundle;
    bundle.header.set_episode_id("ep_fleet");
    bundle.header.mutable_version()->set_policy_version("p1");
    bundle.header.mutable_version()->set_env_version("e1");
    bundle.header.mutable_version()->set_git_sha("deadbeef");
    bundle.header.mutable_version()->set_seed("7");
    bundle.transitions = {Bt("CheckCollision", "RUNNING", "FAILURE", 2)};
    proto::ActionEvent act;
    act.set_type("cmd_vel");
    bundle.recorded_actions = {act};
    bundle.replayed_actions = {act};
    bundle.bit_exact = true;
    for (int i = 0; i < 8; ++i) {
        EpisodeMetric hist;
        hist.stupid = true;
        hist.success = false;
        hist.attributed = false;
        bundle.history.push_back(hist);
    }
    const auto sidecar = RunEvidencePipeline(bundle, 0.95, 0.8);
    EXPECT_FALSE(sidecar.gate().attribution_coverage_ok());
    EXPECT_FALSE(sidecar.gate().passed());
}

TEST(Signals, PathDeviationAndTwist) {
    automsgs::msgs::geometry_msgs::Pose2D p;
    p.set_y(1.0);
    automsgs::msgs::nav_msgs::Path path;
    path.add_poses()->mutable_pose()->mutable_position()->set_x(0.0);
    path.add_poses()->mutable_pose()->mutable_position()->set_x(2.0);
    EXPECT_NEAR(DistanceToPath(p, path), 1.0, 1e-9);

    automsgs::msgs::geometry_msgs::Twist tw;
    tw.mutable_linear()->set_x(0.3);
    tw.mutable_angular()->set_z(-0.1);
    Sample s;
    s.channel = "/cmd_vel";
    s.payload = tw.SerializeAsString();
    auto parsed = ParseTwist(s);
    ASSERT_TRUE(parsed);
    EXPECT_NEAR(parsed->linear().x(), 0.3, 1e-6);

    automsgs::msgs::geometry_msgs::TwistStamped ts;
    ts.mutable_header()->set_frame_id("base_link");
    ts.mutable_twist()->mutable_linear()->set_x(0.4);
    ts.mutable_twist()->mutable_angular()->set_z(0.05);
    Sample stamped;
    stamped.channel = "/cmd_vel";
    stamped.message_type = "automsgs.msgs.geometry_msgs.TwistStamped";
    stamped.payload = ts.SerializeAsString();
    auto pst = ParseTwist(stamped);
    ASSERT_TRUE(pst);
    EXPECT_NEAR(pst->linear().x(), 0.4, 1e-6);
    EXPECT_NEAR(pst->angular().z(), 0.05, 1e-6);

    PathTracker tracker;
    tracker.SetPath(path);
    tracker.SetOdom(p);
    ASSERT_TRUE(tracker.Deviation());
    EXPECT_NEAR(*tracker.Deviation(), 1.0, 1e-9);
}

TEST(ValueTier, P0UploadsP3Samples) {
    auto p0 = DecideUpload(proto::PRIORITY_P0, "ep", "nav", 0.5, 0.1);
    EXPECT_TRUE(p0.upload);
    EXPECT_EQ(p0.policy, "immediate");
    auto p1 = DecideUpload(proto::PRIORITY_P1, "ep", "nav", 0.5, 0.1);
    EXPECT_TRUE(p1.upload);
    EXPECT_FALSE(p1.expert_demo);
}

TEST(HashChain, PersistRoundtrip) {
    const auto path =
        (std::filesystem::temp_directory_path() / "autonomy_chain.bin").string();
    SafetyHashChain a("r1");
    proto::SafetyEvent e;
    e.set_event_id("e1");
    e.set_kind("estop");
    e.set_episode_id("ep");
    *e.mutable_clock() = ClockAt(1);
    a.Append(e);
    ASSERT_TRUE(a.Save(path));
    SafetyHashChain b("r1");
    ASSERT_TRUE(b.Load(path));
    EXPECT_EQ(b.size(), 1U);
    EXPECT_TRUE(b.Verify(b.events()));
}

TEST(Sidecar, JsonlObservationsRequireNodes) {
    const auto dir = std::filesystem::temp_directory_path() / "autonomy_obs";
    std::filesystem::create_directories(dir);
    const auto path = (dir / "obs.jsonl").string();
    {
        std::ofstream out(path);
        out << R"({"observation_id":"o1","description":"x","evidence_node_ids":["n1"]})"
            << "\n";
    }
    auto loaded = LoadObservationsJsonl(path);
    ASSERT_EQ(loaded.size(), 1U);
    EXPECT_TRUE(AcceptObservation(loaded[0]));
}

TEST(Convert, McapToLeRobotJsonl) {
    const auto dir = std::filesystem::temp_directory_path() / "autonomy_lerobot";
    std::filesystem::create_directories(dir);
    const auto mcap = (dir / "ep.mcap").string();
    const auto out = (dir / "ds").string();

    proto::EpisodeHeader header;
    header.set_episode_id("ep_cv");
    header.mutable_version()->set_policy_version("p");
    header.mutable_version()->set_env_version("e");
    header.mutable_version()->set_git_sha("g");
    header.mutable_version()->set_seed("1");
    header.set_admitted(true);

    automsgs::msgs::nav_msgs::Odometry odom;
    odom.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_x(
        1.0);
    automsgs::msgs::geometry_msgs::Twist tw;
    tw.mutable_linear()->set_x(0.2);

    McapWriter w;
    ASSERT_TRUE(w.Open(mcap, header));
    Sample so = MakeSample("/odom", 100, odom.SerializeAsString());
    Sample sc = MakeSample("/cmd_vel", 200, tw.SerializeAsString());
    ASSERT_TRUE(w.WriteSample(so));
    ASSERT_TRUE(w.WriteSample(sc));
    ASSERT_TRUE(w.Close());
    ASSERT_TRUE(ConvertMcapToLeRobot(mcap, out, 10.0));
    EXPECT_TRUE(std::filesystem::exists(
        std::filesystem::path(out) / "meta" / "info.json"));
    EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(out) / "data" /
                                        "chunk-000" / "episode_000000.jsonl"));

    const auto overlay = (dir / "overlay.json").string();
    const auto report = CompareMcaps(mcap, mcap, 0.95, true);
    EXPECT_TRUE(report.valid());
    EXPECT_TRUE(WriteReplayOverlay(overlay, report));
}

TEST(SafetyMonitor, EvaluateCommand) {
    SafetyCheckInput in;
    in.cmd.mutable_linear()->set_x(0.3);
    in.max_vx = 2.0;
    in.max_wz = 2.0;
    EXPECT_EQ(EvaluateCommand(in), SafetyVerdict::kPass);
    in.cmd.mutable_linear()->set_x(3.0);
    EXPECT_EQ(EvaluateCommand(in), SafetyVerdict::kFallback);
    in.latch_already = true;
    EXPECT_EQ(EvaluateCommand(in), SafetyVerdict::kPass);
    in.latch_already = false;
    in.cmd.mutable_linear()->set_x(0.1);
    in.teleop_active = true;
    EXPECT_EQ(EvaluateCommand(in), SafetyVerdict::kHandover);
}

TEST(Sidecar, JsonContainsGraphEdges) {
    proto::EpisodeSidecar sidecar;
    sidecar.mutable_header()->set_episode_id("ep");
    auto* node = sidecar.mutable_graph()->add_nodes();
    node->set_id("n1");
    node->set_label("obs");
    auto* edge = sidecar.mutable_graph()->add_edges();
    edge->set_from_id("n1");
    edge->set_to_id("n2");
    edge->set_relation("causes");
    const auto path =
        (std::filesystem::temp_directory_path() / "ep.sidecar.json").string();
    ASSERT_TRUE(WriteSidecarJson(path, sidecar));
    std::ifstream in(path);
    auto j = nlohmann::json::parse(in);
    ASSERT_EQ(j["graph_edges"].size(), 1U);
    EXPECT_EQ(j["graph_edges"][0]["from_id"], "n1");
}

TEST(Upload, CopiesAdmittedManifest) {
    const auto src = std::filesystem::temp_directory_path() / "up_src";
    const auto dest = std::filesystem::temp_directory_path() / "up_dest";
    std::filesystem::remove_all(src);
    std::filesystem::remove_all(dest);
    std::filesystem::create_directories(src);
    {
        std::ofstream m(src / "ep.manifest.json");
        m << R"({"episode_id":"ep","admitted":true,"upload":true})" << "\n";
        std::ofstream s(src / "ep.sidecar.json");
        s << "{}\n";
    }
    UploadReport report;
    ASSERT_TRUE(UploadEpisodes(src.string(), dest.string(), &report));
    EXPECT_GE(report.copied, 1);
    EXPECT_TRUE(std::filesystem::exists(dest / "ep.sidecar.json"));
}

TEST(Observation, AppendRejectsUnbound) {
    const auto path =
        (std::filesystem::temp_directory_path() / "obs_append.jsonl").string();
    proto::Observation bad;
    bad.set_description("no nodes");
    EXPECT_FALSE(AppendObservationJsonl(path, bad));
    proto::Observation ok;
    ok.set_observation_id("o2");
    ok.set_description("bound");
    ok.add_evidence_node_ids("n1");
    EXPECT_TRUE(AppendObservationJsonl(path, ok));
}

}  // namespace
}  // namespace recorder
}  // namespace system
}  // namespace autonomy
