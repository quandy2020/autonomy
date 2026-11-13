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
 * @file recorder.cpp
 * @brief Bypass recorder: flag-gated parse, TakeAll flush, MCAP + sidecar.
 */

#include "autonomy/system/recorder/recorder.hpp"

#include <cstddef>
#include <optional>
#include <unordered_map>

#include "autolink/common/environment.hpp"
#include "autolink/common/file.hpp"
#include "autolink/common/log.hpp"
#include "automsgs/msgs/nav_msgs/behavior_tree.pb.h"
#include "automsgs/msgs/vehicle_msgs/robot_task_status.pb.h"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/system/recorder/clock.hpp"
#include "autonomy/system/recorder/metrics.hpp"
#include "autonomy/system/recorder/pipeline.hpp"
#include "autonomy/system/recorder/sidecar.hpp"
#include "autonomy/system/recorder/signals.hpp"
#include "autonomy/system/recorder/value_tier.hpp"

namespace autonomy {
namespace system {
namespace recorder {

namespace {

void ApplyDefaults(proto::RecorderOptions* opts) {
    if (opts->buffer_sec() <= 0) {
        opts->set_buffer_sec(30.0);
    }
    if (opts->bt_log_channel().empty()) {
        opts->set_bt_log_channel("/behavior_tree/log");
    }
    if (opts->cmd_vel_channel().empty()) {
        opts->set_cmd_vel_channel("/cmd_vel");
    }
    if (opts->odom_channel().empty()) {
        opts->set_odom_channel("/odom");
    }
    if (opts->plan_channel().empty()) {
        opts->set_plan_channel("/plan");
    }
    if (opts->teleop_goal_channel().empty()) {
        opts->set_teleop_goal_channel("/autonomy/task/teleop/goal");
    }
    if (opts->p2_sample_rate() <= 0) {
        opts->set_p2_sample_rate(0.5);
    }
    if (opts->p3_sample_rate() <= 0) {
        opts->set_p3_sample_rate(0.1);
    }
    if (opts->max_cmd_vx() <= 0) {
        opts->set_max_cmd_vx(2.0);
    }
    if (opts->max_cmd_wz() <= 0) {
        opts->set_max_cmd_wz(2.0);
    }
    if (opts->action_match_threshold() <= 0) {
        opts->set_action_match_threshold(0.95);
    }
    if (opts->attribution_coverage_floor() <= 0) {
        opts->set_attribution_coverage_floor(0.8);
    }
    if (opts->channels_size() == 0) {
        for (const char* name :
             {"/odom", "/tf", "/tf_static", "/scan", "/imu", "/map",
              "/camera/image_raw", "/cmd_vel", "/navigation/cmd_vel_candidate",
              "/plan", "/behavior_tree/log", "/perception/lidar_bev",
              "/perception/policy_raw", "/perception/follow/grid",
              "/global_costmap", "/atlas/occupancy_grid",
              "/autonomy/task/navigation/goal",
              "/autonomy/task/navigation/feedback",
              "/autonomy/task/teleop/goal", "/autonomy/task/teleop/feedback",
              "/autonomy/task/localization/goal",
              "/autonomy/task/localization/feedback",
              "/autonomy/task/mapping/goal", "/autonomy/task/mapping/feedback",
              "/autonomy/task/charging/goal",
              "/autonomy/task/charging/feedback",
              "/autonomy/task/tracking/goal",
              "/autonomy/task/tracking/feedback",
              "/autonomy/task/exploration/goal",
              "/autonomy/task/exploration/feedback",
              "/autonomy/task/voice/goal",
              "/autonomy/task/voice/feedback", "/goal_pose"}) {
            opts->add_channels()->set_name(name);
        }
    }
    if (opts->flush_period_sec() <= 0) {
        opts->set_flush_period_sec(1.0);
    }
    if (opts->localization_quality_floor() <= 0) {
        opts->set_localization_quality_floor(0.3);
    }
    if (opts->output_dir().empty()) {
        opts->set_output_dir(DefaultRecordRoot());
    }
}

std::optional<EpisodeState> MaybeParseBt(
    const Sample& sample, const std::string& bt_channel,
    EpisodeTracker* episodes, TriggerEngine* triggers,
    std::vector<proto::TriggerEvent>* pending) {
    if (sample.channel != bt_channel) {
        return std::nullopt;
    }
    automsgs::msgs::nav_msgs::BehaviorTreeLog log;
    if (!log.ParseFromArray(sample.payload.data(),
                            static_cast<int>(sample.payload.size()))) {
        return std::nullopt;
    }
    std::optional<EpisodeState> closed;
    for (const auto& ev : log.event_log()) {
        BtTransition t;
        t.node_name = ev.node_name();
        t.previous = ev.previous_status();
        t.current = ev.current_status();
        t.clock = sample.clock;
        if (auto done = episodes->OnBtStatus(t)) {
            closed = std::move(done);
        }
        if (auto trg = triggers->OnBtStatus(t.node_name, t.previous, t.current,
                                            t.clock)) {
            pending->push_back(*trg);
        }
    }
    return closed;
}

}  // namespace

std::string DefaultRecordRoot() {
    const std::string root = autolink::common::GetEnv("AUTONOMY_RECORD_ROOT");
    if (!root.empty()) {
        return root;
    }
    const std::string home = autolink::common::GetEnv("HOME");
    if (!home.empty()) {
        return home + "/.autonomy/records";
    }
    return "/tmp/autonomy/records";
}

proto::RecorderOptions LoadRecorderOptions(const std::string& conf_file) {
    proto::RecorderOptions options;
    const std::string file =
        conf_file.empty() ? std::string("recorder.pb.txt") : conf_file;
    if (autolink::common::PathIsAbsolute(file) ||
        file.find('/') != std::string::npos) {
        std::string path;
        if (autolink::common::GetFilePathWithEnv(file, "AUTONOMY_CONF_PATH",
                                                 &path) ||
            autolink::common::PathExists(file)) {
            if (path.empty()) {
                path = file;
            }
            if (!autolink::common::GetProtoFromFile(path, &options)) {
                AWARN << "recorder: failed to load " << path;
            }
            ApplyDefaults(&options);
            return options;
        }
    }
    if (!common::LoadModuleConf("system", file, &options)) {
        AWARN << "recorder: using defaults, failed to load " << file;
    }
    ApplyDefaults(&options);
    return options;
}

bool Recorder::Configure(const proto::RecorderOptions& options) {
    options_ = options;
    ApplyDefaults(&options_);
    version_ = MakeVersionTuple(options_);
    time_source_ = options_.time_source() == proto::TIME_SOURCE_UNSPECIFIED
                       ? proto::TIME_SOURCE_SOFTWARE
                       : options_.time_source();
    const int64_t window_ns =
        static_cast<int64_t>(options_.buffer_sec() * 1e9);
    buffer_ = std::make_unique<RollingBuffer>(window_ns);
    triggers_ = std::make_unique<TriggerEngine>(options_);
    chain_ = SafetyHashChain(options_.robot_id());
    const std::string chain_path =
        options_.output_dir() + "/safety/chain.bin";
    if (autolink::common::PathExists(chain_path)) {
        chain_.Load(chain_path);
    }
    return true;
}

bool Recorder::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!buffer_ || !triggers_) {
        return false;
    }
    if (options_.full_dump()) {
        std::lock_guard<std::mutex> lock(mutex_);
        episodes_.SetSessionMode(true);
        episodes_.OnGoal(NowDualClock(time_source_));
        AINFO << "recorder: full_dump session " << episodes_.state().episode_id
              << " flush_period=" << options_.flush_period_sec() << "s";
    }
    return subscriber_.Attach(
        node, options_, [this](Sample sample) { HandleSample(std::move(sample)); });
}

void Recorder::HandleSample(Sample sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t n_triggers = pending_triggers_.size();
    const uint16_t flags = sample.flags != 0
                               ? sample.flags
                               : SampleFlagsFor(sample.channel, options_);
    if (flags & kSampleGoal) {
        episodes_.OnGoal(sample.clock);
    }
    std::optional<EpisodeState> closed;
    if (flags & kSampleBt) {
        closed = MaybeParseBt(sample, options_.bt_log_channel(), &episodes_,
                              triggers_.get(), &pending_triggers_);
    }
    if (auto trg = triggers_->OnSample(sample)) {
        pending_triggers_.push_back(*trg);
    }

    if (flags & kSampleOdom) {
        if (auto pose = ParseOdom(sample)) {
            path_tracker_.SetOdom(*pose);
            if (auto d = path_tracker_.Deviation()) {
                if (auto trg = triggers_->OnPathDeviation(*d, sample.clock)) {
                    pending_triggers_.push_back(*trg);
                }
            }
        }
    }
    if (flags & kSamplePlan) {
        if (auto path = ParsePath(sample)) {
            path_tracker_.SetPath(*path);
        }
    }
    if (flags & kSampleCost) {
        if (auto c = ParseCost(sample)) {
            if (auto trg = triggers_->OnCost(*c, sample.clock)) {
                pending_triggers_.push_back(*trg);
            }
        }
    }
    if (flags & kSampleCmdVel) {
        if (auto tw = ParseTwist(sample)) {
            actions_.push_back(TwistToAction(
                *tw, static_cast<uint32_t>(actions_.size()),
                sample.clock.monotonic_ns()));
            constexpr size_t kMaxActions = 8192;
            if (actions_.size() > kMaxActions) {
                actions_.erase(actions_.begin(),
                               actions_.begin() + static_cast<std::ptrdiff_t>(
                                                      kMaxActions / 2));
            }
        }
    }
    if ((flags & kSampleTeleop) && episodes_.state().open) {
        if (auto trg = triggers_->OnTakeover(sample.clock, sample.channel)) {
            pending_triggers_.push_back(*trg);
        }
    }
    if (flags & kSampleFeedback) {
        if (auto result = ParseTaskResult(sample)) {
            using automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_CANCELED;
            using automsgs::msgs::vehicle_msgs::ROBOT_TASK_STATUS_FAILED;
            if (result->status() == ROBOT_TASK_STATUS_FAILED ||
                result->status() == ROBOT_TASK_STATUS_CANCELED) {
                if (auto trg = triggers_->OnTaskFailure(
                        sample.clock, result->message(), sample.channel)) {
                    pending_triggers_.push_back(*trg);
                }
                if (!options_.full_dump() && episodes_.state().open) {
                    closed = episodes_.Close(sample.clock, result->message());
                }
            }
        }
        if (auto q = ParseLocalizationQuality(sample)) {
            if (auto trg = triggers_->OnLocalizationQuality(
                    static_cast<double>(*q), sample.clock)) {
                pending_triggers_.push_back(*trg);
            }
        }
    }

    buffer_->Push(std::move(sample));
    if (closed) {
        FlushLocked(closed->episode_id, true, &*closed);
        return;
    }
    if (pending_triggers_.size() > n_triggers) {
        HandleTrigger(pending_triggers_.back());
    }
}

void Recorder::HandleTrigger(const proto::TriggerEvent& event) {
    if (!episodes_.state().open) {
        episodes_.OnGoal(event.clock());
    }
    FlushLocked(episodes_.state().episode_id, false, nullptr);
}

void Recorder::SpinOnce() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = NowDualClock(time_source_);
    if (triggers_) {
        for (auto& trg : triggers_->PollTimeouts(now)) {
            pending_triggers_.push_back(trg);
            HandleTrigger(trg);
        }
    }
    if (auto trg = triggers_->OnSafetyLatch(latch_.IsLatched(), latch_.Reason(),
                                            now)) {
        pending_triggers_.push_back(*trg);
        HandleTrigger(*trg);
    }
    if (options_.full_dump() && episodes_.state().open) {
        const int64_t period_ns = static_cast<int64_t>(
            options_.flush_period_sec() * 1e9);
        if (last_periodic_flush_ns_ == 0 ||
            now.monotonic_ns() - last_periodic_flush_ns_ >= period_ns) {
            FlushLocked(episodes_.state().episode_id, false, nullptr);
            last_periodic_flush_ns_ = now.monotonic_ns();
        }
    }
}

proto::EpisodeHeader Recorder::MakeHeaderLocked(
    const std::string& episode_id, const EpisodeState* closed) const {
    proto::EpisodeHeader header;
    header.set_episode_id(episode_id);
    *header.mutable_version() = version_.tuple;
    header.set_admitted(version_.admitted || !options_.require_version_tuple());
    for (const auto& m : version_.missing) {
        header.add_missing_fields(m);
    }
    header.set_robot_id(options_.robot_id());
    header.set_time_source(time_source_);
    const auto& st = closed != nullptr ? *closed : episodes_.state();
    header.set_bt_root(st.bt_root);
    *header.mutable_start() = st.start;
    *header.mutable_end() = NowDualClock(time_source_);
    header.set_priority(triggers_->Classify(pending_triggers_, false));
    if (options_.require_version_tuple() && !version_.admitted) {
        header.set_admitted(false);
    }
    return header;
}

std::string Recorder::OutputDir(bool admitted) const {
    return options_.output_dir() + (admitted ? "/episodes" : "/diagnostics");
}

void Recorder::FlushLocked(const std::string& episode_id, bool close_episode,
                           const EpisodeState* closed) {
    if (buffer_ == nullptr || episode_id.empty()) {
        return;
    }
    auto header = MakeHeaderLocked(episode_id, closed);
    const std::string dir = OutputDir(header.admitted());
    autolink::common::EnsureDirectory(dir);
    const std::string mcap_path = dir + "/" + episode_id + ".mcap";
    if (!writer_.is_open()) {
        if (!writer_.Open(mcap_path, header)) {
            AWARN << "recorder: failed to open " << mcap_path;
            return;
        }
    }
    auto batch = buffer_->TakeAll();
    for (const auto& sample : batch) {
        writer_.WriteSample(sample);
        last_flushed_ns_ = sample.clock.monotonic_ns();
    }
    last_path_ = writer_.path();
    if (!close_episode) {
        return;
    }

    const auto obs_path = dir + "/" + episode_id + ".observations.jsonl";
    auto loaded = LoadObservationsJsonl(obs_path);
    observations_.insert(observations_.end(), loaded.begin(), loaded.end());

    for (const auto& trg : pending_triggers_) {
        if (trg.kind() != proto::TRIGGER_ESTOP &&
            trg.kind() != proto::TRIGGER_TASK_FAILURE) {
            continue;
        }
        proto::SafetyEvent ev;
        ev.set_event_id(trg.evidence_node_id());
        ev.set_kind(proto::TriggerKind_Name(trg.kind()));
        ev.set_episode_id(episode_id);
        *ev.mutable_clock() = trg.clock();
        *ev.mutable_version() = header.version();
        ev.set_bt_node(header.bt_root());
        pending_safety_.push_back(chain_.Append(ev));
    }

    EpisodeBundle bundle;
    bundle.header = header;
    bundle.transitions = closed != nullptr ? closed->transitions
                                           : episodes_.state().transitions;
    bundle.triggers = pending_triggers_;
    bundle.observations = observations_;
    bundle.recorded_actions = actions_;
    bundle.bit_exact = false;
    bundle.safety_events = pending_safety_;
    const std::string metrics_dir = options_.output_dir() + "/metrics";
    autolink::common::EnsureDirectory(metrics_dir);
    const std::string metrics_path = metrics_dir + "/episodes.jsonl";
    bundle.history = LoadEpisodeMetrics(metrics_path);
    const auto sidecar = RunEvidencePipeline(
        bundle, options_.action_match_threshold(),
        options_.attribution_coverage_floor());
    writer_.Close();

    const auto upload = DecideUpload(
        header.priority(), episode_id, header.bt_root(),
        options_.full_dump() ? 1.0 : options_.p2_sample_rate(),
        options_.full_dump() ? 1.0 : options_.p3_sample_rate());
    autolink::common::SetProtoToBinaryFile(
        sidecar, dir + "/" + episode_id + ".sidecar.pb");
    WriteSidecarJson(dir + "/" + episode_id + ".sidecar.json", sidecar);
    WriteManifest(dir + "/" + episode_id + ".manifest.json", header, upload);
    WriteReplayOverlay(dir + "/" + episode_id + ".overlay.json",
                       sidecar.replay());
    AppendEpisodeMetric(metrics_path, MetricFromSidecar(sidecar));
    const std::string gate_dir = options_.output_dir() + "/gate";
    autolink::common::EnsureDirectory(gate_dir);
    auto fleet = bundle.history;
    fleet.push_back(MetricFromSidecar(sidecar));
    WriteGateSnapshot(gate_dir + "/latest.json", sidecar.gate(),
                      ComputeMetrics(fleet), episode_id);
    if (header.priority() == proto::PRIORITY_P0) {
        const std::string log_root = autolink::common::GetEnv("AUTONOMY_LOG_ROOT");
        if (!log_root.empty()) {
            WriteHazardPointer(log_root, episode_id, mcap_path,
                               dir + "/" + episode_id + ".sidecar.json");
        }
    }

    const std::string safety_dir = options_.output_dir() + "/safety";
    autolink::common::EnsureDirectory(safety_dir);
    chain_.Save(safety_dir + "/chain.bin");

    ++episode_count_;
    pending_triggers_.clear();
    observations_.clear();
    actions_.clear();
    pending_safety_.clear();
    last_flushed_ns_ = 0;
    last_periodic_flush_ns_ = 0;
    buffer_->Clear();
    AINFO << "recorder: flushed " << mcap_path
          << " admitted=" << header.admitted()
          << " upload=" << upload.upload;
}

void Recorder::Shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    subscriber_.Reset();
    if (episodes_.state().open) {
        auto closed = episodes_.Close(NowDualClock(time_source_), "shutdown");
        if (closed) {
            FlushLocked(closed->episode_id, true, &*closed);
        }
    } else if (writer_.is_open()) {
        writer_.Close();
    }
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
