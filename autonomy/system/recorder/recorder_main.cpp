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
 * @file recorder_main.cpp
 * @brief CLI for autonomy.recorder: record, safety, replay, convert, pipeline, gate.
 */

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "nlohmann/json.hpp"

#include "autolink/autolink.hpp"
#include "autolink/state.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/system/recorder/convert.hpp"
#include "autonomy/system/recorder/observation.hpp"
#include "autonomy/system/recorder/pipeline.hpp"
#include "autonomy/system/recorder/hash_chain.hpp"
#include "autonomy/system/recorder/player.hpp"
#include "autonomy/system/recorder/recorder.hpp"
#include "autonomy/system/recorder/safety_monitor.hpp"
#include "autonomy/system/recorder/sidecar.hpp"
#include "autonomy/system/recorder/upload.hpp"
#include "autonomy/system/safety/safety_latch.hpp"

namespace {

int RunRecord(const std::string& conf, bool full_dump) {
    if (!autolink::Init("autonomy.recorder")) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }
    auto opts = autonomy::system::recorder::LoadRecorderOptions(conf);
    if (full_dump) {
        opts.set_full_dump(true);
    }
    autonomy::system::recorder::Recorder recorder;
    if (!recorder.Configure(opts)) {
        LOG(ERROR) << "recorder configure failed";
        return EXIT_FAILURE;
    }
    auto node = autolink::CreateNode("recorder", "/autonomy/system/recorder");
    if (!node || !recorder.Start(node)) {
        LOG(WARNING) << "recorder: autolink attach incomplete; spinning anyway";
    }
    const auto& ver = recorder.version();
    LOG(INFO) << "autonomy.recorder running admitted=" << ver.admitted
              << " git_sha=" << ver.tuple.git_sha()
              << " missing=" << ver.missing.size()
              << " full_dump=" << opts.full_dump()
              << " (bypass subscriber, does not join control)";
    std::atomic<bool> running{true};
    std::thread poller([&]() {
        while (running.load()) {
            recorder.SpinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    autolink::WaitForShutdown();
    running.store(false);
    if (poller.joinable()) {
        poller.join();
    }
    recorder.Shutdown();
    return EXIT_SUCCESS;
}

int RunReplay(const std::string& mcap, double rate, bool hil,
              const std::string& profile_name, bool sensors_only) {
    using autonomy::system::recorder::ParseReplayProfile;
    using autonomy::system::recorder::PlayMcap;
    using autonomy::system::recorder::ReplayProfile;
    ReplayProfile profile = ParseReplayProfile(profile_name);
    if (hil) {
        autonomy::system::safety::SafetyLatch latch;
        if (latch.IsLatched()) {
            LOG(ERROR) << "HIL refused: SafetyLatch latched (" << latch.Reason()
                       << ")";
            return EXIT_FAILURE;
        }
        rate = 1.0;
        profile = ReplayProfile::kSensorIn;
        LOG(INFO) << "HIL: sensor_in republish at 1x (skips cmd_vel and mid-stack)";
    } else if (sensors_only) {
        profile = ReplayProfile::kMidStack;
    }
    if (!autolink::Init(hil ? "autonomy.recorder.hil"
                            : "autonomy.recorder.replay")) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }
    auto node =
        autolink::CreateNode("recorder_replay", "/autonomy/system/recorder");
    const bool ok = PlayMcap(node, mcap, rate, profile);
    autolink::WaitForShutdown();
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunGate(const std::string& records_dir) {
    const std::string path = records_dir + "/gate/latest.json";
    std::ifstream in(path);
    if (!in) {
        LOG(ERROR) << "gate snapshot missing: " << path;
        return EXIT_FAILURE;
    }
    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        LOG(ERROR) << "gate snapshot unreadable: " << e.what();
        return EXIT_FAILURE;
    }
    const bool passed = j.value("publish_allowed", false);
    std::string reason;
    if (j.contains("gate") && j["gate"].is_object()) {
        reason = j["gate"].value("reason", "");
    }
    LOG(INFO) << "publish_allowed=" << passed
              << " episode=" << j.value("episode_id", "")
              << " reason=" << reason;
    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunSafety(const std::string& conf) {
    if (!autolink::Init("autonomy.recorder.safety")) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }
    auto opts = autonomy::system::recorder::LoadRecorderOptions(conf);
    autonomy::system::recorder::SafetyMonitor monitor;
    if (!monitor.Configure(opts)) {
        LOG(ERROR) << "safety monitor configure failed";
        return EXIT_FAILURE;
    }
    auto node = autolink::CreateNode("recorder_safety",
                                     "/autonomy/system/recorder");
    if (!node || !monitor.Start(node)) {
        LOG(WARNING) << "safety_monitor: autolink attach incomplete";
    }
    LOG(INFO) << "autonomy.recorder safety running (latch-only, no cmd_vel "
                 "publish)";
    std::atomic<bool> running{true};
    std::thread poller([&]() {
        while (running.load()) {
            monitor.SpinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    autolink::WaitForShutdown();
    running.store(false);
    if (poller.joinable()) {
        poller.join();
    }
    monitor.Shutdown();
    return EXIT_SUCCESS;
}

int RunVerifyChain(const std::string& path) {
    autonomy::system::recorder::SafetyHashChain chain;
    if (!chain.Load(path)) {
        LOG(ERROR) << "hash chain invalid or unreadable: " << path;
        return EXIT_FAILURE;
    }
    LOG(INFO) << "hash chain ok entries=" << chain.size()
              << " seal=" << chain.Seal();
    return EXIT_SUCCESS;
}

int RunUpload(const std::string& src, const std::string& dest) {
    autonomy::system::recorder::UploadReport report;
    const bool ok =
        autonomy::system::recorder::UploadEpisodes(src, dest, &report);
    LOG(INFO) << "upload copied=" << report.copied
              << " skipped=" << report.skipped << " failed=" << report.failed
              << " " << report.reason;
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunConvert(const std::string& mcap, const std::string& out, double fps) {
    if (!autonomy::system::recorder::ConvertMcapToLeRobot(mcap, out, fps)) {
        LOG(ERROR) << "convert failed: " << mcap;
        return EXIT_FAILURE;
    }
    LOG(INFO) << "converted " << mcap << " -> " << out;
    return EXIT_SUCCESS;
}

int RunVerdict(const std::string& recorded, const std::string& replayed,
               const std::string& overlay, double match, bool factual) {
    const auto report = autonomy::system::recorder::CompareMcaps(
        recorded, replayed, match, factual);
    if (!overlay.empty()) {
        autonomy::system::recorder::WriteReplayOverlay(overlay, report);
    }
    LOG(INFO) << "verdict valid=" << report.valid()
              << " action_match=" << report.action_match()
              << " bit_exact=" << report.bit_exact()
              << " reason=" << report.reason();
    return report.valid() ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunObserve(const std::string& jsonl, const std::string& id,
               const std::string& desc, const std::vector<std::string>& nodes,
               const std::string& bt, int64_t t0, int64_t t1) {
    autonomy::system::recorder::proto::Observation obs;
    obs.set_observation_id(id);
    obs.set_description(desc);
    obs.set_bt_node(bt);
    obs.set_start_monotonic_ns(t0);
    obs.set_end_monotonic_ns(t1);
    for (const auto& n : nodes) {
        obs.add_evidence_node_ids(n);
    }
    if (!autonomy::system::recorder::AppendObservationJsonl(jsonl, obs)) {
        LOG(ERROR) << "observe rejected (need description + evidence_node_id)";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int RunPipeline(const std::string& sidecar_path, double match,
                double coverage) {
    autonomy::system::recorder::proto::EpisodeSidecar sidecar;
    std::ifstream in(sidecar_path, std::ios::binary);
    if (!in || !sidecar.ParseFromIstream(&in)) {
        LOG(ERROR) << "failed to read sidecar " << sidecar_path;
        return EXIT_FAILURE;
    }
    autonomy::system::recorder::EpisodeBundle bundle;
    bundle.header = sidecar.header();
    for (const auto& t : sidecar.triggers()) {
        bundle.triggers.push_back(t);
    }
    bundle.observations.assign(sidecar.observations().begin(),
                               sidecar.observations().end());
    bundle.safety_events.assign(sidecar.safety_events().begin(),
                                sidecar.safety_events().end());
    bundle.recorded_actions.assign(sidecar.recorded_actions().begin(),
                                   sidecar.recorded_actions().end());
    bundle.replayed_actions.assign(sidecar.replayed_actions().begin(),
                                   sidecar.replayed_actions().end());
    bundle.bit_exact = sidecar.replay().bit_exact();
    const auto out = autonomy::system::recorder::RunEvidencePipeline(
        bundle, match, coverage);
    LOG(INFO) << "pipeline gate=" << out.gate().passed()
              << " reason=" << out.gate().reason();
    return out.gate().passed() ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

/**
 * @brief Process entry: default record; subcommands for replay / safety / offline tools.
 * @param [in] argc: Argument count.
 * @param [in] argv: Argument vector.
 * @return EXIT_SUCCESS on success.
 */
int main(int argc, char** argv) {
    CLI::App app{"autonomy.recorder"};
    app.require_subcommand(0, 1);
    autonomy::common::BindCommonOptions(app);
    bool full_dump = false;
    app.add_flag("--full-dump", full_dump,
                 "Tester session: one episode, periodic MCAP flush, keep P2/P3");

    auto* replay = app.add_subcommand("replay", "SIL/HIL replay an episode MCAP");
    std::string replay_mcap;
    double replay_rate = 1.0;
    bool hil = false;
    bool sensors_only = false;
    std::string replay_profile = "open_loop";
    replay->add_option("-i,--mcap", replay_mcap, "Input MCAP")->required();
    replay->add_option("-r,--rate", replay_rate,
                       "1=realtime, 0=as-fast-as-possible");
    replay->add_option("--profile", replay_profile,
                       "open_loop | mid_stack | sensor_in");
    replay->add_flag("--hil", hil,
                     "HIL: 1x sensor_in; refuse if SafetyLatch is set");
    replay->add_flag("--sensors-only", sensors_only,
                     "Alias of --profile mid_stack (live controller keeps /cmd_vel)");

    auto* safety =
        app.add_subcommand("safety", "L4 independent latch-only safety monitor");

    auto* upload = app.add_subcommand(
        "upload", "Copy admitted P0–P3 episodes whose manifest.upload is true");
    std::string upload_src;
    std::string upload_dest;
    upload->add_option("-s,--src", upload_src, "episodes/ directory")->required();
    upload->add_option("-d,--dest", upload_dest, "destination directory")
        ->required();

    auto* verify_chain =
        app.add_subcommand("verify-chain", "Verify L4 safety hash chain.bin");
    std::string chain_path;
    verify_chain->add_option("-i,--chain", chain_path, "safety/chain.bin")
        ->required();

    auto* convert =
        app.add_subcommand("convert", "MCAP → LeRobot v2.1 JSONL dataset");
    std::string convert_mcap;
    std::string convert_out;
    double fps = 10.0;
    convert->add_option("-i,--mcap", convert_mcap, "Input MCAP")->required();
    convert->add_option("-o,--out", convert_out, "Output dataset dir")
        ->required();
    convert->add_option("--fps", fps);

    auto* pipeline =
        app.add_subcommand("pipeline", "Re-run L2–L6 on a sidecar.pb");
    std::string sidecar_path;
    double match = 0.95;
    double coverage = 0.8;
    pipeline->add_option("-i,--sidecar", sidecar_path, "sidecar.pb")->required();
    pipeline->add_option("--match", match);
    pipeline->add_option("--coverage", coverage);

    auto* verdict =
        app.add_subcommand("verdict", "L6: compare recorded vs replayed MCAP");
    std::string rec_mcap;
    std::string rep_mcap;
    std::string overlay_path;
    double verdict_match = 0.95;
    bool interventional = false;
    verdict->add_option("-a,--recorded", rec_mcap, "Recorded MCAP")->required();
    verdict->add_option("-b,--replayed", rep_mcap, "Replayed MCAP")->required();
    verdict->add_option("-o,--overlay", overlay_path, "Write overlay JSON");
    verdict->add_option("--match", verdict_match);
    verdict->add_flag("--interventional", interventional,
                      "Intervention replay (default factual)");

    auto* gate = app.add_subcommand(
        "gate", "Exit 0 iff $records/gate/latest.json publish_allowed");
    std::string gate_dir;
    gate->add_option("-d,--dir", gate_dir, "records root")->required();

    auto* observe = app.add_subcommand(
        "observe", "Append observation jsonl bound to evidence nodes");
    std::string obs_path;
    std::string obs_id;
    std::string obs_desc;
    std::string obs_bt;
    std::vector<std::string> obs_nodes;
    int64_t obs_t0 = 0;
    int64_t obs_t1 = 0;
    observe->add_option("-o,--jsonl", obs_path, "observations.jsonl")
        ->required();
    observe->add_option("--id", obs_id, "observation_id")->required();
    observe->add_option("-d,--desc", obs_desc, "description")->required();
    observe->add_option("-n,--node", obs_nodes, "evidence_node_id")->required();
    observe->add_option("--bt", obs_bt);
    observe->add_option("--from", obs_t0);
    observe->add_option("--to", obs_t1);

    autonomy::common::ParseOrExit(app, argc, argv);

    const std::string conf = autonomy::common::FLAGS_conf.empty()
                                 ? std::string("recorder.pb.txt")
                                 : autonomy::common::FLAGS_conf;
    if (replay->parsed()) {
        return RunReplay(replay_mcap, replay_rate, hil, replay_profile,
                         sensors_only);
    }
    if (gate->parsed()) {
        return RunGate(gate_dir);
    }
    if (safety->parsed()) {
        return RunSafety(conf);
    }
    if (upload->parsed()) {
        return RunUpload(upload_src, upload_dest);
    }
    if (verify_chain->parsed()) {
        return RunVerifyChain(chain_path);
    }
    if (convert->parsed()) {
        return RunConvert(convert_mcap, convert_out, fps);
    }
    if (pipeline->parsed()) {
        return RunPipeline(sidecar_path, match, coverage);
    }
    if (verdict->parsed()) {
        return RunVerdict(rec_mcap, rep_mcap, overlay_path, verdict_match,
                          !interventional);
    }
    if (observe->parsed()) {
        return RunObserve(obs_path, obs_id, obs_desc, obs_nodes, obs_bt, obs_t0,
                          obs_t1);
    }
    return RunRecord(conf, full_dump);
}
