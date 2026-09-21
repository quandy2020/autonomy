/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/ota/ota_agent.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>

#include "autolink/common/log.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/system/monitor/health_snapshot_store.hpp"
#include "autonomy/system/proto/ota_options.pb.h"
#include "autonomy/system/safety/safety_latch.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace system {
namespace ota {
namespace {

namespace fs = std::filesystem;

bool RunCmd(const std::string& cmd, std::string* detail) {
    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
        if (detail) {
            *detail = "command failed: " + cmd + " rc=" + std::to_string(rc);
        }
        return false;
    }
    return true;
}

std::string ReadFile(const fs::path& path) {
    std::ifstream in(path);
    if (!in) {
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string JsonGetString(const std::string& json, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const auto pos = json.find(needle);
    if (pos == std::string::npos) {
        return {};
    }
    const auto colon = json.find(':', pos);
    if (colon == std::string::npos) {
        return {};
    }
    const auto q1 = json.find('"', colon + 1);
    if (q1 == std::string::npos) {
        return {};
    }
    const auto q2 = json.find('"', q1 + 1);
    if (q2 == std::string::npos) {
        return {};
    }
    return json.substr(q1 + 1, q2 - q1 - 1);
}

OtaOptions LoadOtaOptionsFromConf() {
    OtaOptions opts;
    ::autonomy::system::proto::OtaOptions pb;
    if (!common::LoadModuleConf("system", "ota.pb.txt", &pb)) {
        AWARN << "OTA config not loaded (ota.pb.txt) — using defaults";
        return opts;
    }
    if (!pb.slots_root().empty()) {
        opts.slots_root = pb.slots_root();
    }
    if (!pb.current_link().empty()) {
        opts.current_link = pb.current_link();
    }
    if (!pb.staging_dir().empty()) {
        opts.staging_dir = pb.staging_dir();
    }
    if (!pb.state_path().empty()) {
        opts.state_path = pb.state_path();
    }
    if (pb.health_gate_sec() > 0) {
        opts.health_gate_sec = pb.health_gate_sec();
    }
    opts.delta_fallback_to_full = pb.delta_fallback_to_full();
    opts.allow_start_when_hazard = pb.allow_start_when_hazard();
    return opts;
}

}  // namespace

::automsgs::rpcs::system::OtaState ToProto(State s) {
    using ::automsgs::rpcs::system::OtaState;
    switch (s) {
        case State::kIdle:
            return OtaState::OTA_STATE_IDLE;
        case State::kDownloading:
            return OtaState::OTA_STATE_DOWNLOADING;
        case State::kVerifying:
            return OtaState::OTA_STATE_VERIFYING;
        case State::kApplying:
            return OtaState::OTA_STATE_APPLYING;
        case State::kSwitching:
            return OtaState::OTA_STATE_SWITCHING;
        case State::kRestarting:
            return OtaState::OTA_STATE_RESTARTING;
        case State::kHealthGating:
            return OtaState::OTA_STATE_HEALTH_GATING;
        case State::kCommitted:
            return OtaState::OTA_STATE_COMMITTED;
        case State::kRollingBack:
            return OtaState::OTA_STATE_ROLLING_BACK;
        case State::kFailed:
            return OtaState::OTA_STATE_FAILED;
        default:
            return OtaState::OTA_STATE_UNKNOWN;
    }
}

OtaAgent& OtaAgent::Shared() {
    static OtaAgent agent(LoadOtaOptionsFromConf());
    return agent;
}

OtaAgent::OtaAgent(OtaOptions options) : options_(std::move(options)) {
    LoadState();
    current_version_ = ReadVersion(ActiveSlot());
}

void OtaAgent::LoadState() {
    const auto raw = ReadFile(options_.state_path);
    if (raw.empty()) {
        return;
    }
    const auto st = JsonGetString(raw, "state");
    detail_ = JsonGetString(raw, "detail");
    target_version_ = JsonGetString(raw, "target_version");
    package_path_ = JsonGetString(raw, "package_path");
    package_type_ = JsonGetString(raw, "package_type");
    if (st == "failed") {
        state_ = State::kFailed;
    } else if (st == "committed") {
        state_ = State::kCommitted;
    } else if (!st.empty() && st != "idle") {
        // Interrupted mid-flight → mark failed for operator visibility.
        state_ = State::kFailed;
        detail_ = "interrupted; was=" + st;
    }
}

void OtaAgent::PersistState() const {
    try {
        fs::create_directories(fs::path(options_.state_path).parent_path());
    } catch (...) {
    }
    std::ofstream out(options_.state_path, std::ios::trunc);
    out << "{\n"
        << "  \"state\": \"";
    switch (state_) {
        case State::kIdle:
            out << "idle";
            break;
        case State::kDownloading:
            out << "downloading";
            break;
        case State::kVerifying:
            out << "verifying";
            break;
        case State::kApplying:
            out << "applying";
            break;
        case State::kSwitching:
            out << "switching";
            break;
        case State::kRestarting:
            out << "restarting";
            break;
        case State::kHealthGating:
            out << "health_gating";
            break;
        case State::kCommitted:
            out << "committed";
            break;
        case State::kRollingBack:
            out << "rolling_back";
            break;
        case State::kFailed:
            out << "failed";
            break;
    }
    out << "\",\n"
        << "  \"detail\": \"" << detail_ << "\",\n"
        << "  \"target_version\": \"" << target_version_ << "\",\n"
        << "  \"package_path\": \"" << package_path_ << "\",\n"
        << "  \"package_type\": \"" << package_type_ << "\"\n"
        << "}\n";
}

std::string OtaAgent::ActiveSlot() const {
    std::error_code ec;
    if (fs::is_symlink(options_.current_link, ec)) {
        return fs::read_symlink(options_.current_link, ec).string();
    }
    return options_.slots_root + "/slot_a";
}

std::string OtaAgent::InactiveSlot() const {
    const auto active = ActiveSlot();
    const std::string a = options_.slots_root + "/slot_a";
    const std::string b = options_.slots_root + "/slot_b";
    return (active.find("slot_a") != std::string::npos) ? b : a;
}

std::string OtaAgent::ReadVersion(const std::string& slot) const {
    const auto manifest = fs::path(slot) / "share/autonomy/MANIFEST.json";
    const auto raw = ReadFile(manifest);
    const auto v = JsonGetString(raw, "git_describe");
    return v.empty() ? "unknown" : v;
}

bool OtaAgent::Precheck(std::string* detail) const {
    if (!options_.allow_start_when_hazard) {
        if (const auto health =
                monitor::HealthSnapshotStore{}.Read()) {
            if (health->hazard_level() ==
                    ::automsgs::rpcs::system::HAZARD_LEVEL_ERROR ||
                health->emergency_stop_latched() ||
                safety::SafetyLatch{}.IsLatched()) {
                if (detail) {
                    *detail = "refusing OTA while hazard/estop active";
                }
                return false;
            }
        }
    }
    try {
        fs::create_directories(options_.slots_root);
        fs::create_directories(options_.staging_dir);
        fs::create_directories(options_.slots_root + "/slot_a");
        fs::create_directories(options_.slots_root + "/slot_b");
    } catch (const std::exception& ex) {
        if (detail) {
            *detail = ex.what();
        }
        return false;
    }
    return true;
}

bool OtaAgent::DownloadToStaging(const std::string& uri, std::string* detail) {
    progress_ = 0.1f;
    // Local path: copy tree/archive into staging.
    const fs::path src(uri);
    if (!fs::exists(src)) {
        if (detail) {
            *detail = "package_uri not found (http download not implemented; use local path)";
        }
        return false;
    }
    const fs::path dst = fs::path(options_.staging_dir) / "package";
    std::error_code ec;
    fs::remove_all(dst, ec);
    if (fs::is_directory(src)) {
        fs::copy(src, dst, fs::copy_options::recursive, ec);
    } else {
        fs::create_directories(dst);
        fs::copy_file(src, dst / src.filename(), ec);
    }
    if (ec) {
        if (detail) {
            *detail = ec.message();
        }
        return false;
    }
    package_path_ = dst.string();
    progress_ = 0.3f;
    return true;
}

bool OtaAgent::VerifyPackage(std::string* detail) {
    progress_ = 0.4f;
    const fs::path pkg(package_path_);
    const auto manifest = pkg / "package_manifest.json";
    if (fs::exists(manifest)) {
        const auto raw = ReadFile(manifest);
        package_type_ = JsonGetString(raw, "type");
        target_version_ = JsonGetString(raw, "target_version");
        const auto base = JsonGetString(raw, "base_version");
        if (package_type_ == "delta") {
            current_version_ = ReadVersion(ActiveSlot());
            if (!base.empty() && base != current_version_) {
                if (options_.delta_fallback_to_full &&
                    fs::exists(pkg / "full.tar.gz")) {
                    package_type_ = "full";
                    AWARN << "OTA: delta base mismatch; falling back to full";
                } else {
                    if (detail) {
                        *detail = "delta base_version mismatch: need " + base +
                                  " have " + current_version_;
                    }
                    return false;
                }
            }
        }
    } else if (fs::exists(pkg / "full.tar.gz") ||
               fs::exists(pkg / "autonomy.tar.gz")) {
        package_type_ = "full";
    } else {
        // Single .tar.gz copied into staging/package/
        for (const auto& ent : fs::directory_iterator(pkg)) {
            if (ent.path().extension() == ".gz" ||
                ent.path().string().find(".tar") != std::string::npos) {
                package_type_ = "full";
                break;
            }
        }
    }
    if (package_type_.empty()) {
        if (detail) {
            *detail = "cannot determine package type";
        }
        return false;
    }
    progress_ = 0.5f;
    return true;
}

bool OtaAgent::ApplyFull(const fs::path& pkg, std::string* detail) {
    fs::path tar;
    if (fs::exists(pkg / "full.tar.gz")) {
        tar = pkg / "full.tar.gz";
    } else if (fs::exists(pkg / "autonomy.tar.gz")) {
        tar = pkg / "autonomy.tar.gz";
    } else {
        for (const auto& ent : fs::directory_iterator(pkg)) {
            if (ent.path().extension() == ".gz") {
                tar = ent.path();
                break;
            }
        }
    }
    if (tar.empty()) {
        if (detail) {
            *detail = "full.tar.gz missing";
        }
        return false;
    }
    const auto inactive = InactiveSlot();
    std::error_code ec;
    fs::remove_all(inactive, ec);
    fs::create_directories(inactive);
    const std::string cmd =
        "tar -xzf " + tar.string() + " -C " + inactive + " --strip-components=1";
    return RunCmd(cmd, detail);
}

bool OtaAgent::ApplyDelta(const fs::path& pkg, std::string* detail) {
    const auto index = pkg / "delta/file_index.json";
    if (!fs::exists(index)) {
        if (detail) {
            *detail = "delta/file_index.json missing";
        }
        return false;
    }
    const auto active = ActiveSlot();
    const auto inactive = InactiveSlot();
    std::error_code ec;
    fs::remove_all(inactive, ec);
    // Seed inactive from active (file-level delta base).
    fs::copy(active, inactive, fs::copy_options::recursive, ec);
    if (ec) {
        if (detail) {
            *detail = "seed inactive from active failed: " + ec.message();
        }
        return false;
    }
    // Apply payloads listed as simple newline file of "op path" — minimal parser.
    // Expected file_index.json lines approx: {"op":"replace","path":"lib/x.so"}
    // For robustness, copy entire delta/payloads overlay if present.
    const auto payloads = pkg / "delta/payloads";
    if (fs::exists(payloads)) {
        fs::copy(payloads, inactive, fs::copy_options::recursive |
                                         fs::copy_options::overwrite_existing,
                 ec);
        if (ec) {
            if (detail) {
                *detail = "overlay payloads failed: " + ec.message();
            }
            return false;
        }
    }
    // Deletes: optional deletes.txt with one relative path per line.
    const auto deletes = pkg / "delta/deletes.txt";
    if (fs::exists(deletes)) {
        std::ifstream in(deletes);
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) {
                continue;
            }
            fs::remove_all(fs::path(inactive) / line, ec);
        }
    }
    return true;
}

bool OtaAgent::ApplyPackage(std::string* detail) {
    progress_ = 0.6f;
    const fs::path pkg(package_path_);
    if (package_type_ == "delta") {
        return ApplyDelta(pkg, detail);
    }
    return ApplyFull(pkg, detail);
}

bool OtaAgent::SwitchSlot(std::string* detail) {
    progress_ = 0.8f;
    const auto inactive = InactiveSlot();
    std::error_code ec;
    fs::remove(options_.current_link, ec);
    fs::create_directory_symlink(inactive, options_.current_link, ec);
    if (ec) {
        // Fallback: replace as directory copy pointer via rename dance
        if (detail) {
            *detail = "symlink switch failed: " + ec.message();
        }
        return false;
    }
    current_version_ = ReadVersion(inactive);
    progress_ = 0.9f;
    return true;
}

::automsgs::rpcs::system::StartOtaResponse OtaAgent::Start(
    const ::automsgs::rpcs::system::StartOtaRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    ::automsgs::rpcs::system::StartOtaResponse resp;
    if (state_ != State::kIdle && state_ != State::kFailed &&
        state_ != State::kCommitted) {
        resp.mutable_status()->set_code(
            ::automsgs::msgs::status_msgs::ABORTED);
        resp.mutable_status()->set_message("ota busy");
        resp.set_state(ToProto(state_));
        resp.set_detail(detail_);
        return resp;
    }
    abort_requested_ = false;
    detail_.clear();
    progress_ = 0.f;
    current_version_ = ReadVersion(ActiveSlot());

    auto fail = [&](const std::string& why) {
        state_ = State::kFailed;
        detail_ = why;
        PersistState();
        resp.mutable_status()->set_code(
            ::automsgs::msgs::status_msgs::INTERNAL);
        resp.mutable_status()->set_message(why);
        resp.set_state(ToProto(state_));
        resp.set_detail(why);
        return resp;
    };

    std::string why;
    state_ = State::kDownloading;
    PersistState();
    if (!Precheck(&why)) {
        return fail(why);
    }
    if (abort_requested_) {
        return fail("aborted");
    }
    if (!DownloadToStaging(request.package_uri(), &why)) {
        return fail(why);
    }
    if (abort_requested_) {
        return fail("aborted");
    }
    state_ = State::kVerifying;
    PersistState();
    if (!request.package_type().empty()) {
        package_type_ = request.package_type();
    }
    if (!VerifyPackage(&why)) {
        return fail(why);
    }
    state_ = State::kApplying;
    PersistState();
    if (!ApplyPackage(&why)) {
        return fail(why);
    }
    state_ = State::kSwitching;
    PersistState();
    if (!SwitchSlot(&why)) {
        return fail(why);
    }
    state_ = State::kRestarting;
    PersistState();
    // Soft restart hint for launch/systemd operators.
    try {
        fs::path hint = fs::path(options_.staging_dir).parent_path() /
                        "restart_required";
        std::ofstream(hint) << "ota_switch\n";
    } catch (...) {
    }
    state_ = State::kHealthGating;
    PersistState();
    // Best-effort gate: if snapshot exists and shows ERROR, rollback.
    bool gate_ok = true;
    if (const auto health = monitor::HealthSnapshotStore{}.Read()) {
        if (health->hazard_level() ==
            ::automsgs::rpcs::system::HAZARD_LEVEL_ERROR) {
            gate_ok = false;
            why = "health gate failed: hazard ERROR";
        }
    }
    if (!gate_ok) {
        state_ = State::kRollingBack;
        PersistState();
        // Roll back symlink to previous active (now inactive after switch).
        const auto other = InactiveSlot();
        std::error_code ec;
        fs::remove(options_.current_link, ec);
        fs::create_directory_symlink(other, options_.current_link, ec);
        return fail(why);
    }
    state_ = State::kCommitted;
    progress_ = 1.f;
    detail_ = "committed " + target_version_;
    PersistState();
    resp.mutable_status()->set_code(::automsgs::msgs::status_msgs::OK);
    resp.mutable_status()->set_message("ok");
    resp.set_state(ToProto(state_));
    resp.set_detail(detail_);
    return resp;
}

::automsgs::rpcs::system::GetOtaStatusResponse OtaAgent::Status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    ::automsgs::rpcs::system::GetOtaStatusResponse resp;
    resp.mutable_status()->set_code(::automsgs::msgs::status_msgs::OK);
    resp.set_state(ToProto(state_));
    resp.set_current_version(current_version_.empty()
                                 ? ReadVersion(ActiveSlot())
                                 : current_version_);
    resp.set_target_version(target_version_);
    resp.set_progress(progress_);
    resp.set_detail(detail_);
    return resp;
}

::automsgs::rpcs::system::AbortOtaResponse OtaAgent::Abort(
    const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    ::automsgs::rpcs::system::AbortOtaResponse resp;
    if (state_ == State::kDownloading || state_ == State::kVerifying) {
        abort_requested_ = true;
        state_ = State::kFailed;
        detail_ = reason.empty() ? "aborted" : reason;
        PersistState();
        resp.mutable_status()->set_code(::automsgs::msgs::status_msgs::OK);
        resp.mutable_status()->set_message("abort requested");
    } else {
        resp.mutable_status()->set_code(
            ::automsgs::msgs::status_msgs::ABORTED);
        resp.mutable_status()->set_message(
            "abort only allowed before apply");
    }
    resp.set_state(ToProto(state_));
    return resp;
}

}  // namespace ota
}  // namespace system
}  // namespace autonomy
