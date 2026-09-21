/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/logging/event_bundler.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "autolink/common/log.hpp"
#include "autonomy/system/monitor/health_snapshot_store.hpp"

namespace autonomy {
namespace system {
namespace logging {
namespace {

namespace fs = std::filesystem;

std::string DefaultLogRoot() {
    if (const char* env = std::getenv("AUTONOMY_LOG_ROOT");
        env != nullptr && env[0] != '\0') {
        return env;
    }
    if (const char* home = std::getenv("HOME");
        home != nullptr && home[0] != '\0') {
        return std::string(home) + "/.autonomy/log";
    }
    return "/tmp/autonomy/log";
}

int64_t NowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string MakeEventId(const std::string& trigger) {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    gmtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%dT%H%M%SZ") << "_" << trigger;
    return oss.str();
}

}  // namespace

EventBundler::EventBundler(EventBundleOptions options)
    : options_(std::move(options)) {
    if (options_.log_root.empty()) {
        options_.log_root = DefaultLogRoot();
    }
}

std::string EventBundler::ResolveEventPath(const std::string& event_id) const {
    return options_.log_root + "/" + options_.events_subdir + "/" + event_id;
}

std::string EventBundler::LatestEventPath() const {
    const fs::path root =
        fs::path(options_.log_root) / options_.events_subdir;
    if (!fs::exists(root)) {
        return {};
    }
    fs::path latest;
    fs::file_time_type best{};
    bool any = false;
    for (const auto& ent : fs::directory_iterator(root)) {
        if (!ent.is_directory()) {
            continue;
        }
        const auto wt = ent.last_write_time();
        if (!any || wt > best) {
            best = wt;
            latest = ent.path();
            any = true;
        }
    }
    return any ? latest.string() : std::string{};
}

std::string EventBundler::MaybeBundle(
    const std::string& trigger,
    const monitor::SystemHealthSnapshot& snap) {
    const int64_t now = NowNs();
    const int64_t cooldown_ns =
        static_cast<int64_t>(options_.cooldown_s) * 1000000000LL;
    if (trigger == last_trigger_ && (now - last_bundle_ns_) < cooldown_ns) {
        return {};
    }

    const std::string event_id = MakeEventId(trigger);
    const fs::path dir = ResolveEventPath(event_id);
    try {
        fs::create_directories(dir);
        fs::create_directories(dir / "logs");
    } catch (const std::exception& ex) {
        AWARN << "EventBundler: mkdir failed: " << ex.what();
        return {};
    }

    {
        std::ofstream meta(dir / "meta.json");
        meta << "{\n"
             << "  \"event_id\": \"" << event_id << "\",\n"
             << "  \"trigger\": \"" << trigger << "\",\n"
             << "  \"detail\": \"" << snap.detail << "\",\n"
             << "  \"hazard\": " << static_cast<int>(snap.hazard_level) << ",\n"
             << "  \"mrm_active\": " << (snap.mrm_active ? "true" : "false")
             << ",\n"
             << "  \"emergency_stop_latched\": "
             << (snap.emergency_stop_latched ? "true" : "false") << "\n"
             << "}\n";
    }

    monitor::HealthSnapshotStore store;
    const auto health = store.ToProto(snap);
    std::ofstream snap_out(dir / "snapshot.pb", std::ios::binary);
    const std::string bytes = health.SerializeAsString();
    snap_out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));

    {
        std::ofstream conf(dir / "conf_hash.txt");
        conf << "monitor.pb.txt\n";
        conf << "window_before_s=" << options_.window_before_s << "\n";
    }

    // Best-effort: copy recent log files from GLOG_log_dir if set.
    if (const char* glog = std::getenv("GLOG_log_dir");
        glog != nullptr && glog[0] != '\0') {
        try {
            for (const auto& ent : fs::directory_iterator(glog)) {
                if (!ent.is_regular_file()) {
                    continue;
                }
                const auto name = ent.path().filename().string();
                if (name.find(".log.") == std::string::npos) {
                    continue;
                }
                std::error_code ec;
                fs::copy_file(ent.path(), dir / "logs" / name,
                              fs::copy_options::overwrite_existing, ec);
            }
        } catch (...) {
        }
    }

    last_trigger_ = trigger;
    last_bundle_ns_ = now;
    AWARN << "EventBundler: wrote " << dir.string();
    return event_id;
}

}  // namespace logging
}  // namespace system
}  // namespace autonomy
