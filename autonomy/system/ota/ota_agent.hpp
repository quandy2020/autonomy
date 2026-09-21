/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <atomic>
#include <filesystem>
#include <mutex>
#include <string>

#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace system {
namespace ota {

enum class State {
    kIdle = 0,
    kDownloading,
    kVerifying,
    kApplying,
    kSwitching,
    kRestarting,
    kHealthGating,
    kCommitted,
    kRollingBack,
    kFailed,
};

struct OtaOptions {
    std::string slots_root{"/opt/autonomy-slots"};
    std::string current_link{"/opt/autonomy"};
    std::string staging_dir{"/var/lib/autonomy/ota/staging"};
    std::string state_path{"/var/lib/autonomy/ota/state.json"};
    int health_gate_sec{45};
    bool delta_fallback_to_full{true};
    bool allow_start_when_hazard{false};
};

/**
 * Dual-slot OTA FSM (C++). Supports full tar.gz and file-level delta packages.
 */
class OtaAgent {
public:
    explicit OtaAgent(OtaOptions options = {});

    ::automsgs::rpcs::system::StartOtaResponse Start(
        const ::automsgs::rpcs::system::StartOtaRequest& request);
    ::automsgs::rpcs::system::GetOtaStatusResponse Status() const;
    ::automsgs::rpcs::system::AbortOtaResponse Abort(const std::string& reason);

    static OtaAgent& Shared();

private:
    bool Precheck(std::string* detail) const;
    bool DownloadToStaging(const std::string& uri, std::string* detail);
    bool VerifyPackage(std::string* detail);
    bool ApplyPackage(std::string* detail);
    bool ApplyFull(const std::filesystem::path& pkg, std::string* detail);
    bool ApplyDelta(const std::filesystem::path& pkg, std::string* detail);
    bool SwitchSlot(std::string* detail);
    void PersistState() const;
    void LoadState();
    std::string InactiveSlot() const;
    std::string ActiveSlot() const;
    std::string ReadVersion(const std::string& slot) const;

    OtaOptions options_;
    mutable std::mutex mutex_;
    State state_{State::kIdle};
    std::string detail_;
    std::string current_version_;
    std::string target_version_;
    std::string package_path_;
    std::string package_type_;
    float progress_{0.f};
    std::atomic<bool> abort_requested_{false};
};

::automsgs::rpcs::system::OtaState ToProto(State s);

}  // namespace ota
}  // namespace system
}  // namespace autonomy
