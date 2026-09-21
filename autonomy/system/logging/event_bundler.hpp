/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include "autonomy/system/monitor/system_health_snapshot.hpp"

namespace autonomy {
namespace system {
namespace logging {

struct EventBundleOptions {
    std::string log_root;       // empty → $HOME/.autonomy/log
    std::string events_subdir{"events"};
    int window_before_s{60};
    int window_after_s{0};
    int cooldown_s{60};
};

/**
 * Pack evidence under $log_root/events/<event_id>/ on Hazard/E-Stop triggers.
 */
class EventBundler {
public:
    explicit EventBundler(EventBundleOptions options = {});

    /// @return event_id or empty if skipped (cooldown / IO error)
    std::string MaybeBundle(const std::string& trigger,
                            const monitor::SystemHealthSnapshot& snap);

    std::string LatestEventPath() const;
    std::string ResolveEventPath(const std::string& event_id) const;

private:
    EventBundleOptions options_;
    std::string last_trigger_;
    int64_t last_bundle_ns_{0};
};

}  // namespace logging
}  // namespace system
}  // namespace autonomy
