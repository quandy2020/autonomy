/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/system/monitor/ntp_monitor/ntp_monitor.hpp"

#include <array>
#include <cstdio>
#include <regex>
#include <sstream>
#include <string>

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

namespace {

bool ParseChronycTracking(const std::string& output, double* offset_seconds) {
    if (offset_seconds == nullptr)
        return false;
    const std::regex line_re(R"(^(.+[A-Za-z()]) *: (.*))");
    const std::regex system_time_re(R"(([0-9.]+) seconds (slow|fast).*)");
    std::istringstream iss(output);
    std::string line;
    std::string system_time_value;
    while (std::getline(iss, line)) {
        std::smatch match;
        if (!std::regex_match(line, match, line_re))
            continue;
        if (match[1].str() == "System time")
            system_time_value = match[2].str();
    }
    if (system_time_value.empty())
        return false;
    std::smatch st_match;
    if (!std::regex_match(system_time_value, st_match, system_time_re))
        return false;
    double offset = std::stod(st_match[1].str());
    if (st_match[2].str() == "fast")
        offset = -offset;
    *offset_seconds = offset;
    return true;
}

std::string RunChronycTracking() {
    std::array<char, 256> buf{};
    std::string result;
    FILE* pipe = popen("chronyc tracking 2>/dev/null", "r");
    if (pipe == nullptr)
        return result;
    while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr)
        result += buf.data();
    pclose(pipe);
    return result;
}

}  // namespace

void NtpMonitor::Collect() {
    const std::string out = RunChronycTracking();
    chronyc_available_ = !out.empty();
    if (!chronyc_available_) {
        offset_seconds_ = std::nan("");
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
        if (offset_gauge_)
            static_cast<prometheus::Gauge*>(offset_gauge_)->Set(0.0);
#endif
        return;
    }
    double offset = std::nan("");
    if (ParseChronycTracking(out, &offset))
        offset_seconds_ = offset;
    else
        offset_seconds_ = std::nan("");

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (offset_gauge_ && std::isfinite(offset_seconds_))
        static_cast<prometheus::Gauge*>(offset_gauge_)->Set(offset_seconds_);
#endif
}

void NtpMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    auto& family = prometheus::BuildGauge()
                       .Name("autonomy_system_ntp_offset_seconds")
                       .Help("NTP offset from chronyc tracking (seconds)")
                       .Register(*reg);
    offset_gauge_ = &family.Add({});
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
