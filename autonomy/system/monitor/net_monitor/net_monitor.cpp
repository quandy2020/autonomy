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

#include "autonomy/system/monitor/net_monitor/net_monitor.hpp"

#include <fstream>
#include <sstream>
#include <string>

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/family.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

namespace {

bool ParseNetDev(std::map<std::string, NetInterfaceStats>* out) {
    if (out == nullptr)
        return false;
    out->clear();
    std::ifstream f("/proc/net/dev");
    if (!f.is_open())
        return false;
    std::string line;
    std::getline(f, line);
    std::getline(f, line);
    while (std::getline(f, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos)
            continue;
        std::string name = line.substr(0, colon);
        while (!name.empty() && name.front() == ' ')
            name.erase(name.begin());
        if (name == "lo")
            continue;
        std::istringstream iss(line.substr(colon + 1));
        NetInterfaceStats stats;
        stats.name = name;
        uint64_t rx_packets, rx_errs, rx_drop, rx_fifo, rx_frame, rx_compressed,
            rx_multicast;
        uint64_t tx_packets, tx_errs, tx_drop, tx_fifo, tx_colls, tx_carrier,
            tx_compressed;
        if (!(iss >> stats.rx_bytes >> rx_packets >> rx_errs >> rx_drop >>
              rx_fifo >> rx_frame >> rx_compressed >> rx_multicast >>
              stats.tx_bytes >> tx_packets >> tx_errs >> tx_drop >> tx_fifo >>
              tx_colls >> tx_carrier >> tx_compressed))
            continue;
        (*out)[name] = stats;
    }
    return !out->empty();
}

}  // namespace

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
struct NetMonitor::PrometheusGauges {
    prometheus::Family<prometheus::Gauge>* rx_family{nullptr};
    prometheus::Family<prometheus::Gauge>* tx_family{nullptr};
    std::map<std::string, prometheus::Gauge*> rx_rate;
    std::map<std::string, prometheus::Gauge*> tx_rate;
};
#endif

void NetMonitor::Collect() {
    std::map<std::string, NetInterfaceStats> curr;
    if (!ParseNetDev(&curr))
        return;

    const auto now = std::chrono::steady_clock::now();
    if (first_collect_) {
        for (const auto& [name, st] : curr)
            prev_bytes_[name] = {st.rx_bytes, st.tx_bytes};
        prev_time_ = now;
        interfaces_ = std::move(curr);
        first_collect_ = false;
        return;
    }

    const double dt = std::chrono::duration<double>(now - prev_time_).count();
    if (dt <= 0.0)
        return;

    for (auto& [name, st] : curr) {
        auto it = prev_bytes_.find(name);
        if (it != prev_bytes_.end()) {
            const auto& prev = it->second;
            st.rx_bytes_per_sec =
                static_cast<double>(st.rx_bytes - prev.first) / dt;
            st.tx_bytes_per_sec =
                static_cast<double>(st.tx_bytes - prev.second) / dt;
        }
        prev_bytes_[name] = {st.rx_bytes, st.tx_bytes};
    }
    prev_time_ = now;
    interfaces_ = std::move(curr);

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (!gauges_)
        return;
    for (const auto& [name, st] : interfaces_) {
        if (gauges_->rx_family) {
            auto& g = gauges_->rx_rate[name];
            if (g == nullptr)
                g = &gauges_->rx_family->Add({{"device", name}});
            g->Set(st.rx_bytes_per_sec);
        }
        if (gauges_->tx_family) {
            auto& g = gauges_->tx_rate[name];
            if (g == nullptr)
                g = &gauges_->tx_family->Add({{"device", name}});
            g->Set(st.tx_bytes_per_sec);
        }
    }
#endif
}

void NetMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    gauges_ = std::make_unique<PrometheusGauges>();
    gauges_->rx_family = &prometheus::BuildGauge()
                              .Name("autonomy_system_net_rx_bytes_per_second")
                              .Help("Network receive throughput bytes per second")
                              .Register(*reg);
    gauges_->tx_family = &prometheus::BuildGauge()
                              .Name("autonomy_system_net_tx_bytes_per_second")
                              .Help("Network transmit throughput bytes per second")
                              .Register(*reg);
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
