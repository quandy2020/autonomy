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

#include "autonomy/system/monitor/gpu_monitor/gpu_monitor.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/family.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace fs = std::filesystem;

namespace autonomy {
namespace system {
namespace monitor {

namespace {

bool ReadPercentFile(const fs::path& path, double* percent) {
    if (percent == nullptr)
        return false;
    std::ifstream f(path);
    if (!f.is_open())
        return false;
    double v = 0.0;
    if (!(f >> v))
        return false;
    *percent = v;
    return true;
}

void DiscoverGpuBusy(std::vector<GpuBusyReading>* out) {
    if (out == nullptr)
        return;
    out->clear();
    const fs::path drm_root("/sys/class/drm");
    if (!fs::exists(drm_root))
        return;

    for (const auto& entry : fs::directory_iterator(drm_root)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("card", 0) != 0)
            continue;
        if (name.find('-') != std::string::npos)
            continue;
        const fs::path busy_path = entry.path() / "device" / "gpu_busy_percent";
        double busy = 0.0;
        if (!ReadPercentFile(busy_path, &busy))
            continue;
        GpuBusyReading reading;
        reading.card = name;
        reading.busy_percent = busy;
        out->push_back(reading);
    }
}

}  // namespace

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
struct GpuMonitor::PrometheusGauges {
    prometheus::Family<prometheus::Gauge>* family{nullptr};
    std::vector<prometheus::Gauge*> busy;
};
#endif

void GpuMonitor::Collect() {
    DiscoverGpuBusy(&gpus_);

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (!gauges_ || !gauges_->family)
        return;
    for (size_t i = 0; i < gpus_.size(); ++i) {
        while (gauges_->busy.size() <= i) {
            const std::string card = gpus_[gauges_->busy.size()].card;
            gauges_->busy.push_back(
                &gauges_->family->Add({{"card", card}}));
        }
        if (gauges_->busy[i])
            gauges_->busy[i]->Set(gpus_[i].busy_percent);
    }
#endif
}

void GpuMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    gauges_ = std::make_unique<PrometheusGauges>();
    gauges_->family = &prometheus::BuildGauge()
                           .Name("autonomy_system_gpu_busy_percent")
                           .Help("GPU busy percent from DRM sysfs when available")
                           .Register(*reg);
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
