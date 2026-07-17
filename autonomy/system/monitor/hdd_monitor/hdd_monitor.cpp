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

#include "autonomy/system/monitor/hdd_monitor/hdd_monitor.hpp"

#include <sys/statvfs.h>

#include <fstream>
#include <set>
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

bool IsDataFilesystem(const std::string& fstype) {
    static const char* kTypes[] = {"ext4", "ext3", "xfs", "btrfs", "f2fs",
                                   "vfat", "exfat", "ntfs", "overlay"};
    for (const char* t : kTypes) {
        if (fstype == t)
            return true;
    }
    return false;
}

bool QueryStatvfs(const std::string& mount_point, MountDiskUsage* out) {
    if (out == nullptr)
        return false;
    struct statvfs st {};
    if (statvfs(mount_point.c_str(), &st) != 0)
        return false;
    const uint64_t total = static_cast<uint64_t>(st.f_blocks) * st.f_frsize;
    const uint64_t avail = static_cast<uint64_t>(st.f_bavail) * st.f_frsize;
    out->mount_point = mount_point;
    out->total_bytes = total;
    out->available_bytes = avail;
    if (total > 0 && total >= avail)
        out->usage_percent =
            100.0 * static_cast<double>(total - avail) / static_cast<double>(total);
    else
        out->usage_percent = 0.0;
    return total > 0;
}

}  // namespace

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
struct HddMonitor::PrometheusGauges {
    prometheus::Family<prometheus::Gauge>* usage_family{nullptr};
    std::vector<prometheus::Gauge*> usage_percent;
};
#endif

void HddMonitor::RefreshMountList() {
    if (mounts_discovered_)
        return;

    std::set<std::string> seen;
    std::ifstream f("/proc/mounts");
    if (!f.is_open()) {
        MountDiskUsage root;
        if (QueryStatvfs("/", &root))
            mounts_.push_back(root);
        mounts_discovered_ = true;
        return;
    }

    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line);
        std::string device, mount_point, fstype;
        if (!(iss >> device >> mount_point >> fstype))
            continue;
        if (!IsDataFilesystem(fstype))
            continue;
        if (mount_point.find("/proc") == 0 || mount_point.find("/sys") == 0 ||
            mount_point.find("/dev") == 0)
            continue;
        if (!seen.insert(mount_point).second)
            continue;
        MountDiskUsage usage;
        if (QueryStatvfs(mount_point, &usage))
            mounts_.push_back(usage);
    }

    if (mounts_.empty()) {
        MountDiskUsage root;
        if (QueryStatvfs("/", &root))
            mounts_.push_back(root);
    }
    mounts_discovered_ = true;
}

void HddMonitor::Collect() {
    RefreshMountList();
    for (auto& m : mounts_)
        QueryStatvfs(m.mount_point, &m);

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (!gauges_ || !gauges_->usage_family)
        return;
    for (size_t i = 0; i < mounts_.size(); ++i) {
        while (gauges_->usage_percent.size() <= i) {
            const std::string mp = mounts_[gauges_->usage_percent.size()].mount_point;
            gauges_->usage_percent.push_back(
                &gauges_->usage_family->Add({{"mount", mp}}));
        }
        if (gauges_->usage_percent[i])
            gauges_->usage_percent[i]->Set(mounts_[i].usage_percent);
    }
#endif
}

void HddMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    gauges_ = std::make_unique<PrometheusGauges>();
    gauges_->usage_family = &prometheus::BuildGauge()
                                 .Name("autonomy_system_disk_usage_percent")
                                 .Help("Disk space usage percent per mount point")
                                 .Register(*reg);
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
