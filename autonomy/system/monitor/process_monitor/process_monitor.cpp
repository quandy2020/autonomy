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

#include "autonomy/system/monitor/process_monitor/process_monitor.hpp"

#include <dirent.h>

#include <cctype>
#include <fstream>
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

bool IsNumericPid(const char* name) {
    if (name == nullptr || *name == '\0')
        return false;
    for (const char* p = name; *p != '\0'; ++p) {
        if (!std::isdigit(static_cast<unsigned char>(*p)))
            return false;
    }
    return true;
}

bool ReadProcessRssKb(const std::string& pid, uint64_t* rss_kb, std::string* comm) {
    if (rss_kb == nullptr)
        return false;
    std::ifstream status("/proc/" + pid + "/status");
    if (!status.is_open())
        return false;
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream iss(line.substr(6));
            iss >> *rss_kb;
        } else if (comm != nullptr && line.rfind("Name:", 0) == 0) {
            *comm = line.substr(5);
            while (!comm->empty() && (comm->front() == ' ' || comm->front() == '\t'))
                comm->erase(comm->begin());
        }
    }
    return *rss_kb > 0;
}

}  // namespace

void ProcessMonitor::Collect() {
    process_count_ = 0;
    total_rss_kb_ = 0;
    top_rss_comm_.clear();
    uint64_t top_rss = 0;

    DIR* dir = opendir("/proc");
    if (dir == nullptr)
        return;

    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (!IsNumericPid(ent->d_name))
            continue;
        ++process_count_;
        uint64_t rss = 0;
        std::string comm;
        if (ReadProcessRssKb(ent->d_name, &rss, &comm)) {
            total_rss_kb_ += rss;
            if (rss > top_rss) {
                top_rss = rss;
                top_rss_comm_ = comm;
            }
        }
    }
    closedir(dir);

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (count_gauge_)
        static_cast<prometheus::Gauge*>(count_gauge_)
            ->Set(static_cast<double>(process_count_));
    if (rss_gauge_)
        static_cast<prometheus::Gauge*>(rss_gauge_)
            ->Set(static_cast<double>(total_rss_kb_));
#endif
}

void ProcessMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    auto& count_family = prometheus::BuildGauge()
                             .Name("autonomy_system_process_count")
                             .Help("Number of running processes")
                             .Register(*reg);
    count_gauge_ = &count_family.Add({});
    auto& rss_family = prometheus::BuildGauge()
                           .Name("autonomy_system_process_rss_total_kb")
                           .Help("Sum of VmRSS across processes in KB")
                           .Register(*reg);
    rss_gauge_ = &rss_family.Add({});
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
