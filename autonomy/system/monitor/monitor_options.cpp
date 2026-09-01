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

#include "autonomy/system/monitor/monitor_options.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/system/monitor/ops_types.hpp"
#include "autonomy/task/teleop/constants.hpp"
#include "autonomy/task/common/names.hpp"

namespace autonomy {
namespace system {
namespace monitor {
namespace {

using ::autonomy::common::ConfigurationFileResolver;
using ::autonomy::common::ConfigurationSearchDirectories;
using ::autonomy::common::LuaParameterDictionary;

void ApplyBool(LuaParameterDictionary* dict, const char* key, bool* field) {
    if (dict->HasKey(key))
        *field = dict->GetBool(key);
}

void ApplyDouble(LuaParameterDictionary* dict, const char* key, double* field) {
    if (dict->HasKey(key))
        *field = dict->GetDouble(key);
}

void ApplyString(LuaParameterDictionary* dict, const char* key,
                 std::string* field) {
    if (dict->HasKey(key))
        *field = dict->GetString(key);
}

std::vector<ChannelWatchOptions> LoadChannelWatches(
    LuaParameterDictionary* dict) {
    std::vector<ChannelWatchOptions> out;
    if (!dict->HasKey("channel_watches"))
        return out;
    for (const auto& entry :
         dict->GetDictionary("channel_watches")
             ->GetArrayValuesAsDictionaries()) {
        ChannelWatchOptions w;
        if (entry->HasKey("channel"))
            w.channel = entry->GetString("channel");
        ApplyDouble(entry.get(), "timeout_sec", &w.timeout_sec);
        ApplyDouble(entry.get(), "min_rate_hz", &w.min_rate_hz);
        if (!w.channel.empty())
            out.push_back(std::move(w));
    }
    return out;
}

std::vector<LatencyWatchOptions> LoadLatencyWatches(
    LuaParameterDictionary* dict) {
    std::vector<LatencyWatchOptions> out;
    if (!dict->HasKey("latency_watches"))
        return out;
    for (const auto& entry :
         dict->GetDictionary("latency_watches")
             ->GetArrayValuesAsDictionaries()) {
        LatencyWatchOptions w;
        if (entry->HasKey("channel"))
            w.channel = entry->GetString("channel");
        ApplyDouble(entry.get(), "max_age_sec", &w.max_age_sec);
        if (!w.channel.empty())
            out.push_back(std::move(w));
    }
    return out;
}

void LoadMrmOptions(LuaParameterDictionary* dict, MrmHandlerOptions* mrm) {
    if (mrm == nullptr)
        return;
    if (!dict->HasKey("mrm"))
        return;
    auto m = dict->GetDictionary("mrm");
    ApplyString(m.get(), "cmd_vel_channel", &mrm->cmd_vel_channel);
    ApplyBool(m.get(), "emergency_stop_on_error",
              &mrm->emergency_stop_on_error);
}

void ApplyDefaultWatches(MonitorOptions* opts) {
    if (opts == nullptr)
        return;
    if (opts->channel_watches.empty()) {
        opts->channel_watches.push_back(
            {task::teleop::kCommandVelocityTopic, 1.0, 0.0});
        opts->channel_watches.push_back(
            {task::kTeleopGoal, 5.0, 0.0});
        opts->channel_watches.push_back(
            {task::kTeleopFeedback, 5.0, 0.0});
    }
    if (opts->latency_watches.empty()) {
        opts->latency_watches.push_back(
            {task::teleop::kCommandVelocityTopic, 0.5});
    }
}

MonitorOptions LoadFromDictionary(LuaParameterDictionary* dict) {
    MonitorOptions opts = MonitorOptions::Default();
    ApplyBool(dict, "enable_cpu_monitor", &opts.enable_cpu_monitor);
    ApplyBool(dict, "enable_gpu_monitor", &opts.enable_gpu_monitor);
    ApplyBool(dict, "enable_mem_monitor", &opts.enable_mem_monitor);
    ApplyBool(dict, "enable_hdd_monitor", &opts.enable_hdd_monitor);
    ApplyBool(dict, "enable_net_monitor", &opts.enable_net_monitor);
    ApplyBool(dict, "enable_ntp_monitor", &opts.enable_ntp_monitor);
    ApplyBool(dict, "enable_process_monitor", &opts.enable_process_monitor);
    ApplyBool(dict, "enable_voltage_monitor", &opts.enable_voltage_monitor);
    ApplyBool(dict, "enable_channel_monitor", &opts.enable_channel_monitor);
    ApplyBool(dict, "enable_latency_monitor", &opts.enable_latency_monitor);
    ApplyBool(dict, "enable_hazard_monitor", &opts.enable_hazard_monitor);
    ApplyBool(dict, "enable_mrm_handler", &opts.enable_mrm_handler);
    ApplyBool(dict, "enable_prometheus", &opts.enable_prometheus);
    ApplyString(dict, "prometheus_bind_address", &opts.prometheus_bind_address);
    ApplyString(dict, "prometheus_metrics_prefix",
                &opts.prometheus_metrics_prefix);
    ApplyDouble(dict, "collect_interval_sec", &opts.collect_interval_sec);
    ApplyBool(dict, "enable_cpu_profile", &opts.enable_cpu_profile);
    ApplyString(dict, "cpu_profile_filename", &opts.cpu_profile_filename);
    ApplyBool(dict, "enable_heap_profile", &opts.enable_heap_profile);
    ApplyString(dict, "heap_profile_filename", &opts.heap_profile_filename);
    opts.channel_watches = LoadChannelWatches(dict);
    opts.latency_watches = LoadLatencyWatches(dict);
    LoadMrmOptions(dict, &opts.mrm);
    ApplyDefaultWatches(&opts);
    return opts;
}

}  // namespace

MonitorOptions MonitorOptions::Default() {
    MonitorOptions opts;
    opts.enable_cpu_monitor = true;
    opts.enable_mem_monitor = true;
    opts.enable_prometheus = true;
    opts.prometheus_bind_address = "0.0.0.0:9090";
    opts.prometheus_metrics_prefix = "autonomy_system";
    opts.collect_interval_sec = 1.0;
    ApplyDefaultWatches(&opts);
    return opts;
}

MonitorOptions LoadMonitorOptions(const std::string& configuration_directory,
                                  const std::string& configuration_basename) {
    try {
        const auto dirs =
            ConfigurationSearchDirectories(configuration_directory);
        auto file_resolver = std::make_unique<ConfigurationFileResolver>(dirs);
        const std::string code =
            file_resolver->GetFileContentOrDie(configuration_basename);
        LuaParameterDictionary dict(code, std::move(file_resolver));
        if (dict.HasKey("monitor")) {
            return LoadFromDictionary(dict.GetDictionary("monitor").get());
        }
        return LoadFromDictionary(&dict);
    } catch (const std::exception& ex) {
        AWARN << "Monitor config not loaded (" << configuration_basename
              << "): " << ex.what() << " — using defaults";
        return MonitorOptions::Default();
    }
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
