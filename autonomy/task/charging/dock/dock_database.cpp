/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/charging/dock/dock_database.hpp"

#include <cmath>
#include <limits>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/charging/dock/exceptions.hpp"
#include "autonomy/task/charging/dock/plugin_registry.hpp"
#include "autonomy/task/charging/dock/utils.hpp"

namespace autonomy {
namespace task {
namespace charging {

bool DockDatabase::LoadPlugins(
    const std::shared_ptr<autolink::Node>& node,
    const ::autonomy::task::proto::DockingOptions& options) {
    plugins_.clear();
    auto add = [&](const ::autonomy::task::proto::DockPluginSpec& spec) {
        const std::string type =
            spec.type().empty() ? "simple_charging_dock" : spec.type();
        const std::string name = spec.name().empty() ? type : spec.name();
        auto created = DockPlugins().CreateObjectOrNull(type);
        if (!created) {
            AERROR << "unknown charging dock plugin type=" << type;
            return false;
        }
        ChargingDock::Ptr plugin(created.release());
        if (!plugin->Configure(node, name, spec)) {
            AERROR << "dock plugin configure failed name=" << name;
            return false;
        }
        plugins_.emplace(name, std::move(plugin));
        AINFO << "dock plugin loaded name=" << name << " type=" << type;
        return true;
    };

    if (options.plugins().empty()) {
        ::autonomy::task::proto::DockPluginSpec spec;
        spec.set_name("simple_charging_dock");
        spec.set_type("simple_charging_dock");
        return add(spec);
    }
    for (const auto& spec : options.plugins()) {
        if (!add(spec)) {
            return false;
        }
    }
    return !plugins_.empty();
}

bool DockDatabase::Initialize(
    const std::shared_ptr<autolink::Node>& node,
    const ::autonomy::task::proto::DockingOptions& options) {
    node_ = node;
    if (!LoadPlugins(node, options)) {
        return false;
    }
    instances_.clear();
    if (!LoadDockInstances(options, &instances_)) {
        AWARN << "dock instance database empty; pose goals still work";
    }
    AINFO << "dock database plugins=" << plugin_size()
          << " instances=" << instance_size();
    return true;
}

void DockDatabase::Activate() {
    for (auto& kv : plugins_) {
        kv.second->Activate();
    }
}

void DockDatabase::Deactivate() {
    for (auto& kv : plugins_) {
        kv.second->Deactivate();
    }
}

bool DockDatabase::ReloadFile(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(mutex_);
    DockMap loaded;
    if (!ParseDockFile(filepath, &loaded)) {
        return false;
    }
    instances_ = std::move(loaded);
    return true;
}

Dock* DockDatabase::FindDock(const std::string& dock_id) {
    auto it = instances_.find(dock_id);
    if (it == instances_.end()) {
        throw DockNotInDB("Dock ID requested is not in database: " + dock_id);
    }
    auto plugin = FindDockPlugin(it->second.type);
    if (!plugin) {
        throw DockNotValid("Dock requested has no valid plugin: " + dock_id);
    }
    it->second.plugin = plugin;
    return &it->second;
}

std::string DockDatabase::SelectDockId(const std::string& preferred) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!preferred.empty()) {
        if (instances_.find(preferred) != instances_.end()) {
            return preferred;
        }
        for (const auto& kv : instances_) {
            if (kv.second.id == preferred) {
                return kv.first;
            }
        }
    }
    if (instances_.empty()) {
        return {};
    }
    return instances_.begin()->first;
}

std::string DockDatabase::NearestDockId(double x, double y) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string best;
    double best_d = std::numeric_limits<double>::infinity();
    for (const auto& kv : instances_) {
        const double dx = kv.second.pose.position().x() - x;
        const double dy = kv.second.pose.position().y() - y;
        const double d = std::hypot(dx, dy);
        if (d < best_d) {
            best_d = d;
            best = kv.first;
        }
    }
    return best;
}

ChargingDock::Ptr DockDatabase::FindDockPlugin(const std::string& type) {
    if (type.empty() && plugins_.size() == 1) {
        return plugins_.begin()->second;
    }
    auto it = plugins_.find(type);
    if (it != plugins_.end()) {
        return it->second;
    }
    return nullptr;
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy
