/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/manager.hpp"

#include <utility>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

PolicyManager::~PolicyManager() {
    if (load_future_.valid()) {
        load_future_.wait();
    }
}

bool PolicyManager::Activate(PolicyModel::SharedPointer model,
                             std::string* error) {
    if (!model) {
        if (error != nullptr) {
            *error = "Null PolicyModel";
        }
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    standby_ = std::move(active_);
    active_ = std::move(model);
    return true;
}

bool PolicyManager::Load(const PolicyConfig& config,
                         std::string* error) {
    auto model = PolicyModel::Create(config, error);
    if (!model) {
        return false;
    }
    return Activate(std::move(model), error);
}

bool PolicyManager::LoadFromFile(const std::string& config_path,
                                 std::string* error) {
    PolicyConfig config;
    if (!PolicyConfig::LoadFromFile(config_path, &config,
                                           error)) {
        return false;
    }
    return Load(config, error);
}

bool PolicyManager::LoadAsync(
    PolicyConfig config,
    std::function<void(bool, const std::string&)> done) {
    bool expected = false;
    if (!load_in_flight_.compare_exchange_strong(expected, true)) {
        if (done) {
            done(false, "PolicyManager load already in flight");
        }
        return false;
    }
    if (load_future_.valid()) {
        load_future_.wait();
    }
    load_future_ = std::async(
        std::launch::async,
        [this, config = std::move(config),
         done = std::move(done)]() mutable {
            std::string error;
            const bool ok = Load(config, &error);
            load_in_flight_.store(false);
            if (done) {
                done(ok, error);
            }
        });
    return true;
}

PolicyModel::ConstSharedPointer PolicyManager::Active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
}

bool PolicyManager::HasActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<bool>(active_);
}

void PolicyManager::Clear() {
    if (load_future_.valid()) {
        load_future_.wait();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    active_.reset();
    standby_.reset();
    load_in_flight_.store(false);
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
