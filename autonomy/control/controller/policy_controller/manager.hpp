/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file manager.hpp
 * @brief Double-buffered PolicyModel hot-swap.
 */

#pragma once

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/control/controller/policy_controller/model.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @class PolicyManager
 * @brief Owns active / standby @ref PolicyModel slots for runtime reload.
 *
 * Control thread holds @c Active() shared_ptr for the whole tick; a successful
 * @c Load swaps the active pointer while the previous model may remain alive
 * until the tick releases its shared_ptr.
 */
class PolicyManager
{
public:
    PolicyManager() = default;
    ~PolicyManager();

    PolicyManager(const PolicyManager&) = delete;
    PolicyManager& operator=(const PolicyManager&) = delete;

    /**
     * @brief Synchronously create and activate a model from @p config.
     * @return false if Create / Activate fails.
     */
    bool Load(const PolicyConfig& config, std::string* error = nullptr);

    /**
     * @brief Load config from file then @ref Load.
     */
    bool LoadFromFile(const std::string& config_path,
                      std::string* error = nullptr);

    /**
     * @brief Background load; only one async load may be in flight.
     * @param done Callback @c (ok, error_message) on the loader thread.
     * @return false if another async load is already running.
     */
    bool LoadAsync(PolicyConfig config,
                   std::function<void(bool, const std::string&)> done);

    /**
     * @brief Current active model (may be null).
     */
    PolicyModel::ConstSharedPointer Active() const;
    /**
     * @brief True if an active model exists (no shared_ptr churn).
     */
    bool HasActive() const;

    /**
     * @brief Drop active / standby models (Cleanup / Deactivate).
     */
    void Clear();

private:
    bool Activate(PolicyModel::SharedPointer model, std::string* error);

    mutable std::mutex mutex_;
    PolicyModel::SharedPointer active_;
    PolicyModel::SharedPointer standby_;
    std::future<void> load_future_;
    std::atomic<bool> load_in_flight_{false};
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
