/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Template base class that wires SensorDriver capture to SensorModule hooks.
 */

#ifndef AUTODRIVER_SENSOR_PLUGIN_HPP_
#define AUTODRIVER_SENSOR_PLUGIN_HPP_

#include <memory>
#include <utility>

#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_module.hpp"
#include "autodriver/sensor_traits.hpp"

namespace autodriver {

/**
 * @class autodriver::SensorPlugin
 * @brief Connects a hardware driver to the module lifecycle and sample hook.
 * @tparam kType Sensor modality handled by this plugin.
 * @tparam kCapture When false, the plugin is attach-only (no hardware capture).
 */
template <SensorType kType, bool kCapture = true>
class SensorPlugin : public SensorModule {
public:
    // Compile-time traits for the sensor modality.
    using Traits = SensorTraits<kType>;

    // Concrete typed sample for this sensor modality.
    using Sample = typename Traits::Sample;

    /**
     * @brief Sensor modality implemented by this plugin.
     * @return The template parameter kType.
     */
    SensorType GetType() const final { return kType; }

    /**
     * @brief Stable instance id from configuration.
     * @return Configured sensor identifier.
     */
    const SensorId& GetSensorId() const final { return id_; }

    /**
     * @brief Bind configuration, create the driver, and register the sample callback.
     * @param context Sensor entry from YAML and upstream sample hook.
     * @return False when driver creation fails.
     */
    bool Init(const Context& context) final {
        id_ = context.sensor.id;
        hook_ = context.hook;
        if constexpr (kCapture) {
            driver_ = MakeDriver(context.sensor);
            if (!driver_) {
                return false;
            }
            driver_->SetSampleCallback(
                [this](std::unique_ptr<SensorSample> sample) {
                    OnSample(std::move(sample));
                });
        }
        return true;
    }

    /**
     * @brief Start hardware capture or mark the attach-only module as active.
     * @return True when capture starts successfully or attach-only mode is enabled.
     */
    bool Start() final {
        if constexpr (kCapture) {
            return driver_ != nullptr && driver_->Start();
        }
        running_ = true;
        return true;
    }

    /**
     * @brief Stop capture and release hardware resources.
     */
    void Stop() final {
        if constexpr (kCapture) {
            if (driver_) {
                driver_->Stop();
            }
        } else {
            running_ = false;
        }
    }

    /**
     * @brief Whether the module is currently capturing.
     * @return True when the driver is running or attach-only mode is active.
     */
    bool IsRunning() const final {
        if constexpr (kCapture) {
            return driver_ != nullptr && driver_->IsRunning();
        }
        return running_;
    }

    /**
     * @brief Underlying SensorDriver when kCapture is true.
     */
    std::shared_ptr<SensorDriver> GetDriver() const final { return driver_; }

protected:
    /**
     * @brief Factory for the hardware backend; override in concrete modules.
     * @param sensor Sensor configuration entry used to construct the driver.
     * @return Shared pointer to the driver, or nullptr when not implemented.
     */
    virtual std::shared_ptr<SensorDriver> MakeDriver(const Config::Sensor&) {
        return nullptr;
    }

private:
    /**
     * @brief Stamp and forward a captured sample to the upstream hook.
     * @param sample Unique pointer to the captured sample; ignored when null or wrong type.
     */
    void OnSample(std::unique_ptr<SensorSample> sample) {
        if (sample == nullptr || sample->type() != kType) {
            return;
        }
        std::shared_ptr<SensorSample> owned(std::move(sample));
        static_cast<Sample&>(*owned).StampInPlace();
        if (hook_) {
            hook_(std::move(owned));
        }
    }

    // Configured sensor identifier.
    SensorId id_;

    // Upstream callback invoked after stamping each sample.
    SampleHook hook_;

    // Hardware backend when kCapture is true.
    std::shared_ptr<SensorDriver> driver_;

    // Running flag for attach-only plugins when kCapture is false.
    bool running_ = false;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_PLUGIN_HPP_
