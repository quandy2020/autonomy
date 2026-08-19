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

#ifndef AUTODRIVER_SENSOR_PLUGIN_HPP_
#define AUTODRIVER_SENSOR_PLUGIN_HPP_

#include <memory>
#include <utility>

#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_module.hpp"
#include "autodriver/sensor_traits.hpp"

namespace autodriver {

// kCapture=false: attach-only (no hardware). kCapture=true: driver + samples.
template <SensorType kType, bool kCapture = true>
class SensorPlugin : public SensorModule {
public:
    using Traits = SensorTraits<kType>;
    using Sample = typename Traits::Sample;

    SensorType GetType() const final { return kType; }
    const SensorId& GetSensorId() const final { return id_; }

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

    bool Start() final {
        if constexpr (kCapture) {
            return driver_ != nullptr && driver_->Start();
        }
        running_ = true;
        return true;
    }

    void Stop() final {
        if constexpr (kCapture) {
            if (driver_) {
                driver_->Stop();
            }
        } else {
            running_ = false;
        }
    }

    bool IsRunning() const final {
        if constexpr (kCapture) {
            return driver_ != nullptr && driver_->IsRunning();
        }
        return running_;
    }

protected:
    virtual std::shared_ptr<SensorDriver> MakeDriver(const Config::Sensor&) {
        return nullptr;
    }

private:
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

    SensorId id_;
    SampleHook hook_;
    std::shared_ptr<SensorDriver> driver_;
    bool running_ = false;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_PLUGIN_HPP_
