/*
 * Copyright 2026 Autodriver contributors
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

#ifndef AUTODRIVER_SENSOR_PLUGIN_HPP_
#define AUTODRIVER_SENSOR_PLUGIN_HPP_

#include <memory>
#include <string>
#include <utility>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_module.hpp"
#include "autodriver/sensor_traits.hpp"
#include "autolink/common/log.hpp"
#include "autolink/node/writer.hpp"

namespace autodriver {

// kCapture=false: Writer only. kCapture=true: driver + Write.
template <SensorType kType, bool kCapture = true>
class SensorPlugin : public SensorModule {
public:
    using Traits = SensorTraits<kType>;
    using Sample = typename Traits::Sample;
    using Message = typename Traits::Message;

    SensorType GetType() const final { return kType; }
    const SensorId& GetSensorId() const final { return id_; }

    bool Init(const Context& context) final {
        if (!context.node) {
            AERROR << "SensorPlugin Init: null node";
            return false;
        }
        id_ = context.sensor.id;
        hook_ = context.hook;
        const std::string stream =
            hardware::GetString(context.sensor.params, "stream");
        const std::string channel =
            ResolveChannel(context.sensor.channel, id_, kType, stream);
        writer_ = context.node->CreateWriter<Message>(WriterAttr(channel));
        if (!writer_) {
            AERROR << "SensorPlugin failed to create writer on " << channel;
            return false;
        }
        if constexpr (kCapture) {
            driver_ = MakeDriver(context.sensor);
            if (!driver_) {
                writer_.reset();
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
        writer_.reset();
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
        if (sample == nullptr || sample->type() != kType || !writer_) {
            return;
        }
        std::shared_ptr<SensorSample> owned(std::move(sample));
        auto& data = static_cast<Sample&>(*owned);
        data.StampInPlace();
        writer_->Write(std::shared_ptr<Message>(owned, &data.msg));
        if (hook_) {
            hook_(std::move(owned));
        }
    }

    SensorId id_;
    SampleHook hook_;
    std::shared_ptr<SensorDriver> driver_;
    std::shared_ptr<autolink::Writer<Message>> writer_;
    bool running_ = false;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_PLUGIN_HPP_
