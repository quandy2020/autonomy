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

#ifndef AUTODRIVER_BRIDGE_PUBLISHER_HPP_
#define AUTODRIVER_BRIDGE_PUBLISHER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "autodriver/sample_sink.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/node/node.hpp"

namespace autodriver {
namespace bridge {

// Owns the autolink Node and Writers. Core autodriver never Write()s.
class Publisher : public SampleSink {
public:
    explicit Publisher(std::string node_name = "autodriver");
    ~Publisher() override;

    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

    bool Initialize();
    autolink::Node* node() { return node_.get(); }

    bool OnAttach(const Config::Sensor& sensor, SensorType type) override;
    void OnDetach(const SensorId& id) override;
    void OnSample(std::shared_ptr<SensorSample> sample) override;

private:
    using WriteFn = std::function<void(const std::shared_ptr<SensorSample>&)>;

    template <SensorType kType>
    bool OpenWriter(const Config::Sensor& sensor);
    bool OpenCameraWriter(const Config::Sensor& sensor);

    std::string node_name_;
    std::shared_ptr<autolink::Node> node_;
    std::unordered_map<SensorId, WriteFn> writers_;
    mutable autolink::base::AtomicRWLock lock_;
};

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_PUBLISHER_HPP_
