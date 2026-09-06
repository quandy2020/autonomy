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

/**
 * @file
 * @brief CAN ProtocolData + MessageManager.
 */

#ifndef AUTODRIVER_CANBUS_PROTOCOL_DATA_HPP_
#define AUTODRIVER_CANBUS_PROTOCOL_DATA_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "autodriver/common/can_socket.hpp"

namespace autodriver {
namespace canbus {

/**
 * @class autodriver::canbus::ProtocolData
 * @brief Decode one CAN frame into a sensor message of type T.
 * @tparam T Application message produced by Parse (e.g. ImuCanEvent).
 *
 * One subclass typically handles a single CAN id (accel frame, GPS lat/lon, …).
 */
template <typename T>
class ProtocolData {
public:
    virtual ~ProtocolData() = default;

    /**
     * @brief CAN identifier this protocol handles (11-bit or 29-bit).
     */
    virtual std::uint32_t can_id() const = 0;

    /**
     * @brief Parse payload into @p msg.
     * @param frame Incoming classical CAN frame (id already normalized).
     * @param msg Output message; must not be null.
     * @return False to skip the publish callback.
     */
    virtual bool Parse(const io::CanFrame& frame, T* msg) const = 0;
};

/**
 * @class autodriver::canbus::MessageManager
 * @brief Registers ProtocolData by CAN id and dispatches received frames.
 * @tparam T Shared message type across registered protocols.
 */
template <typename T>
class MessageManager {
public:
    using ProtocolPtr = std::shared_ptr<ProtocolData<T>>;
    using PublishFn = std::function<void(const T&)>;

    /**
     * @brief Register a protocol; later registrations with the same id replace.
     * @param protocol Non-null ProtocolData instance.
     */
    void Register(ProtocolPtr protocol) {
        if (!protocol) {
            return;
        }
        protocols_[protocol->can_id()] = std::move(protocol);
    }

    /**
     * @brief Callback invoked after a successful ProtocolData::Parse.
     * @param callback May be empty to disable publishing.
     */
    void SetPublishCallback(PublishFn callback) {
        publish_ = std::move(callback);
    }

    /**
     * @brief Lookup protocol by frame.id, Parse, then optionally publish.
     * @param frame Frame to dispatch (extended ids should already be masked).
     * @return True when a protocol matched and Parse succeeded.
     */
    bool Parse(const io::CanFrame& frame) {
        const auto it = protocols_.find(frame.id);
        if (it == protocols_.end()) {
            return false;
        }
        T msg{};
        if (!it->second->Parse(frame, &msg)) {
            return false;
        }
        if (publish_) {
            publish_(msg);
        }
        return true;
    }

    /**
     * @brief Number of registered protocols.
     */
    std::size_t size() const { return protocols_.size(); }

private:
    // can_id → protocol decoder.
    std::unordered_map<std::uint32_t, ProtocolPtr> protocols_;
    
    // Optional sink after successful Parse.
    PublishFn publish_;
};

}  // namespace canbus
}  // namespace autodriver

#endif  // AUTODRIVER_CANBUS_PROTOCOL_DATA_HPP_
