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
 * @brief CAN client abstraction: SocketCAN + in-process Fake bus.
 */

#ifndef AUTODRIVER_CANBUS_CAN_CLIENT_HPP_
#define AUTODRIVER_CANBUS_CAN_CLIENT_HPP_

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>

#include "autodriver/common/can_socket.hpp"

namespace autodriver {
namespace canbus {

/**
 * @class autodriver::canbus::CanClient
 * @brief Abstract CAN I/O port.
 *
 * Implementations hide SocketCAN vs in-memory Fake buses so ProtocolData
 * and sensors can be unit-tested without hardware.
 */
class CanClient {
public:
    virtual ~CanClient() = default;

    /**
     * @brief Open or bind the named channel (e.g. "can0" or "fake0").
     * @param channel Interface or fake bus name.
     * @return True on success.
     */
    virtual bool Init(const std::string& channel) = 0;

    /**
     * @brief Release the underlying resource.
     */
    virtual void Close() = 0;

    /**
     * @brief Transmit one frame.
     * @param frame Classical CAN frame to send.
     * @return True when the frame was accepted by the backend.
     */
    virtual bool Send(const io::CanFrame& frame) = 0;

    /**
     * @brief Receive one frame, optionally waiting.
     * @param frame Output frame; must not be null.
     * @param timeout_ms Poll timeout for blocking backends (Fake ignores).
     * @return True when a frame was copied into @p frame.
     */
    virtual bool Receive(io::CanFrame* frame, int timeout_ms) = 0;

    /**
     * @brief Last Init/Send/Receive error string.
     */
    virtual const std::string& last_error() const = 0;
};

/**
 * @class autodriver::canbus::SocketCanClient
 * @brief Linux SocketCAN-backed CanClient wrapping io::CanSocket.
 */
class SocketCanClient : public CanClient {
public:
    /**
     * @brief Open SocketCAN interface @p channel.
     */
    bool Init(const std::string& channel) override {
        return socket_.Open(channel);
    }

    void Close() override { socket_.Close(); }

    bool Send(const io::CanFrame& frame) override {
        return socket_.Write(frame);
    }

    bool Receive(io::CanFrame* frame, int timeout_ms) override {
        if (frame == nullptr) {
            return false;
        }
        return socket_.Read(*frame, timeout_ms);
    }

    const std::string& last_error() const override {
        return socket_.last_error();
    }

private:
    // Raw SocketCAN fd wrapper.
    io::CanSocket socket_;
};

/**
 * @class autodriver::canbus::FakeCanClient
 * @brief Shared in-memory bus for unit tests (channel name e.g. "fake0").
 *
 * Send() enqueues into a per-channel RX queue; Receive() dequeues.
 * Inject()/Clear() help tests feed frames without a real Sender.
 */
class FakeCanClient : public CanClient {
public:
    /**
     * @brief Bind this client to a named fake bus (creates the bus if needed).
     */
    bool Init(const std::string& channel) override {
        channel_ = channel;
        Bus(channel_);
        last_error_.clear();
        return true;
    }

    void Close() override {}

    bool Send(const io::CanFrame& frame) override {
        auto& bus = Bus(channel_);
        std::lock_guard<std::mutex> lock(bus.mutex);
        bus.rx.push(frame);
        return true;
    }

    bool Receive(io::CanFrame* frame, int /*timeout_ms*/) override {
        if (frame == nullptr) {
            return false;
        }
        auto& bus = Bus(channel_);
        std::lock_guard<std::mutex> lock(bus.mutex);
        if (bus.rx.empty()) {
            return false;
        }
        *frame = bus.rx.front();
        bus.rx.pop();
        return true;
    }

    const std::string& last_error() const override { return last_error_; }

    /**
     * @brief Push a frame onto the bus RX queue as if received from the wire.
     * @param channel Fake bus name.
     * @param frame Frame to inject.
     */
    static void Inject(const std::string& channel, const io::CanFrame& frame) {
        auto& bus = Bus(channel);
        std::lock_guard<std::mutex> lock(bus.mutex);
        bus.rx.push(frame);
    }

    /**
     * @brief Drop all queued frames on a fake bus.
     * @param channel Fake bus name.
     */
    static void Clear(const std::string& channel) {
        auto& bus = Bus(channel);
        std::lock_guard<std::mutex> lock(bus.mutex);
        std::queue<io::CanFrame> empty;
        bus.rx.swap(empty);
    }

private:
    /**
     * @brief Per-channel shared queue state.
     */
    struct SharedBus {
        // Guards rx.
        std::mutex mutex;
        // Frames waiting for Receive().
        std::queue<io::CanFrame> rx;
    };

    /**
     * @brief Lookup or create the shared bus for @p channel.
     */
    static SharedBus& Bus(const std::string& channel) {
        static std::mutex map_mutex;
        static std::unordered_map<std::string, std::unique_ptr<SharedBus>>
            buses;
        std::lock_guard<std::mutex> lock(map_mutex);
        auto& ptr = buses[channel];
        if (!ptr) {
            ptr = std::make_unique<SharedBus>();
        }
        return *ptr;
    }

    // Bound fake bus name after Init().
    std::string channel_;
    // Last error text (rarely set for Fake).
    std::string last_error_;
};

/**
 * @brief Factory: channel starting with "fake" → FakeCanClient, else SocketCAN.
 * @param channel Interface or fake bus name.
 * @return New client instance (not yet Init()'d for Socket; Fake Init is cheap).
 */
inline std::unique_ptr<CanClient> CreateCanClient(const std::string& channel) {
    if (channel.rfind("fake", 0) == 0) {
        return std::make_unique<FakeCanClient>();
    }
    return std::make_unique<SocketCanClient>();
}

}  // namespace canbus
}  // namespace autodriver

#endif  // AUTODRIVER_CANBUS_CAN_CLIENT_HPP_
