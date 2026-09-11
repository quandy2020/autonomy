/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file can_receiver.hpp
 * @brief CAN receive loop + MessageManager dispatch.
 */

#ifndef AUTODRIVER_CANBUS_CAN_RECEIVER_HPP_
#define AUTODRIVER_CANBUS_CAN_RECEIVER_HPP_

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "autodriver/canbus/protocol_data.hpp"
#include "autodriver/common/can_socket.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace canbus {

/**
 * @class autodriver::canbus::CanReceiver
 * @brief Reads SocketCAN frames on a worker thread and feeds MessageManager.
 * @tparam T Message type managed by the embedded MessageManager.
 *
 * Extended frames have their id masked to 29 bits before dispatch so protocols
 * can register the logical id only.
 */
template <typename T>
class CanReceiver {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(CanReceiver)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(CanReceiver)

    // Optional hook invoked for every received frame before MessageManager.
    using FrameHook = std::function<void(const io::CanFrame&)>;

    /**
     * @brief Construct an idle receiver; call Start() to open the CAN interface.
     */
    CanReceiver() = default;

    /**
     * @brief Stops the receive loop on destruction.
     */
    ~CanReceiver() { Stop(); }

  /**
     * @brief Access the MessageManager for Register / SetPublishCallback.
     * @return Mutable reference to the embedded manager.
     */
    MessageManager<T>& manager() { return manager_; }

    /**
     * @brief Const access to the MessageManager.
     * @return Const reference to the embedded manager.
     */
    const MessageManager<T>& manager() const { return manager_; }

    /**
     * @brief Install a pre-dispatch frame hook (e.g. logging / sniffing).
     * @param[in] hook Callback; empty clears the hook.
     */
    void SetFrameHook(FrameHook hook) { frame_hook_ = std::move(hook); }

    /**
     * @brief Open @p interface and start the read loop.
     * @param[in] interface SocketCAN ifname (e.g. "can0").
     * @param[in] poll_timeout_ms Read timeout per iteration.
     * @return False when Open fails.
     */
    bool Start(const std::string& interface, int poll_timeout_ms = 50) {
        if (running_.exchange(true)) {
            return true;
        }
        if (!socket_.Open(interface)) {
            running_ = false;
            last_error_ = socket_.last_error();
            return false;
        }
        poll_timeout_ms_ = poll_timeout_ms;
        worker_ = std::thread([this]() { Loop(); });
        return true;
    }

    /**
     * @brief Stop the loop, close the socket, and join the worker.
     */
    void Stop() {
        if (!running_.exchange(false)) {
            return;
        }
        socket_.Close();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    /**
     * @brief Whether the receive loop is active.
     * @return True while Start succeeded and Stop has not completed.
     */
    bool IsRunning() const { return running_.load(); }

    /**
     * @brief Last Open failure message.
     * @return Reference to the last error string (empty when ok).
     */
    const std::string& last_error() const { return last_error_; }

private:
    /**
     * @brief Worker: Read → mask extended id → frame_hook_ → manager_.Parse.
     */
    void Loop() {
        io::CanFrame frame{};
        while (running_.load()) {
            if (!socket_.Read(frame, poll_timeout_ms_)) {
                continue;
            }
            if (frame.extended) {
                frame.id &= 0x1FFFFFFFu;
            }
            if (frame_hook_) {
                frame_hook_(frame);
            }
            manager_.Parse(frame);
        }
    }

    // Protocol registry and publish sink.
    MessageManager<T> manager_;
    // SocketCAN fd wrapper.
    io::CanSocket socket_;
    // True while Loop() should run.
    std::atomic<bool> running_{false};
    // Background receive thread.
    std::thread worker_;
    // Read timeout passed to CanSocket::Read.
    int poll_timeout_ms_ = 50;
    // Optional pre-dispatch hook.
    FrameHook frame_hook_;
    // Last Start/Open error.
    std::string last_error_;
};

}  // namespace canbus
}  // namespace autodriver

#endif  // AUTODRIVER_CANBUS_CAN_RECEIVER_HPP_
