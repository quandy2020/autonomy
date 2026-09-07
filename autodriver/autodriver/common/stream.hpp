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
 * @brief Transport abstraction (Stream × Parser separation).
 */

#ifndef AUTODRIVER_COMMON_STREAM_HPP_
#define AUTODRIVER_COMMON_STREAM_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "autodriver/common/status.hpp"

namespace autodriver {
namespace common {

/**
 * @class autodriver::common::Stream
 * @brief Byte transport: serial / UDP / TCP / NTRIP.
 */
class Stream {
public:
    using Status = diagnostics::DeviceStatus;

    virtual ~Stream() = default;

    virtual bool Connect() = 0;
    virtual void Disconnect() = 0;
    virtual Status status() const = 0;

    /**
     * @brief Reads up to max_bytes; returns 0 on timeout without error.
     */
    virtual std::size_t Read(std::uint8_t* buffer, std::size_t max_bytes,
                             int timeout_ms) = 0;

    virtual bool Write(const std::uint8_t* buffer, std::size_t length) = 0;

    virtual const std::string& last_error() const = 0;
};

std::unique_ptr<Stream> CreateSerialStream(std::string device, int baud_rate);

/**
 * @brief UDP bind stream (Velodyne/Hesai data ports). host empty → INADDR_ANY.
 */
std::unique_ptr<Stream> CreateUdpStream(std::string host, int port);

/**
 * @brief TCP client stream to host:port.
 */
std::unique_ptr<Stream> CreateTcpStream(std::string host, int port);

/**
 * @brief NTRIP client (TCP + HTTP GET); then read correction stream.
 */
std::unique_ptr<Stream> CreateNtripStream(std::string host, int port,
                                          std::string mountpoint,
                                          std::string user = "",
                                          std::string password = "");

/**
 * @brief Disconnect + Connect with backoff. Returns true when status is kOk.
 */
bool ReconnectStream(Stream* stream, int max_attempts = 3,
                     int delay_ms = 200);

}  // namespace common
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_STREAM_HPP_
