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
 * @file stream.hpp
 * @brief Transport abstraction (Stream × Parser separation).
 */

#ifndef AUTODRIVER_COMMON_STREAM_HPP_
#define AUTODRIVER_COMMON_STREAM_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "autodriver/common/status.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace common {

/**
 * @class autodriver::common::Stream
 * @brief Byte transport: serial / UDP / TCP / NTRIP.
 */
class Stream {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(Stream)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(Stream)

    /**
     * @brief Default constructor (DISALLOW_COPY suppresses the implicit one).
     */
    Stream() = default;

    using Status = diagnostics::DeviceStatus;

    /**
     * @brief Virtual destructor for polymorphic streams.
     */
    virtual ~Stream() = default;

    /**
     * @brief Opens the underlying transport.
     * @return true on success; false if the transport could not be opened.
     */
    virtual bool Connect() = 0;

    /**
     * @brief Closes the underlying transport.
     */
    virtual void Disconnect() = 0;

    /**
     * @brief Current device/transport status.
     * @return Coarse DeviceStatus for this stream.
     */
    virtual Status status() const = 0;

    /**
     * @brief Reads up to max_bytes; returns 0 on timeout without error.
     * @param[out] buffer Destination buffer.
     * @param[in] max_bytes Capacity of @p buffer.
     * @param[in] timeout_ms Read timeout in milliseconds.
     * @return Number of bytes read.
     */
    virtual std::size_t Read(std::uint8_t* buffer, std::size_t max_bytes,
                             int timeout_ms) = 0;

    /**
     * @brief Writes @p length bytes from @p buffer.
     * @param[out] buffer Source bytes to write.
     * @param[in] length Number of bytes to write.
     * @return true when the full payload was written; false on short write or error.
     */
    virtual bool Write(const std::uint8_t* buffer, std::size_t length) = 0;

    /**
     * @brief Last transport error string (empty when none).
     * @return Human-readable error text retained by the stream.
     */
    virtual const std::string& last_error() const = 0;
};

/**
 * @brief Creates a serial Stream for @p device at @p baud_rate.
 * @param[in] device TTY path (e.g. "/dev/ttyUSB0").
 * @param[in] baud_rate Serial baud rate.
 * @return Owning Stream unique_ptr; never null (Connect may still fail later).
 */
std::unique_ptr<Stream> CreateSerialStream(std::string device, int baud_rate);

/**
 * @brief UDP bind stream (Velodyne/Hesai data ports). host empty → INADDR_ANY.
 * @param[in] host Bind address; empty means INADDR_ANY.
 * @param[in] port UDP port to bind.
 * @return Owning Stream unique_ptr.
 */
std::unique_ptr<Stream> CreateUdpStream(std::string host, int port);

/**
 * @brief TCP client stream to host:port.
 * @param[in] host Remote host name or address.
 * @param[in] port Remote TCP port.
 * @return Owning Stream unique_ptr.
 */
std::unique_ptr<Stream> CreateTcpStream(std::string host, int port);

/**
 * @brief NTRIP client (TCP + HTTP GET); then read correction stream.
 * @param[in] host NTRIP caster host.
 * @param[in] port NTRIP caster port.
 * @param[in] mountpoint Mountpoint path on the caster.
 * @param[in] user Optional username for Basic auth.
 * @param[in] password Optional password for Basic auth.
 * @return Owning Stream unique_ptr.
 */
std::unique_ptr<Stream> CreateNtripStream(std::string host, int port,
                                          std::string mountpoint,
                                          std::string user = "",
                                          std::string password = "");

/**
 * @brief Disconnect + Connect with backoff. Returns true when status is kOk.
 * @param[in,out] stream Stream to reconnect; must be non-null.
 * @param[in] max_attempts Maximum Connect attempts after Disconnect.
 * @param[in] delay_ms Sleep between attempts in milliseconds.
 * @return true when @p stream reports DeviceStatus::kOk after reconnect.
 */
bool ReconnectStream(Stream* stream, int max_attempts = 3,
                     int delay_ms = 200);

}  // namespace common
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_STREAM_HPP_
