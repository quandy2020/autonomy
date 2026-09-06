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

#include "autodriver/common/stream.hpp"

#include <cerrno>
#include <cstring>
#include <thread>
#include <utility>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "autodriver/common/serial_port.hpp"
#include "autolink/time/duration.hpp"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace autodriver {
namespace common {
namespace {

class SerialStream final : public Stream {
public:
    SerialStream(std::string device, int baud_rate)
        : device_(std::move(device)), baud_rate_(baud_rate) {}

    bool Connect() override {
        if (port_.IsOpen()) {
            status_ = Status::kOk;
            return true;
        }
        if (!port_.Open(device_, baud_rate_)) {
            status_ = Status::kError;
            return false;
        }
        status_ = Status::kOk;
        return true;
    }

    void Disconnect() override {
        port_.Close();
        status_ = Status::kDisconnected;
    }

    Status status() const override { return status_; }

    std::size_t Read(std::uint8_t* buffer, std::size_t max_bytes,
                     int timeout_ms) override {
        if (!port_.IsOpen()) {
            status_ = Status::kDisconnected;
            return 0;
        }
        return port_.Read(buffer, max_bytes, timeout_ms);
    }

    bool Write(const std::uint8_t* buffer, std::size_t length) override {
        if (!port_.IsOpen()) {
            status_ = Status::kDisconnected;
            return false;
        }
        return port_.Write(buffer, length);
    }

    const std::string& last_error() const override {
        return port_.last_error();
    }

private:
    std::string device_;
    int baud_rate_ = 115200;
    io::SerialPort port_;
    Status status_ = Status::kDisconnected;
};

class UdpStream final : public Stream {
public:
    UdpStream(std::string host, int port)
        : host_(std::move(host)), port_(port) {}

    ~UdpStream() override { Disconnect(); }

    bool Connect() override {
        if (fd_ >= 0) {
            status_ = Status::kOk;
            return true;
        }
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            last_error_ = std::strerror(errno);
            status_ = Status::kError;
            return false;
        }
        int yes = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<std::uint16_t>(port_));
        if (host_.empty() || host_ == "0.0.0.0") {
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
        } else if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
            last_error_ = "invalid UDP bind host: " + host_;
            ::close(fd_);
            fd_ = -1;
            status_ = Status::kError;
            return false;
        }
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            last_error_ = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            status_ = Status::kError;
            return false;
        }
        status_ = Status::kOk;
        return true;
    }

    void Disconnect() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        status_ = Status::kDisconnected;
    }

    Status status() const override { return status_; }

    std::size_t Read(std::uint8_t* buffer, std::size_t max_bytes,
                     int timeout_ms) override {
        if (fd_ < 0) {
            status_ = Status::kDisconnected;
            return 0;
        }
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, timeout_ms);
        if (ready == 0) {
            return 0;
        }
        if (ready < 0) {
            if (errno == EINTR) {
                return 0;
            }
            last_error_ = std::strerror(errno);
            status_ = Status::kError;
            return 0;
        }
        const ssize_t n =
            ::recvfrom(fd_, buffer, max_bytes, 0, nullptr, nullptr);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            last_error_ = std::strerror(errno);
            status_ = Status::kError;
            return 0;
        }
        return static_cast<std::size_t>(n);
    }

    bool Write(const std::uint8_t* /*buffer*/, std::size_t /*length*/) override {
        last_error_ = "UdpStream is bind-only (no peer Write)";
        return false;
    }

    const std::string& last_error() const override { return last_error_; }

private:
    std::string host_;
    int port_ = 0;
    int fd_ = -1;
    Status status_ = Status::kDisconnected;
    std::string last_error_;
};

class TcpStream final : public Stream {
public:
    TcpStream(std::string host, int port)
        : host_(std::move(host)), port_(port) {}

    ~TcpStream() override { Disconnect(); }

    bool Connect() override {
        if (fd_ >= 0) {
            status_ = Status::kOk;
            return true;
        }
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) {
            last_error_ = std::strerror(errno);
            status_ = Status::kError;
            return false;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<std::uint16_t>(port_));
        if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
            last_error_ = "invalid TCP host: " + host_;
            ::close(fd_);
            fd_ = -1;
            status_ = Status::kError;
            return false;
        }
        if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
            0) {
            last_error_ = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            status_ = Status::kError;
            return false;
        }
        status_ = Status::kOk;
        return true;
    }

    void Disconnect() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        status_ = Status::kDisconnected;
    }

    Status status() const override { return status_; }

    std::size_t Read(std::uint8_t* buffer, std::size_t max_bytes,
                     int timeout_ms) override {
        if (fd_ < 0) {
            status_ = Status::kDisconnected;
            return 0;
        }
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, timeout_ms);
        if (ready <= 0) {
            if (ready < 0 && errno != EINTR) {
                last_error_ = std::strerror(errno);
                status_ = Status::kError;
            }
            return 0;
        }
        const ssize_t n = ::recv(fd_, buffer, max_bytes, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            last_error_ = std::strerror(errno);
            status_ = Status::kError;
            return 0;
        }
        if (n == 0) {
            status_ = Status::kDisconnected;
            return 0;
        }
        return static_cast<std::size_t>(n);
    }

    bool Write(const std::uint8_t* buffer, std::size_t length) override {
        if (fd_ < 0 || buffer == nullptr) {
            return false;
        }
        std::size_t sent = 0;
        while (sent < length) {
            const ssize_t n =
                ::send(fd_, buffer + sent, length - sent, MSG_NOSIGNAL);
            if (n <= 0) {
                last_error_ = std::strerror(errno);
                status_ = Status::kError;
                return false;
            }
            sent += static_cast<std::size_t>(n);
        }
        return true;
    }

    const std::string& last_error() const override { return last_error_; }

    int fd() const { return fd_; }

private:
    std::string host_;
    int port_ = 0;
    int fd_ = -1;
    Status status_ = Status::kDisconnected;
    std::string last_error_;
};

class NtripStream final : public Stream {
public:
    NtripStream(std::string host, int port, std::string mountpoint,
                std::string user, std::string password)
        : tcp_(std::move(host), port),
          mountpoint_(std::move(mountpoint)),
          user_(std::move(user)),
          password_(std::move(password)) {}

    bool Connect() override {
        if (!tcp_.Connect()) {
            return false;
        }
        std::string request = "GET /" + mountpoint_ +
                              " HTTP/1.0\r\nUser-Agent: autodriver/NTRIP\r\n";
        if (!user_.empty()) {
            // Minimal Base64 for user:pass (NTRIP classic).
            const std::string token = user_ + ":" + password_;
            static const char* kTable =
                "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz012345"
                "6789+/";
            std::string b64;
            const auto* bytes =
                reinterpret_cast<const unsigned char*>(token.data());
            const std::size_t len = token.size();
            for (std::size_t i = 0; i < len; i += 3) {
                const unsigned int v =
                    (bytes[i] << 16) |
                    ((i + 1 < len ? bytes[i + 1] : 0) << 8) |
                    (i + 2 < len ? bytes[i + 2] : 0);
                b64.push_back(kTable[(v >> 18) & 63]);
                b64.push_back(kTable[(v >> 12) & 63]);
                b64.push_back(i + 1 < len ? kTable[(v >> 6) & 63] : '=');
                b64.push_back(i + 2 < len ? kTable[v & 63] : '=');
            }
            request += "Authorization: Basic " + b64 + "\r\n";
        }
        request += "\r\n";
        if (!tcp_.Write(reinterpret_cast<const std::uint8_t*>(request.data()),
                        request.size())) {
            last_error_ = tcp_.last_error();
            tcp_.Disconnect();
            return false;
        }
        // Drain HTTP headers until blank line (best-effort, short timeout).
        std::string header;
        std::uint8_t buf[256];
        for (int i = 0; i < 40; ++i) {
            const std::size_t n = tcp_.Read(buf, sizeof(buf), 200);
            if (n == 0) {
                continue;
            }
            header.append(reinterpret_cast<char*>(buf), n);
            if (header.find("\r\n\r\n") != std::string::npos) {
                break;
            }
        }
        if (header.find("200") == std::string::npos &&
            header.find("ICY 200") == std::string::npos) {
            last_error_ = "NTRIP handshake failed: " + header.substr(0, 120);
            tcp_.Disconnect();
            return false;
        }
        return true;
    }

    void Disconnect() override { tcp_.Disconnect(); }
    Status status() const override { return tcp_.status(); }
    std::size_t Read(std::uint8_t* buffer, std::size_t max_bytes,
                     int timeout_ms) override {
        return tcp_.Read(buffer, max_bytes, timeout_ms);
    }
    bool Write(const std::uint8_t* buffer, std::size_t length) override {
        return tcp_.Write(buffer, length);
    }
    const std::string& last_error() const override {
        return last_error_.empty() ? tcp_.last_error() : last_error_;
    }

private:
    TcpStream tcp_;
    std::string mountpoint_;
    std::string user_;
    std::string password_;
    std::string last_error_;
};

}  // namespace

std::unique_ptr<Stream> CreateSerialStream(std::string device, int baud_rate) {
    return std::make_unique<SerialStream>(std::move(device), baud_rate);
}

std::unique_ptr<Stream> CreateUdpStream(std::string host, int port) {
    return std::make_unique<UdpStream>(std::move(host), port);
}

std::unique_ptr<Stream> CreateTcpStream(std::string host, int port) {
    return std::make_unique<TcpStream>(std::move(host), port);
}

std::unique_ptr<Stream> CreateNtripStream(std::string host, int port,
                                          std::string mountpoint,
                                          std::string user,
                                          std::string password) {
    return std::make_unique<NtripStream>(std::move(host), port,
                                         std::move(mountpoint), std::move(user),
                                         std::move(password));
}

bool ReconnectStream(Stream* stream, int max_attempts, int delay_ms) {
    if (stream == nullptr || max_attempts <= 0) {
        return false;
    }
    for (int i = 0; i < max_attempts; ++i) {
        stream->Disconnect();
        if (stream->Connect() &&
            stream->status() == Stream::Status::kOk) {
            return true;
        }
        if (delay_ms > 0) {
            autolink::Duration(static_cast<std::int64_t>(delay_ms) * 1'000'000)
                .Sleep();
        }
    }
    return false;
}

}  // namespace common
}  // namespace autodriver
