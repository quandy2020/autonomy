/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <memory>

#include "autolink/io/poll_handler.hpp"

namespace autolink {
namespace io {

class Session
{
public:
    using SessionPtr = std::shared_ptr<Session>;

    Session();
    explicit Session(int fd);

    int Socket(int domain, int type, int protocol);
    int Listen(int backlog);
    int Bind(const struct sockaddr* addr, socklen_t addrlen);
    SessionPtr Accept(struct sockaddr* addr, socklen_t* addrlen);
    int Connect(const struct sockaddr* addr, socklen_t addrlen);
    int Close();

    ssize_t Recv(void* buf, size_t len, int flags, int timeout_ms);
    ssize_t RecvFrom(void* buf, size_t len, int flags,
                     struct sockaddr* src_addr, socklen_t* addrlen,
                     int timeout_ms);
    ssize_t Send(const void* buf, size_t len, int flags, int timeout_ms);
    ssize_t SendTo(const void* buf, size_t len, int flags,
                   const struct sockaddr* dest_addr, socklen_t addrlen,
                   int timeout_ms);
    ssize_t Read(void* buf, size_t count, int timeout_ms);
    ssize_t Write(const void* buf, size_t count, int timeout_ms);

    int fd() const {
        return fd_;
    }
    void set_fd(int fd) {
        fd_ = fd;
        if (poll_handler_) {
            poll_handler_->set_fd(fd_);
        }
    }

private:
    int fd_;
    std::shared_ptr<PollHandler> poll_handler_;
};

}  // namespace io
}  // namespace autolink
