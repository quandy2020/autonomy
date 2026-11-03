/*
 * Copyright 2026 The Openbot Authors
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

#include "atlas/instance_lock.hpp"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace autonomy::localization::atlas {
namespace {

constexpr char kDefaultLockPath[] = "/tmp/autonomy.localization.atlas.lock";
constexpr char kLockPathEnvironment[] =
    "AUTONOMY_ATLAS_INSTANCE_LOCK_PATH";

void SetError(const std::string& message, std::string* error_message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::string SystemError(const std::string& action, int error_number) {
    return action + ": " + std::strerror(error_number);
}

void CloseDescriptor(int descriptor) {
    if (descriptor < 0) {
        return;
    }
    ::close(descriptor);
}

bool WriteAll(int descriptor, const std::string& data) {
    std::size_t offset = 0;
    while (offset < data.size()) {
        const ssize_t written =
            ::write(descriptor, data.data() + offset, data.size() - offset);
        if (written > 0) {
            offset += static_cast<std::size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        if (written == 0) {
            errno = EIO;
        }
        return false;
    }
    return true;
}

std::string ReadOwnerMetadata(int descriptor) {
    std::array<char, 512> buffer{};
    ssize_t size;
    do {
        size = ::pread(descriptor, buffer.data(), buffer.size() - 1, 0);
    } while (size < 0 && errno == EINTR);
    if (size <= 0) {
        return {};
    }
    std::string metadata(buffer.data(), static_cast<std::size_t>(size));
    for (char& character : metadata) {
        const unsigned char value = static_cast<unsigned char>(character);
        if (value < 0x20 || value == 0x7f) {
            character = ' ';
        }
    }
    while (!metadata.empty() && metadata.back() == ' ') {
        metadata.pop_back();
    }
    return metadata;
}

}  // namespace

InstanceLock::~InstanceLock() {
    Release();
}

bool InstanceLock::TryAcquire(const std::string& path,
                              const std::string& config_path,
                              std::string* error_message) {
    if (held()) {
        return true;
    }
    if (path.empty()) {
        SetError("Atlas instance lock path is empty", error_message);
        return false;
    }

    int flags = O_RDWR | O_CREAT | O_CLOEXEC;
#ifdef O_NOFOLLOW
    flags |= O_NOFOLLOW;
#endif
    const int descriptor = ::open(path.c_str(), flags, S_IRUSR | S_IWUSR);
    if (descriptor < 0) {
        SetError(SystemError("failed to open Atlas instance lock " + path,
                             errno),
                 error_message);
        return false;
    }

    struct stat info {};
    if (::fstat(descriptor, &info) != 0) {
        const int error_number = errno;
        CloseDescriptor(descriptor);
        SetError(SystemError("failed to inspect Atlas instance lock " + path,
                             error_number),
                 error_message);
        return false;
    }
    if (!S_ISREG(info.st_mode) || info.st_nlink != 1 ||
        info.st_uid != ::geteuid()) {
        CloseDescriptor(descriptor);
        SetError("Atlas instance lock must be a single-link regular file "
                 "owned by the current user: " +
                     path,
                 error_message);
        return false;
    }
    if (::fchmod(descriptor, S_IRUSR | S_IWUSR) != 0) {
        const int error_number = errno;
        CloseDescriptor(descriptor);
        SetError(SystemError("failed to secure Atlas instance lock " + path,
                             error_number),
                 error_message);
        return false;
    }

    if (::flock(descriptor, LOCK_EX | LOCK_NB) != 0) {
        const int error_number = errno;
        const std::string owner = ReadOwnerMetadata(descriptor);
        CloseDescriptor(descriptor);
        if (error_number == EWOULDBLOCK || error_number == EAGAIN) {
            SetError("another Atlas instance is already running" +
                         (owner.empty() ? std::string()
                                        : " (" + owner + ")"),
                     error_message);
        } else {
            SetError(SystemError("failed to acquire Atlas instance lock " +
                                     path,
                                 error_number),
                     error_message);
        }
        return false;
    }

    const std::string metadata =
        "pid=" + std::to_string(::getpid()) + " config=" + config_path + "\n";
    if (::ftruncate(descriptor, 0) != 0 ||
        ::lseek(descriptor, 0, SEEK_SET) < 0 ||
        !WriteAll(descriptor, metadata) || ::fsync(descriptor) != 0) {
        const int error_number = errno;
        ::flock(descriptor, LOCK_UN);
        CloseDescriptor(descriptor);
        SetError(SystemError("failed to write Atlas instance lock " + path,
                             error_number),
                 error_message);
        return false;
    }

    descriptor_ = descriptor;
    return true;
}

void InstanceLock::Release() {
    const int descriptor = std::exchange(descriptor_, -1);
    if (descriptor < 0) {
        return;
    }
    while (::flock(descriptor, LOCK_UN) != 0 && errno == EINTR) {
    }
    CloseDescriptor(descriptor);
}

std::string ResolveInstanceLockPath() {
    const char* override_path = std::getenv(kLockPathEnvironment);
    std::filesystem::path path =
        override_path != nullptr && override_path[0] != '\0'
            ? std::filesystem::path(override_path)
            : std::filesystem::path(kDefaultLockPath);
    std::error_code error;
    if (path.is_relative()) {
        const std::filesystem::path absolute =
            std::filesystem::absolute(path, error);
        if (!error) {
            path = absolute;
        }
    }
    return path.lexically_normal().string();
}

}  // namespace autonomy::localization::atlas
