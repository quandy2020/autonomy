/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file status.hpp
 * @brief Lightweight status for Autoloop modules (no exceptions on the hot path).
 */

#include <string>
#include <utility>

namespace autoloop {
namespace common {

/**
 * @brief OK / error pair. Empty message means success.
 */
class Status
{
public:
    Status() = default;

    /**
     * @brief Construct an error status.
     * @param [in] message: Human-readable failure.
     */
    explicit Status(std::string message) : message_(std::move(message)) {}

    /**
     * @return True when no error message is set.
     */
    bool ok() const { return message_.empty(); }

    /**
     * @return Error text; empty on success.
     */
    const std::string& message() const { return message_; }

    /**
     * @return Successful status.
     */
    static Status OK() { return Status(); }

    /**
     * @brief Build an error status.
     * @param [in] message: Failure text.
     */
    static Status Error(std::string message) {
        return Status(std::move(message));
    }

private:
    std::string message_;
};

}  // namespace common
}  // namespace autoloop
