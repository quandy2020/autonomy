/*
 * Copyright 2026 The Navigation Authors (navigation)
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

/**
 * @file status.hpp
 * @brief Status and ErrorCode used at module boundaries.
 */

#pragma once

#include <string>
#include <utility>

namespace navigation
{
namespace common
{

/** @brief Stable error categories used across module boundaries. */
enum class ErrorCode
{
    kOk = 0,                 ///< Success.
    kInvalidArgument,        ///< Malformed or out-of-range input.
    kUnavailable,            ///< Temporary unavailability (dependency down).
    kStale,                  ///< Timestamp older than the freshness budget.
    kTimeout,                ///< Operation exceeded its deadline.
    kUnsupported,            ///< Feature/backend not supported.
    kIo,                     ///< File or device I/O failure.
    kModelError,             ///< Model load/inference failure.
    kConfigurationError,     ///< Invalid or inconsistent configuration.
    kInternal,               ///< Unexpected internal failure.
};

/**
 * @brief Structured failure information; never carries an exception.
 *
 * @par Threading
 * Value type; copyable across threads.
 */
struct Status
{
    /** @brief Error category; kOk means success. */
    ErrorCode code{ErrorCode::kOk};
    /** @brief Human-readable failure reason; empty on success. */
    std::string message;

    /** @brief Construct a success status. */
    Status() = default;

    /**
     * @brief Construct a status from category and message.
     *
     * @param[in] error_code Error category.
     * @param[in] error_message Human-readable reason.
     */
    Status(ErrorCode error_code, std::string error_message)
        : code(error_code), message(std::move(error_message))
    {
    }

    /**
     * @brief Return whether the operation succeeded.
     *
     * @return true when code == kOk.
     */
    [[nodiscard]] bool Ok() const
    {
        return code == ErrorCode::kOk;
    }

    /**
     * @brief Build a kInvalidArgument status.
     *
     * @param[in] message Failure reason.
     * @return Status with the corresponding code.
     */
    static Status InvalidArgument(std::string message)
    {
        return Status(ErrorCode::kInvalidArgument, std::move(message));
    }

    /**
     * @brief Build a kUnavailable status.
     *
     * @param[in] message Failure reason.
     * @return Status with the corresponding code.
     */
    static Status Unavailable(std::string message)
    {
        return Status(ErrorCode::kUnavailable, std::move(message));
    }

    /**
     * @brief Build a kStale status.
     *
     * @param[in] message Failure reason.
     * @return Status with the corresponding code.
     */
    static Status Stale(std::string message)
    {
        return Status(ErrorCode::kStale, std::move(message));
    }

    /**
     * @brief Build a kTimeout status.
     *
     * @param[in] message Failure reason.
     * @return Status with the corresponding code.
     */
    static Status Timeout(std::string message)
    {
        return Status(ErrorCode::kTimeout, std::move(message));
    }

    /**
     * @brief Build a kConfigurationError status.
     *
     * @param[in] message Failure reason.
     * @return Status with the corresponding code.
     */
    static Status Configuration(std::string message)
    {
        return Status(ErrorCode::kConfigurationError, std::move(message));
    }

    /**
     * @brief Build a kInternal status.
     *
     * @param[in] message Failure reason.
     * @return Status with the corresponding code.
     */
    static Status Internal(std::string message)
    {
        return Status(ErrorCode::kInternal, std::move(message));
    }
};

}  // namespace common
}  // namespace navigation
