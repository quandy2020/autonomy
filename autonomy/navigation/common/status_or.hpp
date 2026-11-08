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
 * @file status_or.hpp
 * @brief Value-or-Status without exceptions crossing module boundaries.
 */

#pragma once

#include <type_traits>
#include <utility>
#include <variant>

#include "navigation/common/status.hpp"

namespace navigation
{
namespace common
{

/**
 * @brief Holds either a value or a Status.
 *
 * @tparam T Value type; must be movable.
 */
template <typename T>
class StatusOr
{
public:
    /** @brief Construct a default value (T must be default constructible). */
    StatusOr() : storage_(std::in_place_index<0>, T{})
    {
    }

    /**
     * @brief Construct from a value.
     *
     * @param[in] value Value to store; moved into the variant.
     */
    StatusOr(T value)  // NOLINT(google-explicit-constructor)
        : storage_(std::in_place_index<0>, std::move(value))
    {
    }

    /**
     * @brief Construct from an error.
     *
     * @param[in] status Failure status.
     */
    StatusOr(Status status)  // NOLINT(google-explicit-constructor)
        : storage_(std::in_place_index<1>, std::move(status))
    {
    }

    /** @brief Return true when a value is stored. */
    [[nodiscard]] bool Ok() const
    {
        return storage_.index() == 0;
    }

    /** @brief Boolean conversion equivalent to Ok(). */
    explicit operator bool() const
    {
        return Ok();
    }

    /** @brief Access the stored value; callers must check Ok() first. */
    T& Value()
    {
        return std::get<0>(storage_);
    }

    /** @brief Const access to the stored value. */
    const T& Value() const
    {
        return std::get<0>(storage_);
    }

    /** @brief Move the stored value out. */
    T&& Take()
    {
        return std::move(std::get<0>(storage_));
    }

    /** @brief Access the error; callers must check Ok()==false first. */
    const Status& Error() const
    {
        return std::get<1>(storage_);
    }

    /**
     * @brief Return the value or a fallback.
     *
     * @param[in] fallback Value returned on error.
     */
    T ValueOr(T fallback) const
    {
        return Ok() ? std::get<0>(storage_) : std::move(fallback);
    }

private:
    /** @brief Either the value (index 0) or Status (index 1). */
    std::variant<T, Status> storage_;
};

}  // namespace common
}  // namespace navigation
