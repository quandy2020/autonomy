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
 * @file array_view.hpp
 * @brief Non-owning contiguous view; C++17 substitute for std::span.
 */

#pragma once

#include <cstddef>
#include <type_traits>

namespace navigation
{
namespace common
{

/** @brief Non-owning view over a contiguous array. */
template <typename T>
class ArrayView
{
public:
    using value_type = T;
    using size_type = std::size_t;

    /** @brief Construct an empty view. */
    constexpr ArrayView() noexcept = default;

    /**
     * @brief Construct from pointer and length.
     *
     * @param[in] data Start pointer (may be nullptr when size is 0).
     * @param[in] size Number of elements.
     */
    constexpr ArrayView(T* data, size_type size) noexcept : data_(data), size_(size)
    {
    }

    /**
     * @brief Construct from a fixed-size array.
     *
     * @tparam N Array length.
     * @param[in] array Array reference.
     */
    template <std::size_t N>
    constexpr ArrayView(T (&array)[N]) noexcept : data_(array), size_(N)  // NOLINT
    {
    }

    /** @brief Start pointer. */

    [[nodiscard]] constexpr T* data() const noexcept
    {
        return data_;
    }

    /** @brief Number of elements. */

    [[nodiscard]] constexpr size_type size() const noexcept
    {
        return size_;
    }

    /** @brief True when size()==0. */

    [[nodiscard]] constexpr bool empty() const noexcept
    {
        return size_ == 0;
    }

    /**
     * @brief Element access without bounds checking.
     *
     * @param[in] index Zero-based element index.
     */

    [[nodiscard]] constexpr T& operator[](size_type index) const noexcept
    {
        return data_[index];
    }

    /** @brief Begin iterator. */

    [[nodiscard]] constexpr T* begin() const noexcept
    {
        return data_;
    }

    /** @brief End iterator (one past the last element). */

    [[nodiscard]] constexpr T* end() const noexcept
    {
        return data_ + size_;
    }

    /**
     * @brief Create a subview.
     *
     * @param[in] offset First element offset.
     * @param[in] count Number of elements.
     * @warning Callers must keep the range inside this view.
     */

    [[nodiscard]] constexpr ArrayView Subview(size_type offset, size_type count) const noexcept
    {
        return ArrayView(data_ + offset, count);
    }

private:
    /** @brief Non-owning start pointer. */
    T* data_{nullptr};
    /** @brief Element count. */
    size_type size_{0};
};

}  // namespace common
}  // namespace navigation
