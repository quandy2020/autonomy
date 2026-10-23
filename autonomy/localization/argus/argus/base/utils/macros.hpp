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

/**
 * @file macros.hpp
 * @brief Minimal branch/cacheline helpers (from autolink/base/macros, trimmed).
 */

#ifndef ARGUS_COMMON_MACROS_HPP_
#define ARGUS_COMMON_MACROS_HPP_

#include <memory>
#include <utility>

/**
 * @defgroup argus_macros Branch prediction and type helpers
 * @brief Common preprocessor utilities for Argus hot paths and shared
 * ownership.
 * @{
 */

/** @brief Hint that @p x is usually true (GCC/Clang `__builtin_expect`; no-op
 * elsewhere). */
#if defined(__GNUC__) || defined(__clang__)
#define ARGUS_LIKELY(x) (__builtin_expect(!!(x), 1))
/** @brief Hint that @p x is usually false. */
#define ARGUS_UNLIKELY(x) (__builtin_expect(!!(x), 0))
#else
#define ARGUS_LIKELY(x) (x)
#define ARGUS_UNLIKELY(x) (x)
#endif

/** @brief Assumed CPU cache line size in bytes (override before include if
 * needed). */
#ifndef ARGUS_CACHELINE_SIZE
#define ARGUS_CACHELINE_SIZE 64
#endif

/** @brief Defines `SharedPtr`, `ConstSharedPtr`, and `make_shared` on @p Type
 * (internal). */
#define __ARGUS_SHARED_PTR_ALIAS(...)               \
    using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
    using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>

/** @brief Static `make_shared` factory for @p Type (used by @ref
 * ARGUS_SHARED_PTR_DEFINITIONS). */
#define __ARGUS_MAKE_SHARED_DEFINITION(...)                                \
    template <typename... Args>                                            \
    static std::shared_ptr<__VA_ARGS__> make_shared(Args&&... args) {      \
        return std::make_shared<__VA_ARGS__>(std::forward<Args>(args)...); \
    }

/**
 * @brief Inject standard shared-pointer typedefs and `make_shared` into a class
 * body.
 * @param Type Class name (e.g. `AbstractCamera`).
 */
#define ARGUS_SHARED_PTR_DEFINITIONS(...)  \
    __ARGUS_SHARED_PTR_ALIAS(__VA_ARGS__); \
    __ARGUS_MAKE_SHARED_DEFINITION(__VA_ARGS__)

/** @} */  // argus_macros

#endif  // ARGUS_COMMON_MACROS_HPP_
