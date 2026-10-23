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

#ifndef ATLAS_COMMON_MACROS_HPP_
#define ATLAS_COMMON_MACROS_HPP_

#include <memory>
#include <utility>

/**
 * @defgroup argus_macros Branch prediction and type helpers
 * @brief Common preprocessor utilities for Argus hot paths and shared
 * ownership.
 * @{
 */

/**
 * @brief Branch-prediction hint that @p x is usually true.
 * Expands to `__builtin_expect` on GCC/Clang; otherwise evaluates @p x as-is.
 * @param x Boolean-like expression.
 */
#if defined(__GNUC__) || defined(__clang__)
#define ATLAS_LIKELY(x) (__builtin_expect(!!(x), 1))
/**
 * @brief Branch-prediction hint that @p x is usually false.
 * @param x Boolean-like expression.
 */
#define ATLAS_UNLIKELY(x) (__builtin_expect(!!(x), 0))
#else
#define ATLAS_LIKELY(x) (x)
#define ATLAS_UNLIKELY(x) (x)
#endif

/**
 * @brief Assumed CPU cache-line size in bytes.
 * Override by defining `ATLAS_CACHELINE_SIZE` before including this header.
 */
#ifndef ATLAS_CACHELINE_SIZE
#define ATLAS_CACHELINE_SIZE 64
#endif

/** @brief Defines `SharedPtr`, `ConstSharedPtr`, and `make_shared` on @p Type
 * (internal). */
#define __ATLAS_SHARED_PTR_ALIAS(...)             \
  using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
  using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>

/** @brief Static `make_shared` factory for @p Type (used by @ref
 * ATLAS_SHARED_PTR_DEFINITIONS). */
#define __ATLAS_MAKE_SHARED_DEFINITION(...)                            \
  template <typename... Args>                                          \
  static std::shared_ptr<__VA_ARGS__> make_shared(Args&&... args) {    \
    return std::make_shared<__VA_ARGS__>(std::forward<Args>(args)...); \
  }

/**
 * @brief Inject `SharedPtr`, `ConstSharedPtr`, and `make_shared` into a class.
 * Expand inside the class body. Example: `ATLAS_SHARED_PTR_DEFINITIONS(Foo);`
 * @param Type Class name (for example `AbstractCamera`).
 */
#define ATLAS_SHARED_PTR_DEFINITIONS(...) \
  __ATLAS_SHARED_PTR_ALIAS(__VA_ARGS__);  \
  __ATLAS_MAKE_SHARED_DEFINITION(__VA_ARGS__)

/** @} */  // argus_macros

#endif  // ATLAS_COMMON_MACROS_HPP_
