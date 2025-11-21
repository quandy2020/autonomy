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

#include <iostream>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

#include "autolink/base/macros.hpp"

DEFINE_TYPE_TRAIT(HasShutdown, Shutdown)

template <typename T>
typename std::enable_if<HasShutdown<T>::value>::type CallShutdown(T* instance) {
    instance->Shutdown();
}

template <typename T>
typename std::enable_if<!HasShutdown<T>::value>::type CallShutdown(
    T* instance) {
    (void)instance;
}

// There must be many copy-paste versions of these macros which are same
// things, undefine them to avoid conflict.
#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN

#define UNUSED(param) (void)param

#define DISALLOW_COPY_AND_ASSIGN(classname) \
    classname(const classname&) = delete;   \
    classname& operator=(const classname&) = delete;

#define DECLARE_SINGLETON(classname)                                       \
public:                                                                    \
    static classname* Instance(bool create_if_needed = true) {             \
        static classname* instance = nullptr;                              \
        if (!instance && create_if_needed) {                               \
            static std::once_flag flag;                                    \
            std::call_once(                                                \
                flag, [&] { instance = new (std::nothrow) classname(); }); \
        }                                                                  \
        return instance;                                                   \
    }                                                                      \
                                                                           \
    static void CleanUp() {                                                \
        auto instance = Instance(false);                                   \
        if (instance != nullptr) {                                         \
            CallShutdown(instance);                                        \
        }                                                                  \
    }                                                                      \
                                                                           \
private:                                                                   \
    classname();                                                           \
    DISALLOW_COPY_AND_ASSIGN(classname)

/**
 * Disables the copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define AUTOLINK_DISABLE_COPY(...)            \
    __VA_ARGS__(const __VA_ARGS__&) = delete; \
    __VA_ARGS__& operator=(const __VA_ARGS__&) = delete;

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define AUTOLINK_SMART_PTR_DEFINITIONS(...)      \
    AUTOLINK_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
    AUTOLINK_WEAK_PTR_DEFINITIONS(__VA_ARGS__)   \
    AUTOLINK_UNIQUE_PTR_DEFINITIONS(__VA_ARGS__)

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Same as AUTONOMY_SMART_PTR_DEFINITIONS except it excludes the static
 * Class::make_unique() method definition which does not work on classes which
 * are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define AUTOLINK_SMART_PTR_DEFINITIONS_NOT_COPYABLE(...) \
    AUTOLINK_SHARED_PTR_DEFINITIONS(__VA_ARGS__)         \
    AUTOLINK_WEAK_PTR_DEFINITIONS(__VA_ARGS__)           \
    __AUTOLINK_UNIQUE_PTR_ALIAS(__VA_ARGS__)

/**
 * Defines aliases only for using the Class with smart pointers.
 *
 * Same as AUTONOMY_SMART_PTR_DEFINITIONS except it excludes the static
 * method definitions which do not work on pure virtual classes and classes
 * which are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define AUTOLINK_SMART_PTR_ALIASES_ONLY(...) \
    __AUTOLINK_SHARED_PTR_ALIAS(__VA_ARGS__) \
    __AUTOLINK_WEAK_PTR_ALIAS(__VA_ARGS__)   \
    __AUTOLINK_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
    __AUTOLINK_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __AUTOLINK_SHARED_PTR_ALIAS(...)            \
    using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
    using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define __AUTOLINK_MAKE_SHARED_DEFINITION(...)                             \
    template <typename... Args>                                            \
    static std::shared_ptr<__VA_ARGS__> make_shared(Args&&... args) {      \
        return std::make_shared<__VA_ARGS__>(std::forward<Args>(args)...); \
    }

/// Defines aliases and static functions for using the Class with shared_ptrs.
#define AUTOLINK_SHARED_PTR_DEFINITIONS(...) \
    __AUTOLINK_SHARED_PTR_ALIAS(__VA_ARGS__) \
    __AUTOLINK_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __AUTOLINK_WEAK_PTR_ALIAS(...)          \
    using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
    using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

/// Defines aliases and static functions for using the Class with weak_ptrs.
#define AUTOLINK_WEAK_PTR_DEFINITIONS(...) \
    __AUTOLINK_WEAK_PTR_ALIAS(__VA_ARGS__)

#define __AUTOLINK_UNIQUE_PTR_ALIAS(...) \
    using UniquePtr = std::unique_ptr<__VA_ARGS__>;

#define __AUTOLINK_MAKE_UNIQUE_DEFINITION(...)                        \
    template <typename... Args>                                       \
    static std::unique_ptr<__VA_ARGS__> make_unique(Args&&... args) { \
        return std::unique_ptr<__VA_ARGS__>(                          \
            new __VA_ARGS__(std::forward<Args>(args)...));            \
    }

/// Defines aliases and static functions for using the Class with unique_ptrs.
#define AUTOLINK_UNIQUE_PTR_DEFINITIONS(...) \
    __AUTOLINK_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
    __AUTOLINK_MAKE_UNIQUE_DEFINITION(__VA_ARGS__)

#define AUTOLINK_STRING_JOIN(arg1, arg2) AUTOLINK_DO_STRING_JOIN(arg1, arg2)
#define AUTOLINK_DO_STRING_JOIN(arg1, arg2) arg1##arg2
