/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

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

// SharedPtr / make_shared aliases for action Server, Client, etc.
#define __AUTOLINK_SHARED_PTR_ALIAS(...)            \
    using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
    using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define __AUTOLINK_MAKE_SHARED_DEFINITION(...)                             \
    template <typename... Args>                                            \
    static std::shared_ptr<__VA_ARGS__> make_shared(Args&&... args) {      \
        return std::make_shared<__VA_ARGS__>(std::forward<Args>(args)...); \
    }

#define AUTOLINK_SHARED_PTR_DEFINITIONS(...) \
    __AUTOLINK_SHARED_PTR_ALIAS(__VA_ARGS__) \
    __AUTOLINK_MAKE_SHARED_DEFINITION(__VA_ARGS__)

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
