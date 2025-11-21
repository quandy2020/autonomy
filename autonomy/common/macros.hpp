// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <utility>
#include <mutex>
#include <type_traits>

#ifndef AUTONOMY_COMMON__MACROS_HPP_
#define AUTONOMY_COMMON__MACROS_HPP_

/**
 * Disables the copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define AUTONOMY_DISABLE_COPY(...) \
  __VA_ARGS__(const __VA_ARGS__ &) = delete; \
  __VA_ARGS__ & operator=(const __VA_ARGS__ &) = delete;

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define AUTONOMY_SMART_PTR_DEFINITIONS(...) \
  AUTONOMY_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  AUTONOMY_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  AUTONOMY_UNIQUE_PTR_DEFINITIONS(__VA_ARGS__)

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
#define AUTONOMY_SMART_PTR_DEFINITIONS_NOT_COPYABLE(...) \
  AUTONOMY_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  AUTONOMY_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  __AUTONOMY_UNIQUE_PTR_ALIAS(__VA_ARGS__)

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
#define AUTONOMY_SMART_PTR_ALIASES_ONLY(...) \
  __AUTONOMY_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __AUTONOMY_WEAK_PTR_ALIAS(__VA_ARGS__) \
  __AUTONOMY_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  __AUTONOMY_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __AUTONOMY_SHARED_PTR_ALIAS(...) \
  using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
  using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define __AUTONOMY_MAKE_SHARED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> \
  make_shared(Args && ... args) \
  { \
    return std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
  }

/// Defines aliases and static functions for using the Class with shared_ptrs.
#define AUTONOMY_SHARED_PTR_DEFINITIONS(...) \
  __AUTONOMY_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __AUTONOMY_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __AUTONOMY_WEAK_PTR_ALIAS(...) \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

/// Defines aliases and static functions for using the Class with weak_ptrs.
#define AUTONOMY_WEAK_PTR_DEFINITIONS(...) __AUTONOMY_WEAK_PTR_ALIAS(__VA_ARGS__)

#define __AUTONOMY_UNIQUE_PTR_ALIAS(...) using UniquePtr = std::unique_ptr<__VA_ARGS__>;

#define __AUTONOMY_MAKE_UNIQUE_DEFINITION(...) \
  template<typename ... Args> \
  static std::unique_ptr<__VA_ARGS__> \
  make_unique(Args && ... args) \
  { \
    return std::unique_ptr<__VA_ARGS__>(new __VA_ARGS__(std::forward<Args>(args) ...)); \
  }

/// Defines aliases and static functions for using the Class with unique_ptrs.
#define AUTONOMY_UNIQUE_PTR_DEFINITIONS(...) \
  __AUTONOMY_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  __AUTONOMY_MAKE_UNIQUE_DEFINITION(__VA_ARGS__)

#define AUTONOMY_STRING_JOIN(arg1, arg2) AUTONOMY_DO_STRING_JOIN(arg1, arg2)
#define AUTONOMY_DO_STRING_JOIN(arg1, arg2) arg1 ## arg2


#define CACHELINE_SIZE 64

#define DEFINE_TYPE_TRAIT(name, func)                     \
  template <typename T>                                   \
  struct name {                                           \
    template <typename Class>                             \
    static constexpr bool Test(decltype(&Class::func)*) { \
      return true;                                        \
    }                                                     \
    template <typename>                                   \
    static constexpr bool Test(...) {                     \
      return false;                                       \
    }                                                     \
                                                          \
    static constexpr bool value = Test<T>(nullptr);       \
  };                                                      \
                                                          \
  template <typename T>                                   \
  constexpr bool name<T>::value;


namespace autonomy {

inline void cpu_relax() 
{
#if defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#else
  asm volatile("rep; nop" ::: "memory");
#endif
}

// inline void* CheckedMalloc(size_t size) 
// {
//   void* ptr = std::malloc(size);
//   if (!ptr) {
//     throw std::bad_alloc();
//   }
//   return ptr;
// }

// inline void* CheckedCalloc(size_t num, size_t size) 
// {
//   void* ptr = std::calloc(num, size);
//   if (!ptr) {
//     throw std::bad_alloc();
//   }
//   return ptr;
// }

DEFINE_TYPE_TRAIT(HasShutdown, Shutdown)

template <typename T>
typename std::enable_if<HasShutdown<T>::value>::type CallShutdown(T *instance) {
  instance->Shutdown();
}

template <typename T>
typename std::enable_if<!HasShutdown<T>::value>::type CallShutdown(T *instance) {
  (void)instance;
}

}  // namespace autonomy

// There must be many copy-paste versions of these macros which are same
// things, undefine them to avoid conflict.
#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN

#define UNUSED(param) (void)param

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
  static void CleanUp() {                                                 \
    auto instance = Instance(false);                                      \
    if (instance != nullptr) {                                            \
      CallShutdown(instance);                                             \
    }                                                                     \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)



#endif  // AUTONOMY_COMMON


