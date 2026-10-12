/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#pragma once

/**
 * @file synchronized.hpp
 * @brief Provides an interface to a unique_ptr<T> and a mutex.
 */


#include <memory>
#include <mutex>

namespace bolza {

/**
 * Provides an interface to a unique_ptr<T> and a mutex.
 * - The mutex is locked during construction and released during destruction.
 * - The object T can be called directly through this interface.
 * - The unique_ptr<T> can be swapped or reset.
 *
 * The user of this class has to make sure the unique_ptr<T> and the mutex outlive this interface.
 */
template <typename T>
class LockedPtr {
 public:
  /**
   * @brief Constructs a LockedPtr.
   * @param [in,out] p: P.
   * @param [in] m: M.
   */
  LockedPtr(std::unique_ptr<T>& p, std::mutex& m) : p_(p), lk_(m) {}
  /**
   * @brief Constructs a LockedPtr.
   * @param [in,out] p: P.
   * @param [in,out] m: M.
   * @param [in] adopt_lock: Adopt lock.
   */
  LockedPtr(std::unique_ptr<T>& p, std::mutex& m, std::adopt_lock_t) : p_(p), lk_(m, std::adopt_lock) {}

  /// Access
  T* operator->() { /**
  * @brief Get.
  * @return Result value.
  */
 return p_.get(); }
  const T* operator->() const { /**
  * @brief Get.
  * @return Result value.
  */
 return p_.get(); }
  T& operator*() { return *p_; }
  const T& operator*() const { return *p_; }

  /// Resets the wrapped unique ptr.
  /**
   * @brief Reset.
   * @param [in] p: P.
   */
  void Reset(std::unique_ptr<T> p) noexcept { /**
  * @brief Move.
  * @param [in] p: P.
  * @return Result value.
  */
 p_ = std::move(p); }

  /// Swaps the wrapped unique ptr.
  /**
   * @brief Swap.
   * @param [in,out] p: P.
   */
  void swap(std::unique_ptr<T>& p) noexcept { /**
  * @brief Swap.
  * @param [in] p: P.
  * @return Result value.
  */
 p_.swap(p); }

  /// Returns true if the wrapped unique ptr is not null.
  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  std::unique_ptr<T>& p_;
  std::unique_lock<std::mutex> lk_;
};

/**
 * Provides a const interface to a pointer T* and a mutex.
 * - The mutex is locked during construction and released during destruction.
 * - const methods on T* can be called directly through this interface.
 *
 * The user of this class has to make sure the original object T and the mutex outlive this interface.
 */
template <typename T>
class LockedConstPtr {
 public:
  /**
   * @brief Constructs a LockedConstPtr.
   * @param [in] p: P.
   * @param [in] m: M.
   */
  LockedConstPtr(const T* p, std::mutex& m) : p_(p), lk_(m) {}
  /**
   * @brief Constructs a LockedConstPtr.
   * @param [in] p: P.
   * @param [in,out] m: M.
   * @param [in] adopt_lock: Adopt lock.
   */
  LockedConstPtr(const T* p, std::mutex& m, std::adopt_lock_t) : p_(p), lk_(m, std::adopt_lock) {}

  /// const access
  const T* operator->() const { return p_; }
  const T& operator*() const { return *p_; }

  /// Returns true if the wrapped pointer is not null.
  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  const T* p_;
  std::unique_lock<std::mutex> lk_;
};

/**
 * This class wraps a std::unique_ptr<T> with a mutex and provides an interface that automatically locks the mutex.
 *
 * - Calling the wrapped object:
 * 1) The operator->() of this class can be used to directly call a member function of T while holding the lock.
 * 2) lock() will return a custom pointer (LockedPtr/LockedConstPtr) to the wrapped object that holds the lock for the lifetime of that
 * pointer. This is useful if the lock should be held during multiple member function calls.
 *
 * - Wrapping a new object:
 * The wrapped object can be swapped or reset either directly through the Synchronized<T> or through a LockedPtr.
 *
 * - Locking multiple objects:
 * A helper function "synchronizeLock" is available to lock multiple Synchronized<T>s at the same time, similar to std::lock(.., ..)
 */
template <typename T>
class Synchronized {
 public:
  /// Default constructor : Owns nothing
  /**
   * @brief Constructs a Synchronized.
   */
  Synchronized() = default;

  /// Construct from unique_ptr
  /**
   * @brief Constructs a Synchronized.
   */
  explicit Synchronized(std::unique_ptr<T> p) noexcept : p_(std::move(p)) {}

  /// Destructor : Obtains the lock and destroys the owned object.
  /**
   * @brief Destructor.
   */
  ~Synchronized() {
    /**
     * @brief Lk.
     * @param [in] m_: M.
     * @return Result value.
     */
    std::unique_lock<std::mutex> lk(m_);
    /**
     * @brief Reset.
     * @return Result value.
     */
    p_.reset();
  }

  // Disable copy operations
  /**
   * @brief Constructs a Synchronized.
   */
  Synchronized(const Synchronized&) = delete;
  Synchronized& operator=(const Synchronized&) = delete;

  /// Move construction
  /**
   * @brief Constructs a Synchronized.
   * @param [in,out] other: Other.
   */
  Synchronized(Synchronized&& other) noexcept {
    // Current object is being created, no need to lock this->m_
    /**
     * @brief Lk Other.
     * @param [in] m_: M.
     * @return Result value.
     */
    std::lock_guard<std::mutex> LkOther(other.m_);
    /**
     * @brief Move.
     * @param [in] p_: P.
     * @return Result value.
     */
    p_ = std::move(other.p_);
  }

  /// Move assignment
  Synchronized& operator=(Synchronized&& other) noexcept {
    // Both other and this object already exist. Need to lock both
    /**
     * @brief Lk This.
     * @param [in] m_: M.
     * @param [in] defer_lock: Defer lock.
     * @return Result value.
     */
    std::unique_lock<std::mutex> lkThis(m_, std::defer_lock);
    /**
     * @brief Lk Other.
     * @param [in] m_: M.
     * @param [in] defer_lock: Defer lock.
     * @return Result value.
     */
    std::unique_lock<std::mutex> lkOther(other.m_, std::defer_lock);
    /**
     * @brief Lock.
     * @param [in] lkOther: Lk Other.
     * @param [in] lkThis: Lk This.
     * @return Result value.
     */
    std::lock(lkOther, lkThis);
    /**
     * @brief Swap.
     * @param [in] p_: P.
     * @return Result value.
     */
    p_.swap(other.p_);
    return *this;
  }

  /// Reset the wrapped object. The object is reseated while holding the lock.
  /**
   * @brief Reset.
   * @param [in] p: P.
   */
  void Reset(std::unique_ptr<T> p) noexcept {
    /**
     * @brief Swap.
     * @param [in] p: P.
     */
    swap(p);  // by using swap, the old object is destroyed after the lock is released
  }

  /// Swaps the wrapped object. The object is swapped while holding the lock.
  /**
   * @brief Swap.
   * @param [in,out] p: P.
   */
  void swap(std::unique_ptr<T>& p) noexcept {
    /**
     * @brief Lk.
     * @param [in] m_: M.
     * @return Result value.
     */
    std::lock_guard<std::mutex> lk(m_);
    /**
     * @brief Swap.
     * @param [in] p: P.
     * @return Result value.
     */
    p_.swap(p);
  }

  /// Returns a pointer that holds the lock to the wrapped object. Lifetime of the LockedPtr<> determines the lifetime of the lock.
  /**
   * @brief Lock.
   * @return Result value.
   */
  LockedPtr<T> lock() { return {p_, m_}; }
  /**
   * @brief Lock.
   * @return Result value.
   */
  LockedConstPtr<T> lock() const { return {p_.get(), m_}; }

  /// Returns a pointer that adopts and holds the lock to the wrapped object. Lifetime of the LockedPtr<> determines the lifetime of the
  /// lock.
  /**
   * @brief Adopt Lock.
   * @return Result value.
   */
  LockedPtr<T> AdoptLock() { return {p_, m_, std::adopt_lock}; }
  /**
   * @brief Adopt Lock.
   * @return Result value.
   */
  LockedConstPtr<T> AdoptLock() const { return {p_.get(), m_, std::adopt_lock}; }

  /// Apply operations directly on the held object. Holds the lock for the duration of the call.
  LockedPtr<T> operator->() { /**
  * @brief Lock.
  * @return Result value.
  */
 return this->lock(); }
  LockedConstPtr<T> operator->() const { /**
  * @brief Lock.
  * @return Result value.
  */
 return this->lock(); }

  /// Get direct access to the internal mutex.
  /**
   * @brief Get Mutex.
   * @return Result value.
   */
  std::mutex& GetMutex() const noexcept { return m_; }

 private:
  mutable std::mutex m_;
  std::unique_ptr<T> p_;
};

/**
 * Helper function to lock multiple synchronized<T> objects simultaneously, preventing deadlocks.
 * @return a tuple of LockedPtr<T> / LockedConstPtr<T>.
 */
template <typename... SV>
auto SynchronizeLock(SV&... sv) -> std::tuple<decltype(sv.adoptLock())...> {
  /**
   * @brief Lock.
   * @return Result value.
   */
  std::lock(sv.getMutex()...);
  using tuple_t = std::tuple<decltype(sv.adoptLock())...>;
  /**
   * @brief Tuple t.
   * @return Result value.
   */
  return tuple_t(sv.adoptLock()...);
}

}  // namespace bolza
