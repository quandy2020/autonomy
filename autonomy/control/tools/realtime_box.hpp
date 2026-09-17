// Ported into autonomy::control::tools
// Simple mutex-protected replacement for realtime_tools::RealtimeThreadSafeBox.

#pragma once

#include <mutex>
#include <optional>
#include <utility>

namespace autonomy {
namespace control {
namespace tools {

/**
 * \brief Mutex-protected value box used by Pid for non-RT gain updates.
 *
 * Non-realtime threads use get()/set(); the realtime loop uses try_get() so
 * it does not block if a writer holds the lock.
 */
template <typename T>
class RealtimeThreadSafeBox
{
public:
  RealtimeThreadSafeBox() = default;

  explicit RealtimeThreadSafeBox(const T & initial) : value_(initial) {}

  RealtimeThreadSafeBox(const RealtimeThreadSafeBox & other)
  {
    value_ = other.get();
  }

  RealtimeThreadSafeBox & operator=(const RealtimeThreadSafeBox & other)
  {
    if (this == &other) {
      return *this;
    }
    set(other.get());
    return *this;
  }

  T get() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  std::optional<T> try_get() const
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return std::nullopt;
    }
    return value_;
  }

  void set(const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
  }

  void set(T && value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
  }

private:
  mutable std::mutex mutex_;
  T value_{};
};

}  // namespace tools
}  // namespace control
}  // namespace autonomy
