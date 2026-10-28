#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace autonomy::localization::atlas {

/**
 * @brief Thread-safe FIFO with optional bounded capacity and close semantics.
 * @tparam T Movable value stored by the queue.
 *
 * Producers block while a bounded queue is full. Consumers block while it is
 * empty. Close() wakes both groups; queued values remain available to Pop().
 */
template <typename T>
class BlockingQueue
{
public:
    /** @param capacity Maximum size, or zero for an unbounded queue. */
    explicit BlockingQueue(std::size_t capacity = 0) : capacity_(capacity) {}

    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;

    /** @return false if the queue was closed before insertion. */
    bool Push(const T& value) {
        return PushImpl(value);
    }
    /** @copydoc Push(const T&) */
    bool Push(T&& value) {
        return PushImpl(std::move(value));
    }

    /** @return false for a null output or a closed, drained queue. */
    bool Pop(T* value) {
        if (value == nullptr)
            return false;
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this] { return closed_ || !queue_.empty(); });
        if (queue_.empty())
            return false;
        *value = std::move(queue_.front());
        queue_.pop_front();
        not_full_.notify_one();
        return true;
    }

    /** @brief Removes a value without waiting. */
    bool TryPop(T* value) {
        if (value == nullptr)
            return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return false;
        *value = std::move(queue_.front());
        queue_.pop_front();
        not_full_.notify_one();
        return true;
    }

    /** @brief Prevents future pushes and wakes all waiting threads. */
    void Close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        not_empty_.notify_all();
        not_full_.notify_all();
    }

    /** @return Whether Close() has been called. */
    bool closed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_;
    }

    /** @return Current number of queued values. */
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    template <typename U>
    bool PushImpl(U&& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [this] {
            return closed_ || capacity_ == 0 || queue_.size() < capacity_;
        });
        if (closed_)
            return false;
        queue_.emplace_back(std::forward<U>(value));
        not_empty_.notify_one();
        return true;
    }

    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    std::deque<T> queue_;
    bool closed_ = false;
};

}  // namespace autonomy::localization::atlas
