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
 * @file joinable_thread.hpp
 * @brief RAII std::thread wrapper; C++17 substitute for std::jthread.
 */

#pragma once

#include <thread>
#include <utility>

namespace navigation
{
namespace common
{

/** @brief Thread that joins on destruction. */
class JoinableThread
{
public:
    /** @brief Construct an empty wrapper. */
    JoinableThread() = default;

    /**
     * @brief Start a thread from a callable.
     *
     * @param[in] function Callable executed on the new thread.
     * @param[in] args Arguments forwarded to the callable.
     */
    template <typename Function, typename... Args>
    explicit JoinableThread(Function&& function, Args&&... args)
        : thread_(std::forward<Function>(function), std::forward<Args>(args)...)
    {
    }

    /** @brief Join the thread if joinable. */
    ~JoinableThread()
    {
        Join();
    }

    JoinableThread(const JoinableThread&) = delete;
    JoinableThread& operator=(const JoinableThread&) = delete;

    JoinableThread(JoinableThread&& other) noexcept : thread_(std::move(other.thread_))
    {
    }

    JoinableThread& operator=(JoinableThread&& other) noexcept
    {
        if (this != &other)
        {
            Join();
            thread_ = std::move(other.thread_);
        }
        return *this;
    }

    /** @brief True while the underlying std::thread is joinable. */
    [[nodiscard]] bool Joinable() const
    {
        return thread_.joinable();
    }

    /** @brief Join the thread; no-op when already joined. */
    void Join()
    {
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    /** @brief Return the thread id. */
    [[nodiscard]] std::thread::id Id() const
    {
        return thread_.get_id();
    }

private:
    /** @brief Owned thread handle. */
    std::thread thread_;
};

}  // namespace common
}  // namespace navigation
