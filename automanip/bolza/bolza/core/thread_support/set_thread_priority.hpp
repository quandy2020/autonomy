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
 * @file set_thread_priority.hpp
 * @brief Sets the priority of the input thread.
 */


#include <pthread.h>
#include <iostream>
#include <thread>

namespace bolza {

/**
 * Sets the priority of the input thread.
 *
 * @param [in] priority: The priority of the thread from 0 (lowest) to 99 (highest)
 * @param [in] thread: A reference to the tread.
 */
inline void SetThreadPriority(int priority, pthread_t thread) {
  sched_param sched{};
  sched.sched_priority = priority;

  if (priority != 0) {
    if (pthread_setschedparam(thread, SCHED_FIFO, &sched) != 0) {
      std::cerr << "WARNING: Failed to set threads priority (one possible reason could be "
                   "that the user and the group permissions are not set properly.)"
                << std::endl;
    }
  }
}

/**
 * Sets the priority of the input thread.
 *
 * @param [in] priority: The priority of the thread from 0 (lowest) to 99 (highest)
 * @param [in] thread: A reference to the tread.
 */
inline void SetThreadPriority(int priority, std::thread& thread) {
  /**
   * @brief Constructs a SetThreadPriority.
   * @param [in] priority: Priority.
   */
  SetThreadPriority(priority, thread.native_handle());
}

/**
 * Sets the priority of the thread this function is called from.
 *
 * @param [in] priority: The priority of the thread from 0 (lowest) to 99 (highest)
 */
inline void SetThisThreadPriority(int priority) {
  /**
   * @brief Constructs a SetThreadPriority.
   * @param [in] priority: Priority.
   */
  SetThreadPriority(priority, pthread_self());
}

}  // namespace bolza
