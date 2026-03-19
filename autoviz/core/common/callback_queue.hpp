/*
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

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "websocket_logging.hpp"

namespace foxglove_ws {

class CallbackQueue {
public:
  CallbackQueue(LogCallback logCallback, size_t numThreads = 1)
      : _logCallback(logCallback)
      , _quit(false) {
    for (size_t i = 0; i < numThreads; ++i) {
      _workerThreads.push_back(std::thread(&CallbackQueue::doWork, this));
    }
  }

  ~CallbackQueue() {
    stop();
  }

  void stop() {
    _quit = true;
    _cv.notify_all();
    for (auto& thread : _workerThreads) {
      thread.join();
    }
  }

  void addCallback(std::function<void(void)> cb) {
    if (_quit) {
      return;
    }
    std::unique_lock<std::mutex> lock(_mutex);
    _callbackQueue.push_back(cb);
    _cv.notify_one();
  }

private:
  void doWork() {
    while (!_quit) {
      std::unique_lock<std::mutex> lock(_mutex);
      _cv.wait(lock, [this] {
        return (_quit || !_callbackQueue.empty());
      });
      if (_quit) {
        break;
      } else if (!_callbackQueue.empty()) {
        std::function<void(void)> cb = _callbackQueue.front();
        _callbackQueue.pop_front();
        lock.unlock();
        try {
          cb();
        } catch (const std::exception& ex) {
          // Should never get here if we catch all exceptions in the callbacks.
          const std::string msg =
            std::string("Caught unhandled exception in calback_queue") + ex.what();
          _logCallback(WebSocketLogLevel::Error, msg.c_str());
        } catch (...) {
          _logCallback(WebSocketLogLevel::Error, "Caught unhandled exception in calback_queue");
        }
      }
    }
  }

  LogCallback _logCallback;
  std::atomic<bool> _quit;
  std::mutex _mutex;
  std::condition_variable _cv;
  std::deque<std::function<void(void)>> _callbackQueue;
  std::vector<std::thread> _workerThreads;
};

}  // namespace foxglove_ws
