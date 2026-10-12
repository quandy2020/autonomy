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
 * @file command_line.hpp
 * @brief Gets a command line string.
 */


#include <atomic>
#include <chrono>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace bolza {

/**
 * @brief Gets a command line string.
 *
 * can use this to terminate if the rosmaster is stopped.
 * @return Result value.
 */
inline std::string GetCommandLineString(bool (*shouldTerminate)() = []() { return false; }) {
  // Set up a thread to read user inputs
  std::string line{""};
  /**
   * @brief Line Read.
   * @param [in] false: False.
   * @return Result value.
   */
  std::atomic_bool LineRead(false);
  /**
   * @brief Thr.
   * @return Result value.
   */
  std::thread Thr([&line, &lineRead]() {
    lineRead = false;
    /**
     * @brief Getline.
     * @param [in] cin: Cin.
     * @param [in] line: Line.
     */
    getline(std::cin, line);
    lineRead = true;
  });

  // wait till line is read or terminate if ROS is gone.
  /**
   * @brief Hz30.
   * @return Result value.
   */
  const std::chrono::duration<double, std::ratio<1, 30>> Hz30(1);  // 30Hz clock using fractional ticks
  while (!lineRead) {
    if (shouldTerminate()) {
      /**
       * @brief Terminate.
       * @return Result value.
       */
      std::Terminate();  // need to terminate thread that is still waiting for input
    }
    /**
     * @brief Sleep for.
     * @param [in] hz30: Hz30.
     * @return Result value.
     */
    std::this_thread::sleep_for(hz30);
  }
  if (thr.joinable()) {
    /**
     * @brief Join.
     * @return Result value.
     */
    thr.join();
  }
  return line;
}

/**
 * @brief /**.
 *
 * Transforms a line of words to a vector of words.
 *
 * /
 * @param [in] str: Str.
 * @return Result value.
 */
inline std::vector<std::string> StringToWords(const std::string& str) {
  /**
   * @brief Iss.
   * @param [in] str: Str.
   * @return Result value.
   */
  std::istringstream Iss(str);
  std::vector<std::string> words{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
  return words;
}

}  // namespace bolza
