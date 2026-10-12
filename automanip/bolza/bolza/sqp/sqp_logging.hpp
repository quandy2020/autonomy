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
 * @file sqp_logging.hpp
 * @brief sqp logging (bolza).
 */


#include <ostream>
#include <string>

#include <bolza/core/types.hpp>

#include <bolza/oc/oc_data/performance_index.hpp>

#include "bolza/sqp/sqp_solver_status.hpp"

namespace bolza {
namespace sqp {

struct LogEntry {
  // Problem info
  size_t problemNumber = 0;
  scalar_t time = 0.0;  // horizon start time for the currently solved problem

  // Iteration info
  size_t iteration = 0;

  // Computation time
  scalar_t linearQuadraticApproximationTime = 0.0;
  scalar_t solveQpTime = 0.0;
  scalar_t linesearchTime = 0.0;

  // Line search
  PerformanceIndex baselinePerformanceIndex;  // before taking the step
  scalar_t totalConstraintViolationBaseline;  // constraint metric used in the line search
  StepInfo stepInfo;

  // Convergence
  Convergence convergence = Convergence::FALSE;
};

std::ostream& operator<<(std::ostream& stream, const LogEntry& logEntry);

/**
 * @brief Log Header.
 * @return Result value.
 */
std::string LogHeader();

template <typename T>
class Logger {
 public:
  /**
   * @brief Constructs a Logger.
   * @param [in] numberOfEntries: Number Of Entries.
   */
  explicit Logger(size_t numberOfEntries);

  /**
   * @brief Current Entry.
   * @return Result value.
   */
  T& CurrentEntry();

  /**
   * @brief Advance.
   */
  void Advance();

  /**
   * @brief Write.
   * @param [in,out] stream: Stream.
   */
  void write(std::ostream& stream) const;

 private:
  using const_iterator = typename std::vector<T>::const_iterator;
  using iterator = typename std::vector<T>::iterator;

  bool full_;
  const_iterator front_;
  iterator back_;
  std::vector<T> data_;
};

template <typename T>
Logger<T>::Logger(size_t numberOfEntries)
    : full_(false),
      data_(numberOfEntries + 1, T())  // +1 so we have space for a new element to write to when full.
{
  front_ = data_.begin();
  back_ = data_.begin();
}

template <typename T>
/**
 * @brief Current Entry.
 * @return Result value.
 */
T& Logger<T>::CurrentEntry() {
  return *back_;
}

template <typename T>
/**
 * @brief Advance.
 * @return Result value.
 */
void Logger<T>::Advance() {
  // move current entry iterator
  ++back_;

  // Wrap back pointer
  if (back_ == data_.end()) {
    back_ = data_.begin();
    full_ = true;
  }

  // Move (+wrap) front point
  if (full_) {
    ++front_;
    if (front_ == data_.end()) {
      front_ = data_.begin();
    }
  }
}

template <typename T>
/**
 * @brief Write.
 * @param [in,out] stream: Stream.
 * @return Result value.
 */
void Logger<T>::write(std::ostream& stream) const {
  const_iterator writeIt = front_;

  if (writeIt > back_) {
    // write front -> end
    while (writeIt != data_.end()) {
      stream << *writeIt;
      ++writeIt;
    }
    // wrap around
    writeIt = data_.cbegin();
  }

  // write front -> back
  while (writeIt < back_) {
    stream << *writeIt;
    ++writeIt;
  }
}

}  // namespace sqp
}  // namespace bolza