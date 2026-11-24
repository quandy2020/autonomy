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

/**
 * @file coverage_exception.hpp
 * @brief Coverage-stage error category.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_EXCEPTION_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_EXCEPTION_HPP_

#include <stdexcept>
namespace autonomy::task::coverage {
/**
 * @brief CoverageException with a diagnostic reason.
 */
class CoverageException : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_EXCEPTION_HPP_
