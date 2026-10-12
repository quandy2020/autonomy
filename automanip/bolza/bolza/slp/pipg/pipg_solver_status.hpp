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
 * @file pipg_solver_status.hpp
 * @brief Transforms pipg::SolverStatus to string.
 */


#include <string>

namespace bolza {
namespace pipg {

enum class SolverStatus {
  SUCCESS,
  MAX_ITER,
  UNDEFINED,
};

/**
 * @brief Transforms pipg::SolverStatus to string.
 * @param [in] s: S.
 * @return Result value.
 */
inline std::string ToString(SolverStatus s) {
  switch (s) {
    case SolverStatus::SUCCESS:
      /**
       * @brief String.
       * @return Result value.
       */
      return std::string("SUCCESS");
    case SolverStatus::MAX_ITER:
      /**
       * @brief String.
       * @return Result value.
       */
      return std::string("MAX_ITER");
    default:
      /**
       * @brief String.
       * @return Result value.
       */
      return std::string("UNDEFINED");
  }
}

}  // namespace pipg
}  // namespace bolza
