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
 * @file riccati_modification.hpp
 * @brief The struct contains Riccati equation modification terms.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace riccati_modification {

/**
 * The struct contains Riccati equation modification terms.
 */
struct Data {
  scalar_t time_ = 0.0;

  matrix_t deltaQm_;
  matrix_t deltaGm_;
  vector_t deltaGv_;

  /** The Hessian matrix of the Hamiltonian, \f$Hm\f$. */
  matrix_t hamiltonianHessian_;
  /** The right pseudo-inverse of \f$Dm\f$ */
  matrix_t constraintRangeProjector_;
  /** \f$DmNull inv(DmNull^T * Hm * DmNull) * DmNull^T = (I - invHm * Dm^T * inv(Dm * invHm * Dm^T) * Dm) * invHm\f$ */
  matrix_t constraintNullProjector_;
};

/**
 * @brief /**.
 *
 * Displays all variables
 *
 * /
 * @param [in] data: Data.
 */
void Display(const Data& data);

}  // namespace riccati_modification
}  // namespace bolza
