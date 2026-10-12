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

#include <iostream>

#include "bolza/ddp/riccati_equations/riccati_modification.hpp"

namespace bolza {
namespace riccati_modification {

/**
 * Displays all variables
 */
void Display(const Data& data) {
  std::cerr << std::endl;
  std::cerr << "time: " << data.time_ << "\n";
  std::cerr << "deltaQm:\n" << data.deltaQm_ << "\n";
  std::cerr << "deltaRm:\n" << data.deltaGm_ << "\n";
  std::cerr << "deltaPm:\n" << data.deltaGv_.transpose() << "\n";

  std::cerr << "constraintRangeProjector:\n" << data.constraintRangeProjector_ << "\n";
  std::cerr << "constraintNullProjector: \n" << data.constraintNullProjector_ << "\n";
  std::cerr << std::endl;
}

}  // namespace riccati_modification
}  // namespace bolza
