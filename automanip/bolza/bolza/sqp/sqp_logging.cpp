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

#include "bolza/sqp/sqp_logging.hpp"

#include <iomanip>

namespace bolza {
namespace sqp {

std::ostream& operator<<(std::ostream& stream, const LogEntry& logEntry) {
  const std::string delim = ", ";
  const std::string lineEnd = "\n";
  // clang-format off
  stream  << std::setprecision(16) // print decimals up to machine epsilon of double (~10^-16)
          << logEntry.problemNumber << delim
          << logEntry.time << delim
          << logEntry.iteration << delim
          << logEntry.linearQuadraticApproximationTime << delim
          << logEntry.solveQpTime << delim
          << logEntry.linesearchTime << delim
          << logEntry.baselinePerformanceIndex.merit << delim
          << logEntry.baselinePerformanceIndex.dynamicsViolationSSE << delim
          << logEntry.baselinePerformanceIndex.equalityConstraintsSSE << delim
          << logEntry.totalConstraintViolationBaseline << delim
          << logEntry.stepInfo.stepSize << delim
          << ToString(logEntry.stepInfo.stepType) << delim
          << logEntry.stepInfo.dx_norm << delim
          << logEntry.stepInfo.du_norm << delim
          << logEntry.stepInfo.performanceAfterStep.merit << delim
          << logEntry.stepInfo.performanceAfterStep.dynamicsViolationSSE << delim
          << logEntry.stepInfo.performanceAfterStep.equalityConstraintsSSE << delim
          << logEntry.stepInfo.totalConstraintViolationAfterStep << delim
          << ToString(logEntry.convergence) << lineEnd;
  // clang-format on
  return stream;
}

std::string LogHeader() {
  const std::string delim = ", ";
  const std::string lineEnd = "\n";
  std::stringstream stream;
  // clang-format off
  stream  << "problemNumber" << delim
          << "time" << delim
          << "iteration" << delim
          << "linearQuadraticApproximationTime" << delim
          << "solveQpTime" << delim
          << "linesearchTime" << delim
          << "baselinePerformanceIndex/merit" << delim
          << "baselinePerformanceIndex/dynamicsViolationSSE" << delim
          << "baselinePerformanceIndex/equalityConstraintsSSE" << delim
          << "totalConstraintViolationBaseline" << delim
          << "stepSize" << delim
          << "stepType" << delim
          << "dxNorm" << delim
          << "duNorm" << delim
          << "performanceAfterStep/merit" << delim
          << "performanceAfterStep/dynamicsViolationSSE" << delim
          << "performanceAfterStep/equalityConstraintsSSE" << delim
          << "totalConstraintViolationAfterStep" << delim
          << "convergence" << lineEnd;
  // clang-format on
  return stream.str();
}

}  // namespace sqp
}  // namespace bolza
