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

#include "bolza/mpcnet/rollout/mpcnet_data_generation.hpp"

#include <random>

#include "bolza/mpcnet/control/mpcnet_behavioral_controller.hpp"

namespace bolza {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const data_array_t* MPCNetDataGeneration::Run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation,
                                              size_t nSamples, const matrix_t& samplingCovariance,
                                              const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
                                              const TargetTrajectories& targetTrajectories) {
  // clear data array
  dataArray_.clear();

  // set system
  set(alpha, policyFilePath, initialObservation, modeSchedule, targetTrajectories);

  // set up scalar standard normal generator and compute Cholesky decomposition of covariance matrix
  std::random_device randomDevice;
  std::default_random_engine pseudoRandomNumberGenerator(randomDevice());
  std::normal_distribution<scalar_t> standardNormalDistribution(scalar_t(0.0), scalar_t(1.0));
  auto standardNormalNullaryOp = [&](scalar_t) -> scalar_t { return standardNormalDistribution(pseudoRandomNumberGenerator); };
  const matrix_t L = samplingCovariance.llt().matrixL();

  // run data generation
  int iteration = 0;
  try {
    while (systemObservation_.time <= targetTrajectories.timeTrajectory.back()) {
      // step system
      Step(timeStep);

      // downsample the data signal by an integer factor
      if (iteration % dataDecimation == 0) {
        // get nominal data point
        const vector_t deviation = vector_t::Zero(primalSolution_.stateTrajectory_.front().size());
        dataArray_.push_back(GetDataPoint(*mpcPtr_, *mpcnetDefinitionPtr_, deviation));

        // get samples around nominal data point
        for (int i = 0; i < nSamples; i++) {
          const vector_t deviation = L * vector_t::NullaryExpr(primalSolution_.stateTrajectory_.front().size(), standardNormalNullaryOp);
          dataArray_.push_back(GetDataPoint(*mpcPtr_, *mpcnetDefinitionPtr_, deviation));
        }
      }

      // update iteration
      ++iteration;
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "[MPCNetDataGeneration::Run] a standard exception was caught, with message: " << e.what() << "\n";
    // this data generation run failed, clear data
    dataArray_.clear();
  }

  // return pointer to the data array
  return &dataArray_;
}

}  // namespace mpcnet
}  // namespace bolza
