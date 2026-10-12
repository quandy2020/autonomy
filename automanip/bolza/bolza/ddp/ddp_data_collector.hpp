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
 * @file ddp_data_collector.hpp
 * @brief ddp_data_collector.hpp.
 */


#include <bolza/core/types.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>

#include "gauss_newton_ddp.hpp"

namespace bolza {

/**
 * Collects the required data from DDP instance. It uses swap method wherever it is possible.
 */
class DDPDataCollector {
 public:
  /**
   * Default constructor.
   */
  DDPDataCollector() = default;

  /**
   * Constructor.
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDynamicsPtr: The system dynamics and derivatives for the subsystems.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and final costs) and its derivatives for subsystems.
   */
  DDPDataCollector(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                    const CostFunctionBase* costFunctionPtr);

  /**
   * Default destructor.
   */
  ~DDPDataCollector() = default;

  /**
   * @brief Collects the required data from DDP instance. It uses swap method wherever it is possible.
   * @param [in] constDdpPtr: A pointer to the DDP instance.
   */
  void Collect(const GaussNewtonDDP* constDdpPtr);

  scalar_t initTime_ = 0.0;
  scalar_t finalTime_ = 0.0;
  vector_t initState_;

  size_t initActivePartition_ = 0;
  size_t finalActivePartition_ = 0;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_;

  unsigned long long int rewindCounter_ = 0;

  ModeSchedule modeSchedule_;

  std::vector<LinearController> optimizedControllersStock_;

  scalar_array2_t nominalTimeTrajectoriesStock_;
  size_array2_t nominalPostEventIndicesStock_;
  vector_array2_t nominalStateTrajectoriesStock_;
  vector_array2_t nominalInputTrajectoriesStock_;

  // model data trajectory
  std::vector<std::vector<ModelData>> modelDataTrajectoriesStock_;

  // event times model data
  std::vector<std::vector<ModelData>> modelDataEventTimesStock_;

  // projected model data trajectory
  std::vector<std::vector<ModelData>> projectedModelDataTrajectoriesStock_;

  // Riccati modification
  std::vector<std::vector<riccati_modification::Data>> riccatiModificationTrajectoriesStock_;

  // TODO(mspieler): delete this
  //  matrix_array2_t RmInverseTrajectoriesStock_;
  //  matrix_array2_t RmInvConstrainedCholTrajectoryStock_;
  //  matrix_array2_t DmDagerTrajectoriesStock_;

  // final cost which is interpreted as the Heuristic function
  ScalarFunctionQuadraticApproximation heuristics_;

  scalar_array2_t SsTimeTrajectoriesStock_;
  scalar_array2_t SsNormalizedTimeTrajectoriesStock_;
  size_array2_t SsNormalizedEventsPastTheEndIndecesStock_;
  scalar_array2_t sTrajectoriesStock_;
  vector_array2_t SvTrajectoriesStock_;
  matrix_array2_t SmTrajectoriesStock_;

  vector_array3_t EvDevEventTimesTrajectoryStockSet_;           // state-input constraint derivative w.r.t. event times
  vector_array3_t EvDevEventTimesProjectedTrajectoryStockSet_;  // DmDager * EvDevEventTimes

 protected:
  /**
   * @brief Resizes the data of the other-class's member.
   * @param [in] numPartitions: Number of partitions.
   */
  void ResizeDataContainer(size_t numPartitions);

  /**
   * Calculates sensitivity of the state-input constraints to event times.
   *
   * @param [in] constDdpPtr: A pointer to the DDP instance.
   * @param [in] timeTrajectoriesStock: The time trajectory stamp.
   * @param [in] stateTrajectoriesStock: The state trajectory.
   * @param [in] inputTrajectoriesStock: The control input trajectory.
   * @param [out] EvDevEventTimesTrajectoriesStockSet: The sensitivity of the state-input constraints to the event times. Here
   * EvDevEventTimesTrajectoriesStockSet[i] is the EvDevEventTimesTrajectoriesStock for the i'th event time.
   * @param [out] EvDevEventTimesProjectedTrajectoriesStockSet: The projected sensitivity of the state-input constraints to the event times,
   * Defined as (DmDager * EvDevEventTimes).
   */
  void CalculateStateInputConstraintsSensitivity(const GaussNewtonDDP* constDdpPtr,
                                                 const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                 const vector_array2_t& stateTrajectoriesStock,
                                                 const vector_array2_t& inputTrajectoriesStock,
                                                 vector_array3_t& EvDevEventTimesTrajectoriesStockSet,
                                                 vector_array3_t& EvDevEventTimesProjectedTrajectoriesStockSet);

 private:
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
  std::unique_ptr<ConstraintBase> systemConstraintsPtr_;
  std::unique_ptr<CostFunctionBase> costFunctionPtr_;
};

}  // namespace bolza
