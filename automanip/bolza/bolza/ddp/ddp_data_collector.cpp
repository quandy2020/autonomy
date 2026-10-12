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

#include "bolza/ddp/ddp_data_collector.hpp"

namespace bolza {

void DDPDataCollector::Collect(const GaussNewtonDDP* constDdpPtr) {
  // TODO(mspieler): avoid const_cast
  auto* ddpPtr = const_cast<GaussNewtonDDP*>(constDdpPtr);

  /*
   * Data which should be copied
   */
  // initial time and state plus final time
  initTime_ = ddpPtr->initTime_;
  finalTime_ = ddpPtr->finalTime_;
  initState_ = ddpPtr->initState_;

  // active partitions range: [initActivePartition_, finalActivePartition_]
  initActivePartition_ = ddpPtr->initActivePartition_;
  finalActivePartition_ = ddpPtr->finalActivePartition_;

  // data resizing
  bool numPartitionsChanged = numPartitions_ != ddpPtr->numPartitions_;
  if (numPartitionsChanged) {
    ResizeDataContainer(ddpPtr->numPartitions_);
  }

  numPartitions_ = ddpPtr->numPartitions_;
  partitioningTimes_ = ddpPtr->partitioningTimes_;

  rewindCounter_ = ddpPtr->rewindCounter_;

  modeSchedule_ = ddpPtr->GetReferenceManager().GetModeSchedule();

  // optimized controller
  optimizedControllersStock_ = ddpPtr->nominalControllersStock_;

  // nominal trajectories (LQ approximation is around the cached trajectories)
  nominalPostEventIndicesStock_ = ddpPtr->cachedPostEventIndicesStock_;
  nominalTimeTrajectoriesStock_ = ddpPtr->cachedTimeTrajectoriesStock_;
  nominalStateTrajectoriesStock_ = ddpPtr->cachedStateTrajectoriesStock_;
  nominalInputTrajectoriesStock_ = ddpPtr->cachedInputTrajectoriesStock_;

  /*
   * Data which can be swapped. Note that these variables should have correct size.
   * Otherwise use setOptimizer() to construct them with correct size
   */
  // model data trajectory
  modelDataTrajectoriesStock_.swap(ddpPtr->cachedModelDataTrajectoriesStock_);

  // event times model data
  modelDataEventTimesStock_.swap(ddpPtr->cachedModelDataEventTimesStock_);

  // projected model data trajectory
  projectedModelDataTrajectoriesStock_.swap(ddpPtr->cachedProjectedModelDataTrajectoriesStock_);

  // Riccati modification
  riccatiModificationTrajectoriesStock_.swap(ddpPtr->cachedRiccatiModificationTrajectoriesStock_);

  // final cost which is interpreted as the Heuristic function
  std::swap(heuristics_, ddpPtr->heuristics_);

  // Riccati coefficients
  SsTimeTrajectoriesStock_.swap(ddpPtr->SsTimeTrajectoryStock_);
  SsNormalizedTimeTrajectoriesStock_.swap(ddpPtr->SsNormalizedTimeTrajectoryStock_);
  SsNormalizedEventsPastTheEndIndecesStock_.swap(ddpPtr->SsNormalizedEventsPastTheEndIndecesStock_);
  SmTrajectoriesStock_.swap(ddpPtr->SmTrajectoryStock_);
  SvTrajectoriesStock_.swap(ddpPtr->SvTrajectoryStock_);
  sTrajectoriesStock_.swap(ddpPtr->sTrajectoryStock_);

  // state-input constraints derivatives w.r.t. to the event times
  CalculateStateInputConstraintsSensitivity(constDdpPtr, nominalTimeTrajectoriesStock_, nominalStateTrajectoriesStock_,
                                            nominalInputTrajectoriesStock_, EvDevEventTimesTrajectoryStockSet_,
                                            EvDevEventTimesProjectedTrajectoryStockSet_);
}

void DDPDataCollector::ResizeDataContainer(size_t numPartitions) {
  if (numPartitions == 0) {
    throw std::runtime_error("The number of Partitions cannot be zero!");
  }

  /*
   * Data which should be copied
   */
  // optimized controller
  optimizedControllersStock_.resize(numPartitions);

  // nominal trajectories
  nominalTimeTrajectoriesStock_.resize(numPartitions);
  nominalPostEventIndicesStock_.resize(numPartitions);
  nominalStateTrajectoriesStock_.resize(numPartitions);
  nominalInputTrajectoriesStock_.resize(numPartitions);

  /*
   * Data which can be swapped. Note that these variables should have correct size.
   * Otherwise use setOptimizer() to construct them with correct size
   */
  // model data trajectory
  modelDataTrajectoriesStock_.resize(numPartitions);

  // event times model data
  modelDataEventTimesStock_.resize(numPartitions);

  // projected model data trajectory
  projectedModelDataTrajectoriesStock_.resize(numPartitions);

  // Riccati modification
  riccatiModificationTrajectoriesStock_.resize(numPartitions);

  // Riccati coefficients
  SsTimeTrajectoriesStock_.resize(numPartitions);
  SsNormalizedTimeTrajectoriesStock_.resize(numPartitions);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
  SmTrajectoriesStock_.resize(numPartitions);
  SvTrajectoriesStock_.resize(numPartitions);
  sTrajectoriesStock_.resize(numPartitions);
}

}  // namespace bolza
