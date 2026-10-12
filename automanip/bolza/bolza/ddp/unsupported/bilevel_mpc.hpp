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

#include <condition_variable>
#include <mutex>
#include <thread>

#include <bolza/ddp/ddp_data_collector.hpp>
#include <bolza/bilevel/frank_wolfe_gddp.hpp>

#include "bolza/ddp/gauss_newton_ddp_mpc.hpp"

namespace bolza {
/**
 *
 * This an MPC implementation with bolza optimal control solver.
 */
class BilevelMpc : public GaussNewtonDDPMPC {
 public:
  /**
 * @brief Function.
 *
 * Constructor
 * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
 * @param [in] systemDynamicsPtr: The system dynamics derivatives for subsystems of the system.
 * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
 * @param [in]  /**
  * @brief.
  *
  * /
  * @param [in,out] defined: Defined.
  * @param [in] nullptr: Nullptr.
  * @return Result value.
  */
 costFunctionPtr: The cost Function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] partitioningTimes: This will be used as the initial time partitioning. As the MPC progresses the internal
   * partitioningTimes will be shifted in time automatically.
   * @param [in] slqSettings: Structure containing the settings for the SLQ algorithm.
   * @param [in] gddpSettings: Structure containing the settings for the GDDP algorithm.
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  /**
   * @brief Constructs a BilevelMpc.
   * @param [in] rolloutPtr: Rollout Ptr.
   * @param [in] systemDynamicsPtr: System Dynamics Ptr.
   * @param [in] systemConstraintsPtr: System Constraints Ptr.
   * @param [in] costFunctionPtr: Cost Function Ptr.
   * @param [in] operatingTrajectoriesPtr: Operating Trajectories Ptr.
   * @param [in] partitioningTimes: Partitioning Times.
   * @param [in] slqSettings: Slq Settings.
   * @param [in] gddpSettings: Gddp Settings.
   * @param [in] mpcSettings: Mpc Settings.
   * @param [in] heuristicsFunctionPtr: Heuristics Function Ptr.
   */
  BilevelMpc(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
           const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
           const scalar_array_t& partitioningTimes, const ddp::Settings& slqSettings = ddp::Settings(),
           const GDDPSettings& gddpSettings = GDDPSettings(), const mpc::Settings& mpcSettings = mpc::Settings(),
           const CostFunctionBase* heuristicsFunctionPtr = nullptr);

  /**
 *
 * Destructor.
 */
  
  virtual ~BilevelMpc();
  /**
 *
 * Resets the class to its state after construction.
 */
  void Reset() override;/
    /**
   * @brief The main routine of MPC which runs MPC for the given state and time.
   *
  /**
   * @brief Run.
   * @param [in] currentTime: The given time.
   * @param [in] currentState: The given state.
   * @return True on success.
   */
  virtual bool Run(const scalar_t& currentTime, const vector_t& currentState);/*
 protected:
  /**
 *
 * Rewinds the SLQ-MPC.
 */
  virtual void Rewind();/**
 /**
  * @brief Run Bilevel Worker.
  */
 void RunBilevelWorker();

 private:
  std::unique_ptr<FrankWolfeGDDP> gddpPtr_;

  std::thread workerBilevel_;

  std::mutex dataCollectorMutex_;
  bool activateBilevel_;
  std::atomic_bool terminateBilevel_;
  std::condition_variable bilevelSynchronization_;

  std::unique_ptr<DDPDataCollector> slqDataCollectorPtr_;

  scalar_array_t eventTimesOptimized_;
  size_array_t modeSequenceOptimized_;
};

}  // namespace bolza
