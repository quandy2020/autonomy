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

#include "bolza/ddp/unsupported/bilevel_mpc.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BilevelMpc::BilevelMpc(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                   const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
                   const scalar_array_t& partitioningTimes, const ddp::Settings& slqSettings /*= ddp::Settings()*/,
                   const GDDPSettings& gddpSettings /*= GDDPSettings()*/, const mpc::Settings& mpcSettings /*= mpc::Settings()*/,
                   const CostFunctionBase* heuristicsFunctionPtr /*= nullptr*/)

    : GaussNewtonDDPMPC(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, partitioningTimes,
              slqSettings, mpcSettings, heuristicsFunctionPtr),
      gddpPtr_(new gddp_t(gddpSettings)),
      activateBilevel_(false),
      terminateBilevel_(false),
      slqDataCollectorPtr_(new slq_data_collector_t(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr)) {
  workerBilevel_ = std::thread(&BilevelMpc::RunBilevelWorker, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BilevelMpc::~BilevelMpc() {
  terminateBilevel_ = true;
  bilevelSynchronization_.notify_all();
  workerBilevel_.join();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void BilevelMpc::Reset() {
  GaussNewtonDDPMPC::Reset();

  std::lock_guard<std::mutex> bilevelLock(dataCollectorMutex_);
  activateBilevel_ = false;
  eventTimesOptimized_.clear();
  modeSequenceOptimized_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void BilevelMpc::Rewind() {
  GaussNewtonDDPMPC::Rewind();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void BilevelMpc::RunBilevelWorker() {
  while (!terminateBilevel_) {
    std::unique_lock<std::mutex> bilevelLock(dataCollectorMutex_);
    bilevelSynchronization_.wait(bilevelLock, [&] { return activateBilevel_ || terminateBilevel_; });

    // exit loop
    if (terminateBilevel_) {
      break;
    }

    if (GaussNewtonDDPMPC::mpcSettings_.debugPrint_) {
      std::cerr << "### Bilevel worker started. " << std::endl;
    }

    modeSequenceOptimized_ = slqDataCollectorPtr_->modeSchedule_.modeSequence;

    // TODO: fix me
    //    gddpPtr_->Run(slqDataCollectorPtr_.get(), eventTimesOptimized_,
    //    GaussNewtonDDPMPC::mpcSettings_.maxTimeStep_);

    if (GaussNewtonDDPMPC::mpcSettings_.debugPrint_) {
      std::cerr << "### Bilevel worker finished. " << std::endl;
    }

    activateBilevel_ = false;
    bilevelLock.unlock();
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool BilevelMpc::Run(const scalar_t& currentTime, const state_vector_t& currentState) {
  std::unique_lock<std::mutex> slqLock(dataCollectorMutex_, std::defer_lock_t());
  bool ownership = slqLock.try_lock();
  if (ownership && !GaussNewtonDDPMPC::initRun_) {
    bool rewaindTookPlace = currentTime > 0.1 && GaussNewtonDDPMPC::slqPtr_->getRewindCounter() != slqDataCollectorPtr_->rewindCounter_;
    auto modeSchedule = GaussNewtonDDPMPC::slqPtr_->GetModeSchedule();
    bool modeSequenceUpdated = modeSequenceOptimized_ != modeSchedule.modeSequence;
    if (!rewaindTookPlace && !modeSequenceUpdated) {
      // adjust the SLQ internal controller using trajectory spreading approach
      if (!modeSchedule.eventTimes.empty()) {
        GaussNewtonDDPMPC::slqPtr_->AdjustController(eventTimesOptimized_, modeSchedule.eventTimes);
      }

      // TODO : set eventTimesOptimized_ in the correct place
      // GaussNewtonDDPMPC::slqPtr_->setModeScheduleManagers({eventTimesOptimized_, modeSchedule.modeSequence});
      //      this->logicRulesPtr_->EventTimes() = eventTimesOptimized_;
      //      this->logicRulesPtr_->update();
      //      this->GetLogicRulesMachinePtr()->logicRulesUpdated();
    }

    // collect SLQ variables
    if (GaussNewtonDDPMPC::mpcSettings_.debugPrint_) {
      std::cerr << "### SLQ data collector triggered." << std::endl;
    }
    slqDataCollectorPtr_->Collect(GaussNewtonDDPMPC::slqPtr_.get());

    activateBilevel_ = true;
    slqLock.unlock();
    bilevelSynchronization_.notify_one();
  }

  // run the base method
  bool status = GaussNewtonDDPMPC::Run(currentTime, currentState);

  return status;
}

}  // namespace bolza
