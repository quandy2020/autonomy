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

#include "bolza/mpcnet/rollout/mpcnet_rollout_manager.hpp"

namespace bolza {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPCNetRolloutManager::MPCNetRolloutManager(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads,
                                           std::vector<std::unique_ptr<MPCBase>> mpcPtrs,
                                           std::vector<std::unique_ptr<MPCNetControllerBase>> mpcnetPtrs,
                                           std::vector<std::unique_ptr<RolloutBase>> rolloutPtrs,
                                           std::vector<std::shared_ptr<MPCNetDefinitionBase>> mpcnetDefinitionPtrs,
                                           std::vector<std::shared_ptr<ReferenceManagerInterface>> referenceManagerPtrs) {
  // data generation
  nDataGenerationThreads_ = nDataGenerationThreads;
  if (nDataGenerationThreads_ > 0) {
    dataGenerationThreadPoolPtr_.reset(new ThreadPool(nDataGenerationThreads_));
    dataGenerationPtrs_.reserve(nDataGenerationThreads);
    for (int i = 0; i < nDataGenerationThreads; i++) {
      dataGenerationPtrs_.push_back(
          std::make_unique<MPCNetDataGeneration>(std::move(mpcPtrs.at(i)), std::move(mpcnetPtrs.at(i)), std::move(rolloutPtrs.at(i)),
                                                 std::move(mpcnetDefinitionPtrs.at(i)), referenceManagerPtrs.at(i)));
    }
  }

  // policy evaluation
  nPolicyEvaluationThreads_ = nPolicyEvaluationThreads;
  if (nPolicyEvaluationThreads_ > 0) {
    policyEvaluationThreadPoolPtr_.reset(new ThreadPool(nPolicyEvaluationThreads_));
    policyEvaluationPtrs_.reserve(nPolicyEvaluationThreads_);
    for (int i = nDataGenerationThreads_; i < (nDataGenerationThreads_ + nPolicyEvaluationThreads_); i++) {
      policyEvaluationPtrs_.push_back(
          std::make_unique<MPCNetPolicyEvaluation>(std::move(mpcPtrs.at(i)), std::move(mpcnetPtrs.at(i)), std::move(rolloutPtrs.at(i)),
                                                   std::move(mpcnetDefinitionPtrs.at(i)), referenceManagerPtrs.at(i)));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetRolloutManager::StartDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation,
                                               size_t nSamples, const matrix_t& samplingCovariance,
                                               const std::vector<SystemObservation>& initialObservations,
                                               const std::vector<ModeSchedule>& modeSchedules,
                                               const std::vector<TargetTrajectories>& targetTrajectories) {
  if (nDataGenerationThreads_ <= 0) {
    throw std::runtime_error("[MPCNetRolloutManager::StartDataGeneration] cannot work without at least one data generation thread.");
  }

  // reset variables
  dataGenerationFtrs_.clear();
  nDataGenerationTasksDone_ = 0;

  // push tasks into pool
  for (int i = 0; i < initialObservations.size(); i++) {
    dataGenerationFtrs_.push_back(dataGenerationThreadPoolPtr_->run([=](int threadNumber) {
      const auto* result =
          dataGenerationPtrs_[threadNumber]->run(alpha, policyFilePath, timeStep, dataDecimation, nSamples, samplingCovariance,
                                                 initialObservations.at(i), modeSchedules.at(i), targetTrajectories.at(i));
      nDataGenerationTasksDone_++;
      // print thread and task number
      std::cerr << "Data generation thread " << threadNumber << " finished task " << nDataGenerationTasksDone_ << "\n";
      return result;
    }));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPCNetRolloutManager::IsDataGenerationDone() {
  if (nDataGenerationThreads_ <= 0) {
    throw std::runtime_error("[MPCNetRolloutManager::IsDataGenerationDone] cannot work without at least one data generation thread.");
  }
  if (dataGenerationFtrs_.size() <= 0) {
    throw std::runtime_error(
        "[MPCNetRolloutManager::IsDataGenerationDone] cannot return if startDataGeneration has not been triggered once.");
  }

  // check if done
  if (nDataGenerationTasksDone_ < dataGenerationFtrs_.size()) {
    return false;
  } else if (nDataGenerationTasksDone_ == dataGenerationFtrs_.size()) {
    return true;
  } else {
    throw std::runtime_error("[MPCNetRolloutManager::IsDataGenerationDone] error since more tasks done than futures available.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const data_array_t& MPCNetRolloutManager::GetGeneratedData() {
  if (nDataGenerationThreads_ <= 0) {
    throw std::runtime_error("[MPCNetRolloutManager::GetGeneratedData] cannot work without at least one data generation thread.");
  }
  if (!IsDataGenerationDone()) {
    throw std::runtime_error("[MPCNetRolloutManager::GetGeneratedData] cannot get data when data generation is not done.");
  }

  // clear data array
  dataArray_.clear();

  // get pointers to data
  std::vector<const data_array_t*> dataPtrs;
  dataPtrs.reserve(dataGenerationFtrs_.size());
  for (auto& dataGenerationFtr : dataGenerationFtrs_) {
    try {
      // get results from futures of the tasks
      dataPtrs.push_back(dataGenerationFtr.get());
    } catch (const std::exception& e) {
      // print error for exceptions
      std::cerr << "[MPCNetRolloutManager::GetGeneratedData] a standard exception was caught, with message: " << e.what() << "\n";
    }
  }

  // find number of data points
  int nDataPoints = 0;
  for (int i = 0; i < dataPtrs.size(); i++) {
    nDataPoints += dataPtrs[i]->size();
  }

  // fill data array
  dataArray_.reserve(nDataPoints);
  for (const auto dataPtr : dataPtrs) {
    dataArray_.insert(dataArray_.end(), dataPtr->begin(), dataPtr->end());
  }

  // return data array
  return dataArray_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetRolloutManager::StartPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                                                 const std::vector<SystemObservation>& initialObservations,
                                                 const std::vector<ModeSchedule>& modeSchedules,
                                                 const std::vector<TargetTrajectories>& targetTrajectories) {
  if (nPolicyEvaluationThreads_ <= 0) {
    throw std::runtime_error("[MPCNetRolloutManager::StartPolicyEvaluation] cannot work without at least one policy evaluation thread.");
  }

  // reset variables
  policyEvaluationFtrs_.clear();
  nPolicyEvaluationTasksDone_ = 0;

  // push tasks into pool
  for (int i = 0; i < initialObservations.size(); i++) {
    policyEvaluationFtrs_.push_back(policyEvaluationThreadPoolPtr_->run([=](int threadNumber) {
      const auto result = policyEvaluationPtrs_[threadNumber]->run(alpha, policyFilePath, timeStep, initialObservations.at(i),
                                                                   modeSchedules.at(i), targetTrajectories.at(i));
      nPolicyEvaluationTasksDone_++;
      // print thread and task number
      std::cerr << "Policy evaluation thread " << threadNumber << " finished task " << nPolicyEvaluationTasksDone_ << "\n";
      return result;
    }));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPCNetRolloutManager::IsPolicyEvaluationDone() {
  if (nPolicyEvaluationThreads_ <= 0) {
    throw std::runtime_error("[MPCNetRolloutManager::IsPolicyEvaluationDone] cannot work without at least one policy evaluation thread.");
  }
  if (policyEvaluationFtrs_.size() <= 0) {
    throw std::runtime_error(
        "[MPCNetRolloutManager::IsPolicyEvaluationDone] cannot return if startPolicyEvaluation has not been triggered once.");
  }

  // check if done
  if (nPolicyEvaluationTasksDone_ < policyEvaluationFtrs_.size()) {
    return false;
  } else if (nPolicyEvaluationTasksDone_ == policyEvaluationFtrs_.size()) {
    return true;
  } else {
    throw std::runtime_error("[MPCNetRolloutManager::IsPolicyEvaluationDone] error since more tasks done than futures available.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
metrics_array_t MPCNetRolloutManager::GetComputedMetrics() {
  if (nPolicyEvaluationThreads_ <= 0) {
    throw std::runtime_error("[MPCNetRolloutManager::GetComputedMetrics] cannot work without at least one policy evaluation thread.");
  }
  if (!IsPolicyEvaluationDone()) {
    throw std::runtime_error("[MPCNetRolloutManager::GetComputedMetrics] cannot get metrics when policy evaluation is not done.");
  }

  // get metrics and fill metrics array
  metrics_array_t metricsArray;
  metricsArray.reserve(policyEvaluationFtrs_.size());
  for (auto& policyEvaluationFtr : policyEvaluationFtrs_) {
    try {
      // get results from futures of the tasks
      metricsArray.push_back(policyEvaluationFtr.get());
    } catch (const std::exception& e) {
      // print error for exceptions
      std::cerr << "[MPCNetRolloutManager::GetComputedMetrics] a standard exception was caught, with message: " << e.what() << "\n";
    }
  }

  // return metrics array
  return metricsArray;
}

}  // namespace mpcnet
}  // namespace bolza
