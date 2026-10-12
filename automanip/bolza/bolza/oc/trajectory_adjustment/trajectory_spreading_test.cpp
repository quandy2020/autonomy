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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include <bolza/core/types.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/misc/lookup.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include <bolza/oc/trajectory_adjustment/trajectory_spreading.hpp>

struct Result {
  bolza::scalar_array_t timeTrajectory;
  bolza::vector_array_t stateTrajectory;
  bolza::vector_array_t inputTrajectory;
  bolza::size_array_t postEventsIndeces;
  bolza::vector_array_t eventDataArray;
  // for testing
  bolza::size_array_t modeTrajectory;
  bolza::size_array_t preEventModeTrajectory;
};

std::size_t ModeAtTime(const bolza::ModeSchedule& modeSchedule, bolza::scalar_t time, bool isFinalTime) {
  if (isFinalTime) {
    auto timeItr = std::upper_bound(modeSchedule.eventTimes.begin(), modeSchedule.eventTimes.end(), time);
    int modeIndex = std::distance(modeSchedule.eventTimes.begin(), timeItr);
    return modeSchedule.modeSequence[modeIndex];
  } else {
    return modeSchedule.ModeAtTime(time);
  }
}

class TrajectorySpreadingTest : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr bool DEBUG_PRINT = false;

  /******************************************************************************************************/
  TrajectorySpreadingTest() {
    // rollout settings
    const auto rolloutSettings = []() {
      bolza::Rollout::Settings rolloutSettings;
      rolloutSettings.absTolODE = 1e-8;
      rolloutSettings.relTolODE = 1e-6;
      rolloutSettings.maxNumStepsPerSecond = 10000;
      return rolloutSettings;
    }();

    // dynamics and rollout
    const bolza::matrix_t A = (bolza::matrix_t(STATE_DIM, STATE_DIM) << -2.0, -1.0, 1.0, 0.0).finished();
    const bolza::matrix_t B = (bolza::matrix_t(STATE_DIM, INPUT_DIM) << 1.0, 0.0).finished();
    const bolza::matrix_t G = (bolza::matrix_t(STATE_DIM, STATE_DIM) << 1.0, 0.0, 0.0, 0.9).finished();
    systemPtr.reset(new bolza::LinearSystemDynamics(A, B, G));
    rolloutPtr.reset(new bolza::TimeTriggeredRollout(*systemPtr, rolloutSettings));

    // controller
    const bolza::scalar_array_t cntTimeStamp{timeInterval.first, timeInterval.second};
    const bolza::vector_array_t uff(2, bolza::vector_t::Ones(INPUT_DIM));
    const bolza::matrix_array_t k(2, bolza::matrix_t::Zero(INPUT_DIM, STATE_DIM));
    controllerPtr.reset(new bolza::LinearController(cntTimeStamp, uff, k));

    // trajectory spreading
    trajectorySpreadingPtr.reset(new bolza::TrajectorySpreading(true));
  }

  /******************************************************************************************************/
  Result Rollout(const bolza::ModeSchedule& modeSchedule, const std::pair<bolza::scalar_t, bolza::scalar_t>& timeInterval) const {
    Result out;
    bolza::ModeSchedule modeScheduleTemp = modeSchedule;  // since it is TimeTriggeredRollout, it will not be changed
    rolloutPtr->Run(timeInterval.first, initState, timeInterval.second, controllerPtr.get(), modeScheduleTemp, out.timeTrajectory,
                    out.postEventsIndeces, out.stateTrajectory, out.inputTrajectory);

    out.eventDataArray.resize(out.postEventsIndeces.size());
    // store pre-event state to event data array
    std::transform(out.postEventsIndeces.begin(), out.postEventsIndeces.end(), out.eventDataArray.begin(),
                   [&out](size_t eventIndex) { return out.stateTrajectory[eventIndex - 1]; });

    out.modeTrajectory.resize(out.timeTrajectory.size());
    for (int i = 0; i < out.modeTrajectory.size(); i++) {
      out.modeTrajectory[i] = ModeAtTime(modeSchedule, out.timeTrajectory[i], i == out.modeTrajectory.size() - 1 ? true : false);
    }

    out.preEventModeTrajectory.resize(out.postEventsIndeces.size());
    std::transform(out.postEventsIndeces.begin(), out.postEventsIndeces.end(), out.preEventModeTrajectory.begin(),
                   [&out, &modeSchedule](size_t eventIndex) { return modeSchedule.ModeAtTime(out.timeTrajectory[eventIndex - 1]); });
    return out;
  }

  /******************************************************************************************************/
  std::pair<Result, bolza::TrajectorySpreading::Status> spreadRollout(const bolza::ModeSchedule& modeSchedule,
                                                                     const bolza::ModeSchedule& updatedModeSchedule, const Result& in) {
    // set
    const auto status = trajectorySpreadingPtr->Set(modeSchedule, updatedModeSchedule, in.timeTrajectory);

    Result out = in;
    trajectorySpreadingPtr->AdjustTimeTrajectory(out.timeTrajectory);
    out.postEventsIndeces = trajectorySpreadingPtr->GetPostEventIndices();
    trajectorySpreadingPtr->AdjustTrajectory<bolza::vector_t>(out.stateTrajectory);
    trajectorySpreadingPtr->AdjustTrajectory<bolza::vector_t>(out.inputTrajectory);
    trajectorySpreadingPtr->AdjustTrajectory(out.modeTrajectory);
    out.eventDataArray = trajectorySpreadingPtr->ExtractEventsArray(out.eventDataArray);
    out.preEventModeTrajectory = trajectorySpreadingPtr->ExtractEventsArray(out.preEventModeTrajectory);

    return {out, status};
  }

  /******************************************************************************************************/
  bolza::TrajectorySpreading::Status checkResults(const bolza::ModeSchedule& modeSchedule, const bolza::ModeSchedule& updatedModeSchedule,
                                                 const std::pair<bolza::scalar_t, bolza::scalar_t>& period, bool display = true) {
    // rollout
    const auto result = Rollout(modeSchedule, period);

    if (display) {
      std::cerr << "\nBefore TrajectorySpreading: \n";
      auto itr = result.postEventsIndeces.begin();
      for (size_t k = 0; k < result.timeTrajectory.size(); k++) {
        std::cerr << "\033[0;33m[" << std::setw(2) << k << "]\033[0m";
        std::cerr << "\033[0;34m(" << std::setw(2) << result.modeTrajectory[k] << ")\033[0m";

        std::cerr << "state [" << result.timeTrajectory[k] << "]: " << result.stateTrajectory[k].transpose() << "\n";
        if (itr != result.postEventsIndeces.end() && *itr == k + 1) {
          size_t eventInd = bolza::lookup::FindIndexInTimeArray(modeSchedule.eventTimes, result.timeTrajectory[k]);
          size_t preMode = modeSchedule.modeSequence[eventInd];
          size_t postMode = modeSchedule.modeSequence[eventInd + 1];
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          std::cerr << "Event time: " << result.timeTrajectory[*itr] << "(" << preMode << "->" << postMode << ")"
                    << "\n";
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          itr++;
        }
      }  // end of k loop
      std::cerr << std::endl;
    }

    // spread rollout
    Result spreadResult;
    bolza::TrajectorySpreading::Status status;
    std::tie(spreadResult, status) = spreadRollout(modeSchedule, updatedModeSchedule, result);

    if (display) {
      std::cerr << "\nAfter TrajectorySpreading: \n";
      auto itr = spreadResult.postEventsIndeces.begin();
      for (size_t k = 0; k < spreadResult.timeTrajectory.size(); k++) {
        std::cerr << "\033[0;33m[" << std::setw(2) << k << "]\033[0m";
        std::cerr << "\033[0;34m(" << std::setw(2) << spreadResult.modeTrajectory[k] << ")\033[0m";

        std::cerr << "state [" << spreadResult.timeTrajectory[k] << "]: " << spreadResult.stateTrajectory[k].transpose() << "\n";
        if (itr != spreadResult.postEventsIndeces.end() && *itr == k + 1) {
          size_t eventInd = bolza::lookup::FindIndexInTimeArray(updatedModeSchedule.eventTimes, spreadResult.timeTrajectory[k]);
          size_t preMode = updatedModeSchedule.modeSequence[eventInd];
          size_t postMode = updatedModeSchedule.modeSequence[eventInd + 1];
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          std::cerr << "Event time: " << spreadResult.timeTrajectory[*itr] << "(" << preMode << "->" << postMode << ")"
                    << "\n";
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          itr++;
        }
      }  // end of k loop
      std::cerr << std::endl;
    }

    if (display) {
      auto print = [](size_t n) { std::cerr << n << ", "; };
      std::cerr << "Post Event Indices: ";
      std::for_each(spreadResult.postEventsIndeces.begin(), spreadResult.postEventsIndeces.end(), print);
      std::cerr << "\n";
    }

    // test postEventsIndeces
    if (!spreadResult.timeTrajectory.empty()) {
      const bolza::scalar_t initTime = spreadResult.timeTrajectory.front();
      const bolza::scalar_t finalTime = spreadResult.timeTrajectory.back();

      const auto startEventItr = std::upper_bound(updatedModeSchedule.eventTimes.begin(), updatedModeSchedule.eventTimes.end(), initTime);
      // If the final time is aligned with(the same as) an event time, it belongs to the post-mode.
      const auto endEventItr = std::upper_bound(updatedModeSchedule.eventTimes.begin(), updatedModeSchedule.eventTimes.end(), finalTime);

      bolza::size_array_t PostEventIndices(endEventItr - startEventItr);
      for (auto itr = startEventItr; itr != endEventItr; itr++) {
        int index = std::distance(startEventItr, itr);
        if (itr == endEventItr - 1 && *itr == spreadResult.timeTrajectory.back()) {
          postEventIndices[index] = spreadResult.timeTrajectory.size() - 1;
        } else {
          auto timeItr = std::upper_bound(spreadResult.timeTrajectory.begin(), spreadResult.timeTrajectory.end(), *itr);
          postEventIndices[index] = std::distance(spreadResult.timeTrajectory.begin(), timeItr);
        }
      }

      EXPECT_TRUE(spreadResult.postEventsIndeces.size() == postEventIndices.size());
      EXPECT_TRUE(std::Equal(postEventIndices.begin(), postEventIndices.end(), spreadResult.postEventsIndeces.begin()));

    } else {
      EXPECT_EQ(spreadResult.postEventsIndeces.size(), 0);
    }
    // test postEventsIndeces end

    // test timeTrajectory & stateTrajectory
    // initial time should NOT be changed as long as the time trajectory is non-empty
    if (!spreadResult.timeTrajectory.empty()) {
      EXPECT_TRUE(std::abs(spreadResult.timeTrajectory.front() - period.first) < bolza::numeric_traits::LimitEpsilon<bolza::scalar_t>());
    }

    auto eventIndexActualItr = spreadResult.postEventsIndeces.begin();
    auto eventTimeReferenceInd = bolza::lookup::FindIndexInTimeArray(updatedModeSchedule.eventTimes, period.first);
    for (size_t k = 0; k < spreadResult.timeTrajectory.size(); k++) {
      // Time should be monotonic sequence except the final time. It is possible that the last two time points have the same time, but one
      // stands for pre-mode and the other stands for post-mode.
      if (k > 0 && k < spreadResult.timeTrajectory.size() - 1) {
        EXPECT_TRUE(spreadResult.timeTrajectory[k - 1] < spreadResult.timeTrajectory[k]) << "TimeIndex: " << k;
      }

      // Pre-event time should be equal to the event time
      if (eventIndexActualItr != spreadResult.postEventsIndeces.end() && *eventIndexActualItr == k + 1) {
        constexpr auto eps = bolza::numeric_traits::LimitEpsilon<bolza::scalar_t>();
        EXPECT_EQ(spreadResult.timeTrajectory[k], updatedModeSchedule.eventTimes[eventTimeReferenceInd]);

        // when there is only one point in a mode(for example mode A), the next mode(for example mode B) will overwrite the post-event time
        // of mode A, and thus the time differene between post-event and  pre-event is NOT necessary to be within the limit epsilon
        // EXPECT_LT(spreadResult.timeTrajectory[k + 1], updatedModeSchedule.eventTimes[eventTimeReferenceInd] + eps);

        eventTimeReferenceInd++;
        eventIndexActualItr++;
      }
      // mode should match the given modeSchedule
      auto updatedMode =
          ModeAtTime(updatedModeSchedule, spreadResult.timeTrajectory[k], k == spreadResult.timeTrajectory.size() - 1 ? true : false);
      EXPECT_TRUE(updatedMode == spreadResult.modeTrajectory[k]);
    }  // end of k loop

    // test postEventsIndeces
    EXPECT_TRUE(spreadResult.postEventsIndeces.size() == spreadResult.eventDataArray.size());
    for (int k = 0; k < spreadResult.postEventsIndeces.size(); k++) {
      const size_t preEventIndex = spreadResult.postEventsIndeces[k] - 1;
      const bolza::scalar_t preEventTime = spreadResult.timeTrajectory[preEventIndex];
      // When spreading event time backward, the pre-event may change and differ from whatever that are stored in the eventDataArray. In
      // this situation, the following check will fail.
      // EXPECT_TRUE(spreadResult.stateTrajectory[preEventIndex].isApprox(spreadResult.eventDataArray[k])) << "Event #" << k << " is wrong";
      EXPECT_EQ(updatedModeSchedule.ModeAtTime(preEventTime), spreadResult.preEventModeTrajectory[k]);
    }

    return status;
  }

  /******************************************************************************************************/
  const bolza::vector_t initState = (bolza::vector_t(STATE_DIM) << 0.0, 2.0).finished();
  const std::pair<bolza::scalar_t, bolza::scalar_t> timeInterval{0.0, 1.0};

  std::unique_ptr<bolza::SystemDynamicsBase> systemPtr;
  std::unique_ptr<bolza::TimeTriggeredRollout> rolloutPtr;
  std::unique_ptr<bolza::LinearController> controllerPtr;
  std::unique_ptr<bolza::TrajectorySpreading> trajectorySpreadingPtr;
};

constexpr size_t TrajectorySpreadingTest::STATE_DIM;
constexpr size_t TrajectorySpreadingTest::INPUT_DIM;
constexpr bool TrajectorySpreadingTest::DEBUG_PRINT;

TEST_F(TrajectorySpreadingTest, no_change) {
  const bolza::scalar_array_t eventTimes{0.6, 1.7};
  const bolza::size_array_t modeSequence{0, 1, 2};
  const bolza::ModeSchedule& modeSchedule{eventTimes, modeSequence};
  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.0, 2};

  // rollout
  const auto result = Rollout(modeSchedule, period);
  // spread rollout
  Result spreadResult;
  bolza::TrajectorySpreading::Status status;
  std::tie(spreadResult, status) = spreadRollout(modeSchedule, modeSchedule, result);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_FALSE(status.willPerformTrajectorySpreading);
  EXPECT_TRUE(result.timeTrajectory == spreadResult.timeTrajectory);
  EXPECT_TRUE(result.stateTrajectory == spreadResult.stateTrajectory);
  EXPECT_TRUE(result.inputTrajectory == spreadResult.inputTrajectory);
  EXPECT_TRUE(result.postEventsIndeces == spreadResult.postEventsIndeces);
  EXPECT_TRUE(result.eventDataArray == spreadResult.eventDataArray);
}

TEST_F(TrajectorySpreadingTest, no_matching_modes) {
  const bolza::scalar_array_t eventTimes{0.6, 1.7};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{1.0, 1.1};
  const bolza::size_array_t updatedModeSequence{10, 11, 12};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.0, 2.0};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_TRUE(status.willTruncate);
  EXPECT_FALSE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, partially_matching_modes) {
  const bolza::scalar_array_t eventTimes{0.6, 1.7, 2.3};
  const bolza::size_array_t modeSequence{0, 1, 2, 3};

  const bolza::scalar_array_t updatedEventTimes{0.9, 1.1, 2.1};
  const bolza::size_array_t updatedModeSequence{10, 1, 2, 30};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{1.0, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_TRUE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, final_time_is_the_same_as_event_time_1) {
  const bolza::scalar_array_t eventTimes{1.1, 1.3};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{1.1, 2.1};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.2, 2.1};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, final_time_is_the_same_as_event_time_2) {
  const bolza::scalar_array_t eventTimes{1.1, 2.1};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{1.1, 1.3};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.2, 2.1};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, erase_trajectory) {
  const bolza::scalar_array_t eventTimes{1.1, 1.3};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{1.1, 1.3};
  const bolza::size_array_t updatedModeSequence{0, 1, 3};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.2, 2.1};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_TRUE(status.willTruncate);
  EXPECT_FALSE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, fully_matched_modes) {
  const bolza::scalar_array_t eventTimes{1.1, 1.3};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{0.5, 2.1};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.0, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_at_back_1) {
  const bolza::scalar_array_t eventTimes{0.6, 1.7};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{1.0, 1.1};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.0, 1.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_at_back_2) {
  const bolza::scalar_array_t eventTimes{1.0};
  const bolza::size_array_t modeSequence{0, 1};

  const bolza::scalar_array_t updatedEventTimes{1.0, 2.0};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_at_back_1) {
  const bolza::scalar_array_t eventTimes{1.0, 1.1};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{0.6, 1.7};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.0, 1.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_at_back_2) {
  const bolza::scalar_array_t eventTimes{1, 2, 3.1};
  const bolza::size_array_t modeSequence{0, 1, 2, 4};

  const bolza::scalar_array_t updatedEventTimes{1, 3};
  const bolza::size_array_t updatedModeSequence{0, 1, 3};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.5, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_in_front_1) {
  const bolza::scalar_array_t eventTimes{1, 2};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{0.5, 1.6};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_in_front_2) {
  const bolza::scalar_array_t eventTimes{1, 2};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{1.6};
  const bolza::size_array_t updatedModeSequence{1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_in_front_1) {
  const bolza::scalar_array_t eventTimes{2};
  const bolza::size_array_t modeSequence{1, 2};

  const bolza::scalar_array_t updatedEventTimes{0.5, 1.6};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_in_front_2) {
  const bolza::scalar_array_t eventTimes{0.5, 2};
  const bolza::size_array_t modeSequence{0, 1, 2};

  const bolza::scalar_array_t updatedEventTimes{1, 1.6};
  const bolza::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, overlap_forward) {
  const bolza::scalar_array_t eventTimes{1, 1.5, 2};
  const bolza::size_array_t modeSequence{0, 1, 2, 3};

  const bolza::scalar_array_t updatedEventTimes{1, 2.2, 2.3};
  const bolza::size_array_t updatedModeSequence{0, 1, 2, 3};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0.9, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, overlap_backward) {
  const bolza::scalar_array_t eventTimes{1, 2.2, 3};
  const bolza::size_array_t modeSequence{0, 1, 2, 3};

  const bolza::scalar_array_t updatedEventTimes{1, 1.5, 2};
  const bolza::size_array_t updatedModeSequence{0, 1, 2, 3};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{0, 3.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, anymal_test) {
  const bolza::scalar_array_t eventTimes{1.00001, 1.40001, 1.80001, 2.20001, 2.60001};
  const bolza::size_array_t modeSequence{15, 15, 7, 14, 11, 13};

  const bolza::scalar_array_t updatedEventTimes{1.51913, 1.91913, 2.31913, 2.71913, 3.11913};
  const bolza::size_array_t updatedModeSequence{15, 7, 14, 11, 13, 7};

  const std::pair<bolza::scalar_t, bolza::scalar_t> period{1.4, 2.4};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}
