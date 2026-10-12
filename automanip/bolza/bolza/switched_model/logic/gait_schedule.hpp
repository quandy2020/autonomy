//
// Created by rgrandia on 15.03.20.
//

#pragma once

/**
 * @file gait_schedule.hpp
 * @brief Advances the gait schedule to a specified time, the specified time must be increasing w.r.t. previous calls.
 */


#include <deque>
#include <vector>

#include <bolza/core/reference/mode_schedule.hpp>

#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/logic/gait.hpp"

namespace switched_model {

class GaitSchedule {
 public:
  using GaitSequence = std::vector<Gait>;
  /**
   * @brief Constructs a GaitSchedule.
   * @param [in] time: Time.
   * @param [in] gait: Gait.
   */
  GaitSchedule(scalar_t time, Gait gait);

  /**
   * @brief Advances the gait schedule to a specified time, the specified time must be increasing w.r.t. previous calls.
   * @param [in] time: Time.
   */
  void AdvanceToTime(scalar_t time);

  /**
   * @brief Adds gaits after completing the current gait.
   * @param [in] gait: Gait.
   */
  void SetNextGait(const Gait& gait);

  /**
   * @brief Adds sequence of gaits after completing the current gait.
   * @param [in] gaitSequence: Gait Sequence.
   */
  void SetGaitSequenceAfterCurrentGait(const GaitSequence& gaitSequence);

  /**
   * @brief Adds a gait at a specified time. The gait originally active at that time is shrunk in duration to make the new gait fit. Gaits after.
   *
   * that time are removed
   * @param [in] gait: Gait.
   * @param [in] time: Time.
   */
  void SetGaitAtTime(const Gait& gait, scalar_t time);

  /**
   * @brief Adds schedule of gaits at a specified time. The gait originally active at that time is shrunk in duration to make the new gait fit.
   *
   * Gaits after that time are removed
   * @param [in] gaitSequence: Gait Sequence.
   * @param [in] time: Time.
   */
  void SetGaitSequenceAtTime(const GaitSequence& gaitSequence, scalar_t time);

  /**
   * @brief Adds a gait at the first opportunity after the specified time. Does not adapt gait cycle durations.
   * @param [in] gait: Gait.
   * @param [in] time: Time.
   */
  void SetGaitAfterTime(const Gait& gait, scalar_t time);

  /**
   * @brief Adds a gait sequence  at the first opportunity after the specified time. Does not adapt gait cycle durations.
   * @param [in] gaitSequence: Gait Sequence.
   * @param [in] time: Time.
   */
  void SetGaitSequenceAfterTime(const GaitSequence& gaitSequence, scalar_t time);

  /**
   * @brief Applies the provided function to the current gait. The function can adapt the current phase and gait. If the current gait is the last.
   *
   * scheduled gait, it is repeated before adaptation
   * @param [in] gaitAdaptor: Gait Adaptor.
   */
  void AdaptCurrentGait(
      const std::function<void(scalar_t& currentPhase, Gait& currentGait, scalar_t currTime, Gait& nextGait)>& gaitAdaptor);

  /**
   * @brief Gets the gaitSchedule as a mode schedule from the current time, and for the specified horizon.
   * @param [in] timeHorizon: Time Horizon.
   * @return Result value.
   */
  bolza::ModeSchedule GetModeSchedule(scalar_t timeHorizon) const;

  /**
   * @brief Gets the currently active gait.
   * @return Result value.
   */
  const Gait& GetCurrentGait() const { return gaitSchedule_.front(); }

  /**
   * @brief Gets phase variable for the current gait.
   * @return Result value.
   */
  scalar_t GetCurrentPhase() const { return phase_; }

  /**
   * @brief Gets time variable for the current gait.
   * @return Result value.
   */
  scalar_t GetCurrentTime() const { return time_; }

  /**
   * @brief Gets the currently active mode.
   * @return Result value.
   */
  size_t GetCurrentMode() const { return GetModeFromPhase(GetCurrentPhase(), GetCurrentGait()); }

  /**
   * @brief Returns time until current gait ends a cycle.
   * @return Result value.
   */
  scalar_t TimeLeftInCurrentGait() const { return TimeLeftInGait(GetCurrentPhase(), GetCurrentGait()); }

 private:
  /**
   * @brief Makes the implicit looping of the last gait explicit until the specified time.
   * @param [in] time: Time.
   */
  void RolloutGaitScheduleTillTime(scalar_t time);

  scalar_t time_;
  scalar_t phase_;
  std::deque<Gait> gaitSchedule_;
};

/**
 * @brief Is Standing During Time Horizon.
 * @param [in] timeHorizon: Time Horizon.
 * @param [in] gaitSchedule: Gait Schedule.
 * @return True on success.
 */
bool IsStandingDuringTimeHorizon(scalar_t timeHorizon, const GaitSchedule& gaitSchedule);

/**
 * @brief Is Standing.
 * @param [in] gaitSchedule: Gait Schedule.
 * @return True on success.
 */
bool IsStanding(const GaitSchedule& gaitSchedule);

}  // namespace switched_model
