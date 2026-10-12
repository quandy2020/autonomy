/*
 * GaitAdaptation.h
 *
 *  Created on: Jun 4, 2020
 *      Author: Marko Bjelonic
 */

#pragma once

/**
 * @file gait_adaptation.hpp
 * @brief Determines the gait adaptation strategy per leg.
 */


#include <bolza/switched_model/logic/gait.hpp>
#include <bolza/switched_model/logic/gait_schedule.hpp>

namespace switched_model {

struct GaitAdaptationSettings {
  scalar_t earlyTouchDownTimeWindow = 0.1;
};

class GaitAdaptation {
 public:
  /**
   * @brief Constructs a GaitAdaptation.
   * @param [in] settings: Settings.
   * @param [in] measuredContactFlags: Measured Contact Flags.
   */
  GaitAdaptation(const GaitAdaptationSettings& settings, const contact_flag_t& measuredContactFlags);

  /**
   * @brief Advance.
   * @param [in,out] gaitSchedule: Gait Schedule.
   * @param [in] measuredContactFlags: Measured Contact Flags.
   * @param [in] dt: Dt.
   */
  void Advance(GaitSchedule& gaitSchedule, const contact_flag_t& measuredContactFlags, scalar_t dt);

 private:
  /// Update measured liftoff information
  /**
   * @brief Advance Liftoff Tracking.
   * @param [in] desiredContactFlags: Desired Contact Flags.
   * @param [in] measuredContactFlags: Measured Contact Flags.
   */
  void AdvanceLiftoffTracking(const contact_flag_t& desiredContactFlags, const contact_flag_t& measuredContactFlags);

  /// Update next touchdown and liftoff information
  /**
   * @brief Advance Swing Events.
   * @param [in] gaitSchedule: Gait Schedule.
   */
  void AdvanceSwingEvents(const GaitSchedule& gaitSchedule);

  enum class ScheduledAdaptation { None, EarlyContact };
  /**
   * @brief Is Early Contact.
   * @param [in] adaptation: Adaptation.
   * @return True on success.
   */
  static bool IsEarlyContact(ScheduledAdaptation adaptation) { return adaptation == ScheduledAdaptation::EarlyContact; }

  /**
   * Determines the gait adaptation strategy per leg.
   * @param [in] desiredContactFlags: true = this leg was planned to be in contact, false = this was planned to be in swing.
   * @param [in] measuredContactFlags: true = this leg is measured to be in contact, false = this leg is measured to be in swing.
   * @return Scheduled adaptation enum per leg.
   */
  feet_array_t<ScheduledAdaptation> AdvanceLegStrategies(const contact_flag_t& desiredContactFlags,
                                                         const contact_flag_t& measuredContactFlags);
  /**
   * @brief Desired Contact Measured Contact.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  ScheduledAdaptation DesiredContactMeasuredContact(size_t leg);
  /**
   * @brief Desired Contact Measured Motion.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  ScheduledAdaptation DesiredContactMeasuredMotion(size_t leg);
  /**
   * @brief Desired Motion Measured Contact.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  ScheduledAdaptation DesiredMotionMeasuredContact(size_t leg);
  /**
   * @brief Desired Motion Measured Motion.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  ScheduledAdaptation DesiredMotionMeasuredMotion(size_t leg);

  /**
   * @brief Apply Adaptation.
   * @param [in,out] gaitSchedule: Gait Schedule.
   * @param [in] scheduledAdaptations: Scheduled Adaptations.
   */
  void ApplyAdaptation(GaitSchedule& gaitSchedule, const feet_array_t<ScheduledAdaptation>& scheduledAdaptations);

  /// Planned event timing relative to the current time (before applying the gait adaptation).
  feet_array_t<scalar_t> timeUntilNextTouchDownPerLeg_;
  feet_array_t<scalar_t> timeUntilNextLiftOffPerLeg_;

  feet_array_t<bool> hasLiftedSinceLastContact_;

  GaitAdaptationSettings settings_;
};

/**
 * Removes the first swing phase for all legs flagged with earlyTouchdown
 */
std::function<void(scalar_t& currentPhase, switched_model::Gait& currentGait, scalar_t currTime, switched_model::Gait& nextGait)>
/**
 * @brief Early Touch Down Adaptation.
 * @param [in] earlyTouchDownPerLeg: Early Touch Down Per Leg.
 */
EarlyTouchDownAdaptation(const switched_model::feet_array_t<bool>& earlyTouchDownPerLeg);

/**
 * @brief Gets the mode index of the next phase with the specified contact state, returns the size of the modesequence of no such contact state.
 *
 * was found
 * @param [in] contact: Contact.
 * @param [in] startModeIdx: Start Mode Idx.
 * @param [in] leg: Leg.
 * @param [in] gait: Gait.
 * @return Result value.
 */
int GetModeIndexOfNextContactStateOfLeg(bool contact, int startModeIdx, size_t leg, const Gait& gait);

/**
 * @brief Adapts the mode sequence between the two mode ids of a specific leg to be with the specified flag.
 * @param [in] contact: Contact.
 * @param [in] startModeIdx: Start Mode Idx.
 * @param [in] lastModeIdx: Last Mode Idx.
 * @param [in] leg: Leg.
 * @param [in,out] gait: Gait.
 */
void SetContactStateOfLegBetweenModes(bool contact, int startModeIdx, int lastModeIdx, size_t leg, Gait& gait);

}  // namespace switched_model
