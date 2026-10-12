/*
 * GaitAdaptation.cpp
 *
 *  Created on: Jun 4, 2020
 *      Author: Marko Bjelonic
 */

// anymal ctrl ocs2
#include "bolza/switched_model/logic/gait_adaptation.hpp"

#include "bolza/switched_model/core/motion_phase_definition.hpp"
#include "bolza/switched_model/logic/single_leg_logic.hpp"

namespace switched_model {

GaitAdaptation::GaitAdaptation(const GaitAdaptationSettings& settings, const contact_flag_t& measuredContactFlags)
    : timeUntilNextTouchDownPerLeg_(ConstantFeetArray<scalar_t>(0.0)),
      timeUntilNextLiftOffPerLeg_(ConstantFeetArray<scalar_t>(0.0)),
      hasLiftedSinceLastContact_(ConstantFeetArray<bool>(true)),  // initialize at true to enable immediate early touchdown
      settings_(settings) {}

void GaitAdaptation::Advance(GaitSchedule& gaitSchedule, const contact_flag_t& measuredContactFlags, scalar_t dt) {
  const auto desiredContactFlags = ModeNumber2StanceLeg(gaitSchedule.getCurrentMode());

  // Update tracking if lift off happened
  AdvanceLiftoffTracking(desiredContactFlags, measuredContactFlags);

  // Update internal knowledge of next touchdown / liftoff
  AdvanceSwingEvents(gaitSchedule);

  // Decide on a strategy per leg and schedule adaptation
  const auto& scheduledAdaptation = AdvanceLegStrategies(desiredContactFlags, measuredContactFlags);

  // Apply all scheduled
  ApplyAdaptation(gaitSchedule, scheduledAdaptation);
}

void GaitAdaptation::AdvanceLiftoffTracking(const contact_flag_t& desiredContactFlags, const contact_flag_t& measuredContactFlags) {
  // Lift off tracking resets to false whenever there is a planned + measured contact. The flag changes to true any time there is a measured
  // swing phase since then.
  for (size_t leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    if (desiredContactFlags[leg] && measuredContactFlags[leg]) {
      hasLiftedSinceLastContact_[leg] = false;
    } else if (!measuredContactFlags[leg]) {
      hasLiftedSinceLastContact_[leg] = true;
    }
  }
}

void GaitAdaptation::AdvanceSwingEvents(const GaitSchedule& gaitSchedule) {
  // Only interested in events within the specified windows
  const auto checkHorizon = settings_.earlyTouchDownTimeWindow;

  // Extract contact timings from the gait schedule
  const auto modeSchedule = gaitSchedule.getModeSchedule(checkHorizon);
  const auto contactTimingsPerLeg = switched_model::ExtractContactTimingsPerLeg(modeSchedule);
  const auto currentTime = gaitSchedule.getCurrentTime();

  for (size_t leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    timeUntilNextTouchDownPerLeg_[leg] = GetTimeOfNextTouchDown(currentTime, contactTimingsPerLeg[leg]) - currentTime;
    timeUntilNextLiftOffPerLeg_[leg] = GetTimeOfNextLiftOff(currentTime, contactTimingsPerLeg[leg]) - currentTime;
  }
}

auto GaitAdaptation::AdvanceLegStrategies(const contact_flag_t& desiredContactFlags, const contact_flag_t& measuredContactFlags)
    -> feet_array_t<ScheduledAdaptation> {
  auto adaptations = ConstantFeetArray<ScheduledAdaptation>(ScheduledAdaptation::None);

  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    bool desiredContact = desiredContactFlags[leg];
    bool measuredContact = measuredContactFlags[leg];

    if (desiredContact) {
      if (measuredContact) {
        // Leg was planned to be in contact, and is measured to be in contact
        adaptations[leg] = DesiredContactMeasuredContact(leg);
      } else {
        // Leg was planned to be in contact, but is measured to be in swing
        adaptations[leg] = DesiredContactMeasuredMotion(leg);
      }
    } else {
      if (measuredContact) {
        // Leg was planned to be in swing, but is measured to be in contact
        adaptations[leg] = DesiredMotionMeasuredContact(leg);
      } else {
        // Leg was planned to be in swing, and is measured to be in swing
        adaptations[leg] = DesiredMotionMeasuredMotion(leg);
      }
    }
  }

  return adaptations;
}

auto GaitAdaptation::DesiredContactMeasuredContact(size_t leg) -> ScheduledAdaptation {
  return ScheduledAdaptation::None;
}

auto GaitAdaptation::DesiredContactMeasuredMotion(size_t leg) -> ScheduledAdaptation {
  return ScheduledAdaptation::None;
}

auto GaitAdaptation::DesiredMotionMeasuredContact(size_t leg) -> ScheduledAdaptation {
  if (hasLiftedSinceLastContact_[leg] && !std::isnan(timeUntilNextTouchDownPerLeg_[leg]) &&
      timeUntilNextTouchDownPerLeg_[leg] < settings_.earlyTouchDownTimeWindow) {
    // Touchdown was planned to be soon -> Take the contact early
    return ScheduledAdaptation::EarlyContact;
  } else {
    return ScheduledAdaptation::None;
  }
}

auto GaitAdaptation::DesiredMotionMeasuredMotion(size_t leg) -> ScheduledAdaptation {
  return ScheduledAdaptation::None;
}

void GaitAdaptation::ApplyAdaptation(GaitSchedule& gaitSchedule, const feet_array_t<ScheduledAdaptation>& scheduledAdaptations) {
  // Early contact
  if (std::any_of(scheduledAdaptations.begin(), scheduledAdaptations.end(), isEarlyContact)) {
    const auto earlyTouchDownPerLeg = ApplyPerLeg(isEarlyContact, scheduledAdaptations);
    gaitSchedule.adaptCurrentGait(EarlyTouchDownAdaptation(earlyTouchDownPerLeg));
  }
}

std::function<void(scalar_t& currentPhase, switched_model::Gait& currentGait, scalar_t currTime, switched_model::Gait& nextGait)>
EarlyTouchDownAdaptation(const switched_model::feet_array_t<bool>& earlyTouchDownPerLeg) {
  return [=](scalar_t& currentPhase, Gait& currentGait, scalar_t currTime, Gait& nextGait) {
    const int currentModeIndex = GetModeIndexFromPhase(currentPhase, currentGait);
    for (unsigned int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      if (earlyTouchDownPerLeg[leg]) {
        const int modeIndexOfNextContact = GetModeIndexOfNextContactStateOfLeg(true, currentModeIndex, leg, currentGait);

        if (modeIndexOfNextContact < currentGait.modeSequence.size()) {  // update current gait
          SetContactStateOfLegBetweenModes(true, currentModeIndex, modeIndexOfNextContact, leg, currentGait);
        } else {  // update current gait till end, and next gait
          SetContactStateOfLegBetweenModes(true, currentModeIndex, modeIndexOfNextContact - 1, leg, currentGait);

          int modeIndexOfNextContactOfNextGait = GetModeIndexOfNextContactStateOfLeg(true, 0, leg, nextGait);
          if (modeIndexOfNextContactOfNextGait == nextGait.modeSequence.size()) {
            --modeIndexOfNextContactOfNextGait;
          }
          SetContactStateOfLegBetweenModes(true, 0, modeIndexOfNextContactOfNextGait, leg, nextGait);
        }
      }
    }
  };
}

int GetModeIndexOfNextContactStateOfLeg(bool contact, int startModeIdx, size_t leg, const Gait& gait) {
  int modeIndex = startModeIdx;
  while (modeIndex < gait.modeSequence.size()) {
    size_t currentMode = gait.modeSequence[modeIndex];
    if (ModeNumber2StanceLeg(currentMode)[leg] == contact) {
      break;
    } else {
      ++modeIndex;
    }
  }
  return modeIndex;
}

void SetContactStateOfLegBetweenModes(bool contact, int startModeIdx, int lastModeIdx, size_t leg, Gait& gait) {
  for (int modeIndex = startModeIdx; modeIndex <= lastModeIdx; ++modeIndex) {
    auto stanceLegs = switched_model::ModeNumber2StanceLeg(gait.modeSequence[modeIndex]);
    stanceLegs[leg] = contact;
    gait.modeSequence[modeIndex] = switched_model::StanceLeg2ModeNumber(stanceLegs);
  }
}

}  // namespace switched_model
