//
// Created by rgrandia on 15.03.20.
//

#include "bolza/switched_model/logic/gait_schedule.hpp"

#include "bolza/switched_model/core/motion_phase_definition.hpp"
#include "bolza/switched_model/logic/gait_switching.hpp"

namespace switched_model {

GaitSchedule::GaitSchedule(scalar_t time, Gait gait) : time_(time), phase_(0.0), gaitSchedule_{std::move(gait)} {}

void GaitSchedule::AdvanceToTime(scalar_t time) {
  if (time <= time_) {
    return;
  }

  const scalar_t dt = time - time_;
  std::deque<Gait>::iterator newActiveGait;
  std::tie(phase_, newActiveGait) = AdvancePhase(phase_, dt, gaitSchedule_.begin(), gaitSchedule_.end());
  time_ = time;

  // Remove gaits that have been completed.
  while (newActiveGait != gaitSchedule_.begin()) {
    gaitSchedule_.pop_front();
  }
}

void GaitSchedule::SetNextGait(const Gait& gait) {
  SetGaitSequenceAfterCurrentGait({gait});
}

void GaitSchedule::SetGaitSequenceAfterCurrentGait(const GaitSequence& gaitSequence) {
  gaitSchedule_.erase(gaitSchedule_.begin() + 1, gaitSchedule_.end());
  gaitSchedule_.insert(gaitSchedule_.end(), gaitSequence.begin(), gaitSequence.end());
}

void GaitSchedule::SetGaitAtTime(const Gait& gait, scalar_t time) {
  SetGaitSequenceAtTime({gait}, time);
}

void GaitSchedule::SetGaitSequenceAtTime(const GaitSequence& gaitSequence, scalar_t time) {
  assert(time >= time_);

  RolloutGaitScheduleTillTime(time);

  scalar_t newPhase;
  std::deque<Gait>::iterator newActiveGait;
  std::tie(newPhase, newActiveGait) = AdvancePhase(phase_, time - time_, gaitSchedule_.begin(), gaitSchedule_.end());

  // Shrink the gait that is active at "time", s.t. it ends at "time"
  if (newActiveGait == gaitSchedule_.begin()) {
    // Gait to shrink is already active. Determine new duration from the amount of time and phase left.
    newActiveGait->duration = (time - time_) / (1.0 - phase_);
  } else {
    newActiveGait->duration *= newPhase;
  }

  // Check if duration was set to zero
  if (newActiveGait->duration > 0.0) {  // keep newActiveGait and delete the ones after
    gaitSchedule_.erase(newActiveGait + 1, gaitSchedule_.end());
  } else {  // delete newActiveGait and the ones after
    gaitSchedule_.erase(newActiveGait, gaitSchedule_.end());
  }

  gaitSchedule_.insert(gaitSchedule_.end(), gaitSequence.begin(), gaitSequence.end());
}

void GaitSchedule::SetGaitAfterTime(const Gait& gait, scalar_t time) {
  SetGaitSequenceAfterTime({gait}, time);
}

void GaitSchedule::SetGaitSequenceAfterTime(const GaitSequence& gaitSequence, scalar_t time) {
  assert(time >= time_);

  RolloutGaitScheduleTillTime(time);

  scalar_t newPhase;
  std::deque<Gait>::iterator newActiveGait;
  std::tie(newPhase, newActiveGait) = AdvancePhase(phase_, time - time_, gaitSchedule_.begin(), gaitSchedule_.end());

  gaitSchedule_.erase(newActiveGait + 1, gaitSchedule_.end());
  gaitSchedule_.insert(gaitSchedule_.end(), gaitSequence.begin(), gaitSequence.end());
}

void GaitSchedule::AdaptCurrentGait(
    const std::function<void(scalar_t& currentPhase, Gait& currentGait, scalar_t currTime, Gait& nextGait)>& gaitAdaptor) {
  // Repeat the last gait until we have 3 gaits in the schedule. This prevents that the gaitAdaptor modifies the nominal gait.
  while (gaitSchedule_.size() < 3) {
    gaitSchedule_.push_back(gaitSchedule_.back());
  }

  // Apply gait adaptation to the current and next gait
  gaitAdaptor(phase_, gaitSchedule_.front(), time_, gaitSchedule_[1]);
}

bolza::ModeSchedule GaitSchedule::GetModeSchedule(scalar_t timeHorizon) const {
  return ::switched_model::GetModeSchedule(phase_, time_, timeHorizon, gaitSchedule_.begin(), gaitSchedule_.end());
}

void GaitSchedule::RolloutGaitScheduleTillTime(scalar_t time) {
  scalar_t tGaitEnd = time_ + TimeLeftInGait(GetCurrentPhase(), GetCurrentGait());
  auto gaitIt = gaitSchedule_.begin();
  while (tGaitEnd <= time) {
    if (std::next(gaitIt) == gaitSchedule_.end()) {
      // End of the schedule reached: make the repetition of the last gait explicit
      gaitSchedule_.push_back(gaitSchedule_.back());
      gaitIt = std::prev(gaitSchedule_.end(), 2);  // Iterator can be invalidated after push_back, so reset set explicitly.
    }
    tGaitEnd += gaitIt->duration;
    ++gaitIt;
  }
}

bool IsStandingDuringTimeHorizon(scalar_t timeHorizon, const GaitSchedule& gaitSchedule) {
  const auto modeSchedule = gaitSchedule.GetModeSchedule(timeHorizon);
  return std::all_of(modeSchedule.modeSequence.begin(), modeSchedule.modeSequence.end(),
                     [](size_t mode) { return mode == ModeNumber::STANCE; });
}

bool IsStanding(const GaitSchedule& gaitSchedule) {
  const auto& modeSequence = gaitSchedule.GetCurrentGait().modeSequence;
  return std::all_of(modeSequence.begin(), modeSequence.end(), [](size_t mode) { return mode == ModeNumber::STANCE; });
}

}  // namespace switched_model