#include "bolza/switched_model/initialization/com_kino_initializer.hpp"

#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

ComKinoInitializer::ComKinoInitializer(const com_model_t& comModel, const SwitchedModelModeScheduleManager& modeScheduleManager)
    : comModelPtr_(comModel.clone()), modeScheduleManagerPtr_(&modeScheduleManager) {}

ComKinoInitializer::ComKinoInitializer(const ComKinoInitializer& rhs)
    : bolza::Initializer(rhs), comModelPtr_(rhs.comModelPtr_->clone()), modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

ComKinoInitializer* ComKinoInitializer::Clone() const {
  return new ComKinoInitializer(*this);
}

void ComKinoInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const comkino_state_t comkinoState = state;
  const auto basePose = GetBasePose(comkinoState);
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(time);

  // Initial guess
  input = WeightCompensatingInputs(*comModelPtr_, contactFlags, GetOrientation(basePose));

  nextState = state;
}

}  // namespace switched_model
