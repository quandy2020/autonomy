#pragma once

/**
 * @file com_kino_initializer.hpp
 * @brief com kino initializer (bolza).
 */


#include <bolza/core/initialization/initializer.hpp>

#include "bolza/switched_model/core/com_model_base.hpp"
#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/logic/switched_model_mode_schedule_manager.hpp"

namespace switched_model {

class ComKinoInitializer : public bolza::Initializer {
 public:
  using com_model_t = ComModelBase<scalar_t>;

  /**
   * @brief Constructs a ComKinoInitializer.
   * @param [in] comModel: Com Model.
   * @param [in] modeScheduleManager: Mode Schedule Manager.
   */
  ComKinoInitializer(const com_model_t& comModel, const SwitchedModelModeScheduleManager& modeScheduleManager);

  /**
   * @brief Destructor.
   */
  ~ComKinoInitializer() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  ComKinoInitializer* Clone() const override;

  /**
   * @brief Compute.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] nextTime: Next Time.
   * @param [in,out] input: Input.
   * @param [in,out] nextState: Next State.
   */
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 protected:
  /**
   * @brief Constructs a ComKinoInitializer.
   * @param [in] rhs: Rhs.
   */
  ComKinoInitializer(const ComKinoInitializer& rhs);

 private:
  std::unique_ptr<com_model_t> comModelPtr_;
  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
};

}  // end of namespace switched_model
