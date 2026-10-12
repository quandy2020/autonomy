/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: farbod
 */

#include "bolza/switched_model/core/com_model_base.hpp"

#include <bolza/switched_model/core/rotations.hpp>

namespace switched_model {

comkino_input_t WeightCompensatingInputs(const ComModelBase<scalar_t>& comModel, const contact_flag_t& contactFlags,
                                         const vector3_t& baseOrientation) {
  return WeightCompensatingInputs(comModel.totalMass(), contactFlags, baseOrientation);
}

comkino_input_t WeightCompensatingInputs(scalar_t mass, const contact_flag_t& contactFlags, const vector3_t& baseOrientation) {
  const int numStanceLegs = NumberOfClosedContacts(contactFlags);

  comkino_input_t inputs = comkino_input_t::Zero();
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = mass * 9.81;
    const vector3_t forceInWorld = vector3_t{0.0, 0.0, totalWeight / numStanceLegs};
    const vector3_t forceInBase = RotateVectorOriginToBase(forceInWorld, baseOrientation);

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      if (contactFlags[i]) {
        inputs.segment<3>(3 * i) = forceInBase;
      }
    }
  }

  return inputs;
}

template class ComModelBase<scalar_t>;
template class ComModelBase<bolza::CppAdInterface::ad_scalar_t>;

}  // end of namespace switched_model
