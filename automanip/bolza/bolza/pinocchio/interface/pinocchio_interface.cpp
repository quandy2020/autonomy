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

#include <iomanip>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"
#include "bolza/pinocchio/interface/implementation/pinocchio_interface.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
template <>
PinocchioInterfaceCppAd PinocchioInterface::ToCppAd() const {
  auto cppAdModel = GetModel().cast<ad_scalar_t>();

  // TODO (rgrandia) : remove after bug fix. The cast function forgets to copy this member.
  cppAdModel.supports = GetModel().supports;

  return PinocchioInterfaceCppAd(cppAdModel);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// explicit instantiation
template class PinocchioInterfaceTpl<scalar_t>;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& os, const PinocchioInterface& p) {
  const auto& model = p.getModel();
  os << "model.nv = " << model.nv << '\n';
  os << "model.nq = " << model.nq << '\n';
  os << "model.njoints = " << model.njoints << '\n';
  os << "model.nbodies = " << model.nbodies << '\n';
  os << "model.nframes = " << model.nframes << '\n';

  os << "\nJoints:\n";
  for (int k = 0; k < model.njoints; ++k) {
    os << std::setw(20) << model.names[k] << ":  ";
    os << " ID = " << k;
    os << '\n';
  }

  os << "\nFrames:\n";
  for (int k = 0; k < model.nframes; ++k) {
    os << std::setw(20) << model.frames[k].name << ":  ";
    os << " ID = " << k;
#if PINOCCHIO_VERSION_AT_LEAST(3, 0, 0)
    os << ", parent = " << model.frames[k].parentJoint;
#else
    os << ", parent = " << model.frames[k].parent;
#endif
    os << ", type = ";

    std::string frameType;
    if ((model.frames[k].type & pinocchio::FrameType::OP_FRAME) != 0) {
      frameType += "OP_FRAME ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::JOINT) != 0) {
      frameType += "JOINT ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::FIXED_JOINT) != 0) {
      frameType += "FIXED_JOINT ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::BODY) != 0) {
      frameType += "BODY ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::SENSOR) != 0) {
      frameType += "SENSOR ";
    }
    os << "\"" << frameType << "\"\n";
  }
  return os;
}

}  // namespace bolza
