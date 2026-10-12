#include <gtest/gtest.h>

#include <bolza/pinocchio/interface/pinocchio_interface.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>
#include "CartPoleUrdf.h"

TEST(testPinocchioInterface, buildFromXml) {
  auto pinocchio = bolza::GetPinocchioInterfaceFromUrdfString(cartPoleUrdf);
  std::cout << pinocchio;
}
