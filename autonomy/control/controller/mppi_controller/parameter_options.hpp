/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/control/proto/mppi_controller.pb.h"

namespace autonomy {
namespace common {
class LuaParameterDictionary;
}
namespace control {
namespace controller {
namespace mppi {

proto::MPPIControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
