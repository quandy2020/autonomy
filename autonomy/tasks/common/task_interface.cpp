/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/common/task_interface.hpp"

#include "autonomy/tasks/options.hpp"

namespace autonomy {
namespace tasks {
namespace common {

proto::TaskOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    return autonomy::tasks::LoadOptions(parameter_dictionary);
}

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
