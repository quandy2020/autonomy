#ifndef AUTONOMY_PERCEPTION_TASKS_TASK_REGISTRY_HPP_
#define AUTONOMY_PERCEPTION_TASKS_TASK_REGISTRY_HPP_

#include <array>
#include "autonomy/perception/proto/perception.pb.h"
#include "autonomy/perception/tasks/task_base.hpp"

namespace autonomy::perception::base {

const std::array<Task, 8>& RegisteredTasks();
proto::TaskKind TaskKindFor(Task task);

}  // namespace autonomy::perception::base

#endif
