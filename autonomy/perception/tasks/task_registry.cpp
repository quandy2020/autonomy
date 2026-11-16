#include "autonomy/perception/tasks/task_registry.hpp"

namespace autonomy::perception::base {

const std::array<Task, 8>& RegisteredTasks() {
    static constexpr std::array order{
        Task::Detect, Task::Segment, Task::Classify, Task::Pose,
        Task::Obb, Task::Track, Task::Depth, Task::Semantic};
    return order;
}

proto::TaskKind TaskKindFor(Task task) {
    switch (task) {
        case Task::Detect: return proto::TASK_DETECT;
        case Task::Segment: return proto::TASK_SEGMENT;
        case Task::Classify: return proto::TASK_CLASSIFY;
        case Task::Pose: return proto::TASK_POSE;
        case Task::Obb: return proto::TASK_OBB;
        case Task::Track: return proto::TASK_TRACK;
        case Task::Depth: return proto::TASK_DEPTH;
        case Task::Semantic: return proto::TASK_SEMANTIC;
    }
    return proto::TASK_UNSPECIFIED;
}

}  // namespace autonomy::perception::base
