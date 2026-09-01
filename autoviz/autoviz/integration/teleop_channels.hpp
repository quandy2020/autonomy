/*
 * Copyright 2026 The Openbot Authors
 *
 * Task teleop Autolink channels (convention shared with autonomy.task).
 */

#pragma once

namespace autoviz {
namespace integration {

constexpr char kTeleopGoalChannel[] = "/autonomy/task/teleop/goal";
constexpr char kTeleopFeedbackChannel[] = "/autonomy/task/teleop/feedback";

}  // namespace integration
}  // namespace autoviz
