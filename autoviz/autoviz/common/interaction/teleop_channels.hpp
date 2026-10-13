/*
 * Copyright 2026 The Openbot Authors
 *
 * Task teleop Autolink channels (convention shared with autonomy.task).
 */

/**
 * @file teleop_channels.hpp
 * @brief Canonical Autolink channel names for task teleop goal / feedback.
 *
 * Shared convention with @c autonomy.task so Autoviz teleop UI and the task
 * stack publish / subscribe on the same paths.
 *
 * @see ChannelWriterRegistry
 * @see ChannelReaderRegistry
 */

#pragma once

namespace autoviz {
namespace integration {

/**
 * @brief Autolink channel for teleop goal commands (task → robot / planner).
 *
 * Convention shared with @c autonomy.task.
 */
constexpr char kTeleopGoalChannel[] = "/autonomy/task/teleop/goal";

/**
 * @brief Autolink channel for teleop feedback (robot / planner → UI).
 *
 * Convention shared with @c autonomy.task.
 */
constexpr char kTeleopFeedbackChannel[] = "/autonomy/task/teleop/feedback";

}  // namespace integration
}  // namespace autoviz
