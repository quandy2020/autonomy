-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Shared navigation parameters referenced by planner, controller, and tasks.

AUTONOMY_COMMON = {
    global_frame = "map",
    robot_base_frame = "base_link",
    odom_topic = "odom",
    transform_tolerance = 0.1,

    default_planner_id = "navfn_planner",
    default_controller_id = "FollowPath",
    default_goal_checker_id = "goal_checker",

    -- Must match BT GoalReached and controller SimpleGoalChecker xy tolerance.
    goal_reached_tolerance = 0.25,

    -- Single-process demo spawn (map frame); matches autonomy_tasks_main defaults.
    demo_robot_spawn_x = 1.0,
    demo_robot_spawn_y = 1.0,
    demo_robot_spawn_yaw = 0.0,
}
