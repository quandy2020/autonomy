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
    default_progress_checker_id = "progress_checker",
    default_smoother_id = "simple_smoother",

    -- Must match BT GoalReached (goal_reached_tol) and controller goal checker.
    goal_reached_tolerance = 0.25,
}
