/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/behavior/intent/click_to_goal.hpp"
#include "navigation/behavior/intent/language_to_goal.hpp"

int main()
{
    automsgs::msgs::sensor_msgs::Image depth;
    depth.set_width(4);
    depth.set_height(4);
    depth.set_encoding("32FC1");
    const float values[16] = {2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F,
                              2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F};
    depth.set_data(reinterpret_cast<const char*>(values), sizeof(values));
    automsgs::msgs::sensor_msgs::CameraInfo camera_info;
    camera_info.add_k(100.0);
    camera_info.add_k(0.0);
    camera_info.add_k(2.0);

    navigation::behavior::ClickToGoal click;
    navigation::GoalSpec goal;
    if (!click.Convert(2, 2, depth, camera_info, navigation::common::Pose2D{}, &goal) ||
        !goal.has_pose() ||
        std::abs(goal.pose().pose().position().x() - 2.0) > 1e-9)
    {
        std::cerr << "click goal failed\n";
        return 1;
    }

    navigation::memory::SemanticSceneGraph graph;
    navigation::common::Pose2D kitchen;
    kitchen.set_x(3.0);
    graph.AddNode("kitchen", kitchen);
    navigation::behavior::LanguageToGoal language;
    if (!language.Convert("kitchen", graph, navigation::common::Pose2D{}, &goal) ||
        std::abs(goal.pose().pose().position().x() - 3.0) > 1e-9)
    {
        std::cerr << "language goal failed\n";
        return 1;
    }
    if (language.Convert("unknown", graph, navigation::common::Pose2D{}, &goal))
    {
        std::cerr << "unknown label accepted\n";
        return 1;
    }
    return 0;
}
