/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "navigation/planning/plugins/policy/simulation/policy.hpp"

int main()
{
    try
    {
        navigation::planning::simulation::Policy policy({});
        navigation::config::PolicyOptions options;
        options.set_name("simulation");
        if (!policy.Configure(options))
        {
            throw std::runtime_error("configuration failed");
        }
        navigation::observation::Observation obs;
        auto* pose = obs.mutable_odom()->mutable_pose()->mutable_pose()->mutable_pose();
        pose->mutable_orientation()->set_w(1);
        auto* depth = obs.mutable_depth();
        depth->set_width(10);
        depth->set_height(10);
        depth->set_encoding("32FC1");
        std::vector<float> blocked(100, 0.3F);
        depth->set_data(reinterpret_cast<const char*>(blocked.data()),
                        blocked.size() * sizeof(float));
        navigation::GoalSpec goal;
        goal.set_type(navigation::GoalSpec::TYPE_POSE);
        goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(2);
        goal.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1);
        const auto action = policy.Query(obs, {}, goal, {}).Value().chunk().actions(0);
        if (action.x() != 0 || std::abs(action.theta()) <= 0)
        {
            throw std::runtime_error("pose goal ignored a blocked front");
        }
        std::cout << "PASS pose-goal front clearance\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
