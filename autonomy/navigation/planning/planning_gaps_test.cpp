/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/planning_options.hpp"
#include "navigation/planning/tasks/relocalization_task.hpp"
#include "navigation/planning/tasks/semantic_search_task.hpp"

int main()
{
    using namespace navigation;  // NOLINT

    config::NavigationConfig configuration;
    configuration.mutable_policy()->set_name("vntpa");
    auto* vntpa = configuration.mutable_policy()->mutable_vntpa();
    vntpa->set_action_chunk_length(8);
    vntpa->set_execute_length(2);
    vntpa->set_decision_rate_hz(5.0);
    vntpa->set_action_dt_s(0.1);
    configuration.mutable_scenario()->set_default_scenario("destination");
    planning::PlanningOptions options(configuration);
    std::string error;
    if (!options.Validate(&error))
    {
        std::cerr << "valid options rejected: " << error << "\n";
        return 1;
    }
    configuration.mutable_policy()->mutable_vntpa()->set_execute_length(9);
    if (planning::PlanningOptions(configuration).Validate(&error))
    {
        std::cerr << "invalid execute_length accepted\n";
        return 1;
    }

    memory::SemanticSceneGraph graph;
    common::Pose2D kitchen;
    kitchen.set_x(4.0);
    graph.AddNode("kitchen", kitchen);
    planning::SemanticSearchTask search;
    search.SetGraph(&graph);
    search.SetLabel("kitchen");
    planning::NavigationFrame frame;
    if (!search.Execute(frame).Ok() || !frame.goal.has_pose() ||
        frame.goal.pose().pose().position().x() != 4.0)
    {
        std::cerr << "semantic planning task failed\n";
        return 1;
    }

    planning::RelocalizationTask relocalization;
    planning::NavigationFrame degraded;
    if (relocalization.Execute(degraded).Ok() || !degraded.degraded)
    {
        std::cerr << "relocalization trigger failed\n";
        return 1;
    }
    return 0;
}
