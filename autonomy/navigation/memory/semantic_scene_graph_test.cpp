/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/memory/semantic_scene_graph.hpp"

int main()
{
    navigation::memory::SemanticSceneGraph graph;
    navigation::common::Pose2D kitchen;
    kitchen.set_x(2.0);
    navigation::common::Pose2D lobby;
    lobby.set_x(5.0);
    const auto kitchen_id = graph.AddNode("kitchen", kitchen);
    const auto lobby_id = graph.AddNode("lobby", lobby);
    if (graph.Size() != 2 || kitchen_id == lobby_id || !graph.AddEdge(kitchen_id, lobby_id) ||
        graph.AddEdge(kitchen_id, 999))
    {
        std::cerr << "graph mutation failed\n";
        return 1;
    }
    if (graph.Find("kitchen").size() != 1)
    {
        std::cerr << "find failed\n";
        return 1;
    }
    const auto nearest = graph.Nearest("lobby", navigation::common::Pose2D{});
    if (!nearest.has_value() || std::abs(nearest->pose.x() - 5.0) > 1e-9)
    {
        std::cerr << "nearest failed\n";
        return 1;
    }
    return 0;
}
