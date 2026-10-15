# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.humans import AgentState, SocialForceModel


def test_social_force_moves_agents() -> None:
    model = SocialForceModel()
    agents = [
        AgentState(x=0.0, y=0.0, goal_x=3.0, goal_y=0.0),
        AgentState(x=1.0, y=0.2, goal_x=3.0, goal_y=0.0),
    ]
    x0 = agents[0].x
    model.step(agents, robot_xy=(-1.0, 0.0), dt=0.1)
    assert agents[0].x != x0
