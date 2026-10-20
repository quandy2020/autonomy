# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""HuNav scenario loader + ComputeAgents engine (Autolink / automsgs)."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from autosim.scene.hunav.client import call_compute_agents
from autosim.scene.hunav.codec import AgentCodec, BEH_NAME_TO_TYPE
from autosim.scene.hunav.compute import ComputeAgentsEngine
from autosim.scene.hunav.loader import load_hunav_scenario, unwrap_hunav_params
from autosim.scene.hunav.service import HunavServiceHost

ROOT = Path(__file__).resolve().parents[1]
WAREHOUSE = ROOT / "config/scenarios/hunav/warehouse_agents.yaml"


def test_unwrap_hunav_params():
    raw = {
        "hunav_loader": {
            "ros__parameters": {"agents": ["a1"], "a1": {"id": 1}},
        }
    }
    params = unwrap_hunav_params(raw)
    assert params["agents"] == ["a1"]
    assert unwrap_hunav_params({"agents": []})["agents"] == []


def test_load_warehouse_hunav_yaml():
    agents = load_hunav_scenario(WAREHOUSE)
    assert agents.count >= 3
    record = agents.records[0]
    assert "x" in record and "goals" in record
    assert len(record["goals"]) >= 1
    assert record["behavior"] in {
        "regular",
        "impassive",
        "surprised",
        "scared",
        "curious",
        "threatening",
    }
    assert record["name"].startswith("agent")
    assert int(record["id"]) >= 1


def test_pedestrian_loads_hunav_agents_file():
    from autosim.scene.agents.pedestrian import Pedestrian

    ped = Pedestrian(
        {
            "enabled": True,
            "social": True,
            "agents": str(WAREHOUSE),
            "x_length": 40.0,
            "y_length": 40.0,
        }
    )
    assert ped.position.shape[0] >= 3
    assert ped.social is True
    assert len(ped.agent_ids) == ped.position.shape[0]
    assert len(ped.agent_names) == ped.position.shape[0]
    assert int(ped.agent_ids[0]) == 1
    assert str(ped.agent_names[0]) == "agent1"


def test_compute_agents_engine_moves_people():
    codec = AgentCodec()
    if not codec.available:
        pytest.skip("automsgs hunav_msgs stubs not generated yet")
    agents = load_hunav_scenario(WAREHOUSE)
    engine = ComputeAgentsEngine({"x_length": 40.0, "y_length": 40.0})
    engine.pedestrian.spawn_agents(agents)
    before = engine.pedestrian.position.copy()
    msg = codec.pack_agents(engine.pedestrian)
    assert msg.agents[0].name == "agent1"
    assert len(msg.agents[0].goals) >= 1
    robot = codec.pack_robot(0.0, 0.0, 0.0)
    updated = engine.step(msg, robot, dt=0.1)
    assert len(updated.agents) == before.shape[0]
    after = np.array(
        [[a.position.position.x, a.position.position.y] for a in updated.agents]
    )
    # At least one agent should move toward its goal in 0.1 s.
    assert np.linalg.norm(after - before) > 1e-4


def test_codec_roundtrip_apply_agents():
    codec = AgentCodec()
    if not codec.available:
        pytest.skip("automsgs hunav_msgs stubs not generated yet")
    agents = load_hunav_scenario(WAREHOUSE)
    engine = ComputeAgentsEngine({"x_length": 40.0, "y_length": 40.0})
    engine.pedestrian.spawn_agents(agents)
    msg = codec.pack_agents(engine.pedestrian, stamp=(1, 2), frame_id="map")
    assert msg.header.stamp.sec == 1
    engine.pedestrian.position[:] = 0.0
    codec.apply_agents(engine.pedestrian, msg)
    assert abs(float(engine.pedestrian.position[0, 0]) - float(msg.agents[0].position.position.x)) < 1e-6


def test_service_host_compute_agents_inprocess():
    codec = AgentCodec()
    if not codec.available:
        pytest.skip("automsgs hunav_msgs stubs not generated yet")
    from automsgs.srvs import compute_agents_pb2

    agents = load_hunav_scenario(WAREHOUSE)
    engine = ComputeAgentsEngine({"x_length": 40.0, "y_length": 40.0})
    engine.pedestrian.spawn_agents(agents)
    registered = {}

    class FakeNode:
        def create_service(self, channel, callback, msg_type):
            registered[channel] = (callback, msg_type)
            return SimpleNamespace(channel=channel)

    host = HunavServiceHost(FakeNode(), engine)
    assert host.available
    assert host.start()
    assert "/compute_agents" in registered
    cb, msg_type = registered["/compute_agents"]
    assert msg_type is compute_agents_pb2.ComputeAgents

    req = compute_agents_pb2.ComputeAgents()
    req.request.current_agents.CopyFrom(codec.pack_agents(engine.pedestrian))
    req.request.robot.CopyFrom(codec.pack_robot(0.0, 0.0, 0.0))
    req.request.dt = 0.05
    out = cb(req)
    assert out.response.success
    assert len(out.response.updated_agents.agents) == engine.pedestrian.position.shape[0]
    host.stop()


def test_call_compute_agents_client_helper():
    codec = AgentCodec()
    if not codec.available:
        pytest.skip("automsgs hunav_msgs stubs not generated yet")
    from automsgs.srvs import compute_agents_pb2

    agents = load_hunav_scenario(WAREHOUSE)
    engine = ComputeAgentsEngine({"x_length": 40.0, "y_length": 40.0})
    engine.pedestrian.spawn_agents(agents)
    msg = codec.pack_agents(engine.pedestrian)
    robot = codec.pack_robot(1.0, 2.0, 0.3)

    class FakeClient:
        def send_request(self, envelope, timeout_sec=1.0):
            updated = engine.step(
                envelope.request.current_agents,
                envelope.request.robot,
                dt=float(envelope.request.dt),
            )
            out = compute_agents_pb2.ComputeAgents()
            out.response.updated_agents.CopyFrom(updated)
            out.response.success = True
            return out

    updated = call_compute_agents(FakeClient(), msg, robot, dt=0.1)
    assert updated is not None
    assert len(updated.agents) == msg.agents.__len__()


def test_beh_enum_matches_hunav():
    assert BEH_NAME_TO_TYPE["regular"] == 1
    assert BEH_NAME_TO_TYPE["threatening"] == 6
    from autosim.scene.agents.behavior import Behavior

    assert Behavior.kind_name("BEH_REGULAR") == "regular"
    assert Behavior.kind_name("Regular") == "regular"


def test_hunav_fields_roundtrip_skin_group_force():
    codec = AgentCodec()
    if not codec.available:
        pytest.skip("automsgs hunav_msgs stubs not generated yet")
    agents = load_hunav_scenario(WAREHOUSE)
    assert "skin" in agents.records[0]
    assert "group_id" in agents.records[0]
    engine = ComputeAgentsEngine({"x_length": 40.0, "y_length": 40.0})
    engine.pedestrian.spawn_agents(agents)
    ped = engine.pedestrian
    assert ped.skins.shape[0] == ped.position.shape[0]
    assert ped.group_ids[0] == -1
    assert float(ped.goal_force_factor[0]) == pytest.approx(10.0)
    msg = codec.pack_agents(ped)
    assert msg.agents[0].skin == int(ped.skins[0])
    assert msg.agents[0].group_id == -1
    assert msg.agents[0].behavior.goal_force_factor == pytest.approx(10.0)
    assert msg.agents[0].radius == pytest.approx(0.4)


def test_validate_hunav_settings():
    from autosim.runtime.settings import Config

    Config.validate_hunav(None)
    Config.validate_hunav({"service": True, "publish_agents": True})
    with pytest.raises(ValueError):
        Config.validate_hunav({"service": "yes"})
    with pytest.raises(ValueError):
        Config.validate_hunav({"agents": "/no/such/hunav_agents.yaml"})


def test_traffic_skip_pedestrian_step():
    from autosim.scene.traffic import Traffic

    traffic = Traffic.from_block(
        {
            "obstacle": {"enabled": False},
            "pedestrian": {
                "enabled": True,
                "social": True,
                "count": 1,
                "x_length": 10.0,
                "y_length": 10.0,
            },
        }
    )
    before = traffic.pedestrian.position.copy()
    traffic.skip_pedestrian_step = True
    traffic.step(0.1, robot=(0.0, 0.0, 0.0))
    assert np.allclose(traffic.pedestrian.position, before)
