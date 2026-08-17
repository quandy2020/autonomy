import numpy as np

from autosim.sensors import Sensors
from autosim.simulator import Simulator
from tests.fake_simulator import FakeSimulator


def test_fake_laser_ranges_and_points():
    simulator = FakeSimulator(width=64, height=48)
    simulator.reset(x=0.0, y=0.0, yaw=0.0)
    ranges = simulator.laser_ranges(
        angle_min=-np.pi, angle_max=np.pi, num_beams=36, range_max=10.0
    )
    assert ranges.shape == (36,)
    assert np.allclose(ranges, 5.0)

    points = simulator.lidar_points(
        h_min=-np.pi,
        h_max=np.pi,
        h_beams=8,
        v_min=-0.2,
        v_max=0.2,
        v_rings=2,
        range_max=10.0,
    )
    assert points.ndim == 2 and points.shape[1] == 3
    assert points.shape[0] == 16

    sensors = Sensors(
        angle_min=-np.pi,
        angle_max=np.pi,
        num_beams=36,
        range_min=0.1,
        range_max=10.0,
        lidar_3d={
            "horizontal": {"angle_min": -np.pi, "angle_max": np.pi, "num_beams": 8},
            "vertical": {"angle_min": -0.2, "angle_max": 0.2, "num_rings": 2},
            "range_min": 0.1,
            "range_max": 10.0,
            "noise": 0.0,
        },
    )
    clipped = sensors.sample_laser(simulator)
    assert clipped.shape == (36,)
    cloud = sensors.sample_points(simulator)
    assert cloud.shape == (16, 3)
    color, depth = sensors.sample_camera(simulator)
    assert color.shape[2] == 3
    assert depth.shape == color.shape[:2]


def test_simulator_has_no_mock_backend():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={"habitat": {"path": "", "spawn": [0.0, 0.0, 0.0], "robot": {"urdf": ""}}},
        open_session=False,
    )
    assert not hasattr(simulator, "use_mock")
    assert simulator.session is None
