import numpy as np

from autosim.world import MockWorld
from autosim.lidar import Lidar
from autosim.rgbd import Rgbd


def test_mock_world_step_and_sensors():
    world = MockWorld(width=64, height=48)
    world.reset(x=0.0, y=0.0, yaw=0.0)
    world.set_pose(0.1, 0.0, 0.0)
    world.step()
    lidar = Lidar(angle_min=-np.pi, angle_max=np.pi, num_beams=36, range_min=0.1, range_max=10.0)
    ranges = lidar.sample(world)
    assert ranges.shape == (36,)
    assert np.all(ranges >= 0.1)
    rgbd = Rgbd()
    rgb, depth = rgbd.sample(world)
    assert rgb.shape[2] == 3
    assert depth.shape == rgb.shape[:2]
