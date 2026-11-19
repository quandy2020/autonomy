"""Commands move each robot, keep odom/TF in sync and stop on timeout."""
import math
from pathlib import Path
import pytest
from autosim.runtime import Config,Runner
from autosim.robot.urdf import UrdfModel
from autosim.bridge.messages import Messages
from tests.fake_simulator import FakeSimulator
from tests.test_runner_smoke import FakeLink

PROFILES=('google_robot','unitree_g1','unitree_go2')


@pytest.mark.parametrize('profile',PROFILES)
def test_cmd_vel_motion_odom_tf_watchdog(profile):
    data=Config.load(UrdfModel.package_root()/'config/robots'/f'{profile}.yaml').data
    data['habitat']['sensors']['chase']['enabled']=False
    sim=FakeSimulator();sim.urdf=UrdfModel.load(data['habitat']['robot'].get('urdf') or f'robot/{profile}/{profile}.urdf')
    runner=Runner(Config.load(data),link=FakeLink(),simulator=sim)
    runner.sensor_worker.start()
    def command(v,w):
        reader=runner.bridge.command_reader
        reader.pending_message=Messages.encode_twist_stamped(v,w,runner.clock.stamp())
        reader.has_message=True
    try:
        for v,w,expected in [(.3,0,'forward'),(-.3,0,'reverse'),(0,.6,'turn'),(.2,.4,'arc')]:
            start=sim.pose()
            for _ in range(100):command(v,w);runner.cycle(.02)
            end=sim.pose()
            assert sim.pose()==pytest.approx(runner.robot.pose())
            distance=math.hypot(end[0]-start[0],end[1]-start[1])
            if expected=='turn':assert abs(end[2]-start[2])>.5
            else:assert distance>.25
            assert 'odom' in runner.bridge.writers
            odom=runner.node.writers['/odom'].msgs[-1]
            pos=odom.pose.pose.pose.position
            assert (pos.x,pos.y)==pytest.approx(end[:2])
            tf=runner.node.writers['/tf'].msgs[-1]
            root=next(t for t in tf.transforms if t.header.frame_id=='odom' and t.child_frame_id==sim.urdf.root)
            assert (root.transform.translation.x,root.transform.translation.y)==pytest.approx(end[:2])
        for _ in range(80):runner.cycle(.02)
        assert runner.robot.velocity()==pytest.approx((0,0))
        stopped=sim.pose()
        for _ in range(10):runner.cycle(.02)
        assert sim.pose()==pytest.approx(stopped)
    finally:
        runner.sensor_worker.stop()


def test_nonfinite_cmd_vel_rejected():
    from autosim.robot.plant import Robot
    plant=Robot(.5,1,.5)
    with pytest.raises(ValueError):plant.set_twist(math.nan,0,0)
    assert plant.cmd_linear==0
