"""Joint control contract: offsets, atomic validation, mimic and live transforms."""
import math
from types import SimpleNamespace
import numpy as np
import pytest
from autosim.robot.joints import Joint, complete_positions, joint_transform, load_joints
from autosim.backends.habitat.articulation import Articulation


class FakeAO:
    # Deliberately different link, position and velocity orders.
    def __init__(self):
        self.joint_positions = np.array([0., 77., 0.])
        self.joint_velocities = np.array([0., 0.])
    def get_link_ids(self): return [4,2,8]
    def get_link_num_joint_pos(self, link): return 0 if link == 8 else 1
    def get_link_joint_name(self, link): return {4:'arm',2:'slide'}[link]
    def get_link_joint_pos_offset(self, link): return {4:2,2:0}[link]
    def get_link_dof_offset(self, link): return {4:0,2:1}[link]


def test_offset_mapping_and_partial_atomic_updates():
    robot=FakeAO()
    controller=Articulation(robot, {'arm':Joint('arm','revolute','a',-1,1),
                                 'slide':Joint('slide','prismatic','b',0,.5)})
    controller.set_positions({'arm':.6,'slide':.2})
    np.testing.assert_allclose(robot.joint_positions,[.2,77,.6])
    robot.joint_velocities=np.array([.4,.7])
    assert controller.velocities()=={'arm':.4,'slide':.7}
    before=robot.joint_positions.copy()
    for bad in ({'arm':.9,'slide':2}, {'arm':math.nan}, {'unknown':0}):
        with pytest.raises(ValueError): controller.set_positions(bad)
        np.testing.assert_array_equal(robot.joint_positions,before)
    controller.set_positions({'arm':-.3})
    assert controller.positions()['slide']==pytest.approx(.2)


def test_mimic_chain_and_continuous_rotation():
    joints={'wheel':Joint('wheel','continuous','a'),
            'finger':Joint('finger','revolute','b',-10,10,'wheel',-.5,.1),
            'tip':Joint('tip','revolute','c',-10,10,'finger',1,.2)}
    result=complete_positions(joints,{'wheel':4*math.pi})
    assert result['finger']==pytest.approx(-2*math.pi+.1)
    assert result['tip']==pytest.approx(-2*math.pi+.3)
    with pytest.raises(ValueError,match='cycle'):
        complete_positions({'a':Joint('a','revolute','x',mimic='a')},{})


def test_prismatic_motion_respects_origin_rotation():
    xyz,q=joint_transform((1,2,3),(0,0,math.pi/2),(1,0,0),'prismatic',.5)
    np.testing.assert_allclose(xyz,[1,2.5,3],atol=1e-8)
    np.testing.assert_allclose(q,[0,0,math.sqrt(.5),math.sqrt(.5)],atol=1e-8)
    xyz,q=joint_transform((0,0,0),(math.pi/2,0,0),(0,1,0),'revolute',math.pi)
    np.testing.assert_allclose(np.abs(q),[0,math.sqrt(.5),math.sqrt(.5),0],atol=1e-8)


def test_continuous_infinite_limits_and_prismatic_units(tmp_path):
    path=tmp_path/'robot.urdf'
    path.write_text('<robot name="x"><joint name="wheel" type="continuous"><child link="w"/><limit lower="-inf" upper="inf"/></joint><joint name="slide" type="prismatic"><child link="s"/><limit lower="0" upper="0.2"/></joint></robot>')
    joints=load_joints(path)
    assert joints['wheel'].lower is None
    assert joints['wheel'].check(100)==100
    with pytest.raises(ValueError):joints['slide'].check(.3)


def test_bridge_creates_joint_reader_not_writer():
    from autosim.bridge.bridge import Bridge
    class Node:
        def __init__(self):self.readers={};self.writers={}
        def create_reader(self,channel,callback,dtype):self.readers[channel]=callback
        def create_writer(self,channel,dtype,qos_depth):self.writers[channel]=object();return self.writers[channel]
    node=Node()
    bridge=Bridge(node,{'cmd_vel':'/cmd_vel','joint_command':'/joint_commands','joint_states':'/joint_states'},
                 {'cmd_vel':object,'joint_command':object,'joint_states':object})
    msg=SimpleNamespace(name=['arm'],position=[.3])
    node.readers['/joint_commands'](msg)
    assert bridge.joint_command_reader.get_msg() is msg
    assert '/joint_commands' not in node.writers


def test_runner_joint_command_consumed_on_cycle_thread():
    from autosim.runtime.runner import Runner
    class Reader:
        def has_msg(self):return True
        def get_msg(self):return SimpleNamespace(name=['arm'],position=[.3])
    calls=[]
    runner=object.__new__(Runner)
    runner.bridge=SimpleNamespace(joint_command_reader=Reader())
    runner.simulator=SimpleNamespace(
        set_joint_positions=calls.append, joint_states=lambda: {'arm':0.0}
    )
    runner.robot_cfg={'joint_command':{'enabled':True,'timeout_sec':1.0,'mode':'hold'}}
    runner.joint_targets={}
    runner.joint_command_stamp=None
    runner.clock=SimpleNamespace(now=lambda: 0.0)
    runner.poll_joint_command()
    assert calls==[{'arm':.3}]


def test_joint_command_times_out_to_home_pose():
    """A publisher that stops must not leave the arm frozen mid-motion."""
    from autosim.runtime.runner import Runner

    class Reader:
        def __init__(self):
            self.pending = True

        def has_msg(self):
            return self.pending

        def get_msg(self):
            self.pending = False
            return SimpleNamespace(name=['arm'], position=[1.0])

    calls = []
    state = {'arm': 0.0}
    now = [0.0]

    def set_positions(updates):
        calls.append(dict(updates))
        state.update(updates)

    runner = object.__new__(Runner)
    runner.bridge = SimpleNamespace(joint_command_reader=Reader())
    runner.simulator = SimpleNamespace(
        set_joint_positions=set_positions, joint_states=lambda: dict(state)
    )
    runner.robot_cfg = {
        'joint_positions': {'arm': 0.2},
        'joint_command': {'enabled': True, 'timeout_sec': 0.5, 'mode': 'home'},
    }
    runner.joint_targets = {}
    runner.joint_command_stamp = None
    runner.clock = SimpleNamespace(now=lambda: now[0])
    runner.poll_joint_command()
    assert calls == [{'arm': 1.0}]
    now[0] = 1.0
    runner.poll_joint_command()
    assert calls[-1] == {'arm': 0.2}
    # The release fires once; a second idle cycle must not resend it.
    runner.poll_joint_command()
    assert len(calls) == 2


def test_joint_command_ramps_with_max_step():
    """``max_step`` keeps a large target from teleporting the model."""
    from autosim.runtime.runner import Runner

    class Reader:
        def __init__(self):
            self.pending = True

        def has_msg(self):
            return self.pending

        def get_msg(self):
            self.pending = False
            return SimpleNamespace(name=['arm'], position=[1.0])

    calls = []
    runner = object.__new__(Runner)
    runner.bridge = SimpleNamespace(joint_command_reader=Reader())
    runner.simulator = SimpleNamespace(
        set_joint_positions=calls.append, joint_states=lambda: {'arm': 0.0}
    )
    runner.robot_cfg = {
        'joint_command': {'enabled': True, 'timeout_sec': 0.0, 'max_step': 0.1}
    }
    runner.joint_targets = {}
    runner.joint_command_stamp = None
    runner.clock = SimpleNamespace(now=lambda: 0.0)
    runner.poll_joint_command()
    assert calls[-1] == {'arm': 0.1}


def test_initial_pose_and_command_channel_validation():
    from autosim.robot.viewer import viewer_settings
    from autosim.runtime.settings import Config
    data=viewer_settings('google_robot')
    data['habitat']['robot']['joint_command']={'enabled':True,'channel':'/joint_commands'}
    data['habitat']['sensors']['joint_states']={'enabled':True,'channel':'/joint_states'}
    cfg=Config.load(data)
    assert cfg.channel_map()['joint_command']=='/joint_commands'
    data['habitat']['robot']['joint_command']['channel']='/joint_states'
    with pytest.raises(ValueError,match='duplicate'):Config.load(data)
    data['habitat']['robot']['joint_command']['channel']='/joint_commands'
    data['habitat']['robot']['joint_positions']={'joint_elbow':100}
    with pytest.raises(ValueError,match='above'):Config.load(data)


def test_robot_root_and_moving_joint_tf_are_connected(monkeypatch):
    from autosim.runtime import Config, Runner
    from autosim.robot.viewer import viewer_settings
    from autosim.robot.urdf import UrdfModel
    from tests.fake_simulator import FakeSimulator
    from tests.test_runner_smoke import FakeLink
    monkeypatch.setattr('autosim.runtime.runner.time.sleep',lambda _:None)
    data=viewer_settings('google_robot')
    data['habitat']['sensors']['chase']['enabled']=False
    data['habitat']['robot']['tf']={'enabled':True,'publish_map_odom':True}
    model=UrdfModel.load('robot/google_robot/google_robot.urdf')
    joints=load_joints(model.path)
    positions=complete_positions(joints,{})
    positions['joint_shoulder']=.5
    simulator=FakeSimulator()
    simulator.urdf=model
    simulator.joint_states=lambda:positions
    runner=Runner(Config.load(data),max_steps=1,link=FakeLink(),simulator=simulator)
    runner.run()
    dynamic=[t for m in runner.node.writers['/tf'].msgs for t in m.transforms]
    static=[t for m in runner.node.writers['/tf_static'].msgs for t in m.transforms]
    assert any(t.child_frame_id==model.root and t.header.frame_id=='odom' for t in dynamic)
    moving_children={j.child for j in joints.values()}
    assert moving_children <= {t.child_frame_id for t in dynamic}
    assert not moving_children & {t.child_frame_id for t in static}
    child=joints['joint_shoulder'].child
    t=next(t for t in dynamic if t.child_frame_id==child)
    xyz,q=joint_transform(model.parents[child][1],model.joint_rpy[child],model.joint_axis[child],'revolute',.5)
    rotation=t.transform.rotation
    np.testing.assert_allclose([rotation.x,rotation.y,rotation.z,rotation.w],q,atol=1e-6)
