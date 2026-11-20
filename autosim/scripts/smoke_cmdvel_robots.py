#!/usr/bin/env python3
"""Real Autolink cmd_vel transport + Habitat AO + odometry/TF validation."""
from pathlib import Path
import argparse
import json
import math
import time
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
# Use the same protobuf compatibility bootstrap as the project's tests.
import tests.conftest
import autolink
from autosim.runtime import Config,Runner
from autosim.robot.urdf import UrdfModel
from autosim.bridge.messages import Messages
from autosim.robot.viewer import frame_robot,render
from autosim.runtime.runner import cmd_vel_message_type
from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.diagnostic_msgs.diagnostic_array_pb2 import DiagnosticArray
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.tf2_msgs.tf_message_pb2 import TFMessage
import cv2
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--profile',required=True,choices=['google_robot','unitree_g1','unitree_go2','turtlebot3_burger','husky'])
p.add_argument('--cmd-vel-type',choices=['twist','twist_stamped'],default=None,
               help='override robot.cmd_vel_type for transport verification')
p.add_argument('--output',type=Path,default=Path('/tmp/autosim-cmdvel'))
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
data=Config.load(UrdfModel.package_root()/'config/robots'/f'{a.profile}.yaml').data
if a.cmd_vel_type:
    data['habitat']['robot']['cmd_vel_type']=a.cmd_vel_type
prefix=f'/autosim/verification/{a.profile}'
def isolate(block):
    for key,value in list(block.items()):
        if isinstance(value,dict):isolate(value)
        elif isinstance(value,str) and value.startswith('/') and key.endswith(('channel','cmd_vel')):block[key]=prefix+value
isolate(data)
runner=None
try:
    runner=Runner(Config.load(data))
    runner.sensor_worker.start()
    observer=autolink.Node('autosim_cmdvel_verification')
    received={'odom':None,'tf':None,'status':None}
    readers=[observer.create_reader(prefix+'/odom',lambda m:received.update(odom=m),Odometry),
             observer.create_reader(prefix+'/tf',lambda m:received.update(tf=m),TFMessage),
             observer.create_reader(prefix+'/autosim/status',lambda m:received.update(status=m),DiagnosticArray)]
    cmd_type=cmd_vel_message_type(runner.robot_cfg)
    writer=observer.create_writer(prefix+'/cmd_vel',cmd_type,qos_depth=10)
    def command(v,w):
        if cmd_type is Twist:return Messages.encode_twist(v,w)
        return Messages.encode_twist_stamped(v,w,runner.clock.stamp())
    time.sleep(.4)
    sim=runner.simulator
    # Keep sensors cheap while retaining a real render camera for before/after.
    runner.chase=None
    camera=frame_robot(sim)
    before=render(sim,camera)
    start=sim.pose()
    trajectories=[]
    wheels_before=dict(sim.joint_states())
    wheel_phases={}
    wheel_sides=dict(sim.wheel_mounts())
    for label,v,w in [('forward',.3,0),('backward',-.3,0),('turn',0,.6),('arc',.2,.4)]:
        origin=sim.pose()
        phase_before=dict(sim.joint_states())
        for _ in range(60):
            writer.write(command(v,w))
            time.sleep(.01)
            runner.cycle(.02)
        end=sim.pose();distance=math.hypot(end[0]-origin[0],end[1]-origin[1])
        if label=='turn':assert abs(end[2]-origin[2])>.4
        else:assert distance>.12
        assert np.allclose(sim.pose(),runner.robot.pose())
        # URDF root translation must agree with map -> Habitat convention.
        assert np.allclose(tuple(sim.articulated.translation),[end[0],sim.floor_y+sim.root_height(),-end[1]])
        trajectories.append(dict(command=label,pose=end))
        phase_after=dict(sim.joint_states())
        delta={name:phase_after[name]-phase_before[name] for name in phase_after
               if 'wheel' in name.lower()}
        wheel_phases[label]=delta
        if delta:
            left={name:value for name,value in delta.items() if wheel_sides.get(name,0)>0}
            right={name:value for name,value in delta.items() if wheel_sides.get(name,0)<0}
            if label in ('forward','arc'):
                assert all(value>1e-4 for value in delta.values()), f'{label}: wheel did not roll forward: {delta}'
            elif label=='backward':
                assert all(value<-1e-4 for value in delta.values()), f'{label}: wheel did not roll backward: {delta}'
            elif label=='turn':
                assert left and right, f'turn: wheel sides not identified: {wheel_sides}'
                assert all(value<-1e-4 for value in left.values()), f'turn: left wheel did not reverse: {left}'
                assert all(value>1e-4 for value in right.values()), f'turn: right wheel did not advance: {right}'
            if label=='arc':
                assert left and right and min(right.values())>max(left.values()), \
                    f'arc: right side should outrun left: {delta}'
    wheels_after=dict(sim.joint_states())
    spun={name:round(wheels_after[name]-wheels_before[name],3)
          for name in wheels_after if 'wheel' in name.lower()}
    after=render(sim,camera)
    for _ in range(80):runner.cycle(.02);time.sleep(.005)
    assert np.allclose(runner.robot.velocity(),[0,0])
    time.sleep(.1)
    assert received['odom'] is not None and received['tf'] is not None
    assert received['status'] is not None, 'no /autosim/status message'
    streams=[s.name for s in received['status'].status]
    assert 'autosim/control' in streams
    pos=received['odom'].pose.pose.pose.position
    assert np.allclose([pos.x,pos.y],sim.pose()[:2],atol=1e-5)
    root=next(t for t in received['tf'].transforms if t.header.frame_id=='odom' and t.child_frame_id==sim.urdf.root)
    assert np.allclose([root.transform.translation.x,root.transform.translation.y],sim.pose()[:2],atol=1e-5)
    cv2.imwrite(str(a.output/(a.profile+'_before.png')),before[...,::-1]);cv2.imwrite(str(a.output/(a.profile+'_after.png')),after[...,::-1])
    result={'profile':a.profile,'ok':True,
            'transport':'Autolink '+('Twist' if cmd_type is Twist else 'TwistStamped'),
            'trajectories':trajectories,'watchdog_stopped':True,'odom_tf_match':True,
            'wheel_delta':spun,
            'wheel_phases':{label:{name:round(value,3) for name,value in delta.items()}
                            for label,delta in wheel_phases.items()},
            'status_streams':len(streams)}
    (a.output/(a.profile+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
finally:
    if runner is not None:runner.sensor_worker.stop();runner.simulator.close()
    autolink.shutdown()
