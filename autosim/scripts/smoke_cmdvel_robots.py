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
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.tf2_msgs.tf_message_pb2 import TFMessage
import cv2
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--profile',required=True,choices=['google_robot','unitree_g1','unitree_go2'])
p.add_argument('--output',type=Path,default=Path('/tmp/autosim-cmdvel'))
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
data=Config.load(UrdfModel.package_root()/'config/robots'/f'{a.profile}.yaml').data
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
    received={'odom':None,'tf':None}
    readers=[observer.create_reader(prefix+'/odom',lambda m:received.update(odom=m),Odometry),
             observer.create_reader(prefix+'/tf',lambda m:received.update(tf=m),TFMessage)]
    writer=observer.create_writer(prefix+'/cmd_vel',TwistStamped,qos_depth=10)
    time.sleep(.4)
    sim=runner.simulator
    # Keep sensors cheap while retaining a real render camera for before/after.
    runner.chase=None
    camera=frame_robot(sim)
    before=render(sim,camera)
    start=sim.pose()
    trajectories=[]
    for label,v,w in [('forward',.3,0),('backward',-.3,0),('turn',0,.6),('arc',.2,.4)]:
        origin=sim.pose()
        for _ in range(60):
            writer.write(Messages.encode_twist_stamped(v,w,runner.clock.stamp()))
            time.sleep(.01)
            runner.cycle(.02)
        end=sim.pose();distance=math.hypot(end[0]-origin[0],end[1]-origin[1])
        if label=='turn':assert abs(end[2]-origin[2])>.4
        else:assert distance>.12
        assert np.allclose(sim.pose(),runner.robot.pose())
        # URDF root translation must agree with map -> Habitat convention.
        assert np.allclose(tuple(sim.articulated.translation),[end[0],sim.floor_y+runner.robot_cfg['root_height'],-end[1]])
        trajectories.append(dict(command=label,pose=end))
    after=render(sim,camera)
    for _ in range(80):runner.cycle(.02);time.sleep(.005)
    assert np.allclose(runner.robot.velocity(),[0,0])
    time.sleep(.1)
    assert received['odom'] is not None and received['tf'] is not None
    pos=received['odom'].pose.pose.pose.position
    assert np.allclose([pos.x,pos.y],sim.pose()[:2],atol=1e-5)
    root=next(t for t in received['tf'].transforms if t.header.frame_id=='odom' and t.child_frame_id==sim.urdf.root)
    assert np.allclose([root.transform.translation.x,root.transform.translation.y],sim.pose()[:2],atol=1e-5)
    cv2.imwrite(str(a.output/(a.profile+'_before.png')),before[...,::-1]);cv2.imwrite(str(a.output/(a.profile+'_after.png')),after[...,::-1])
    result={'profile':a.profile,'ok':True,'transport':'Autolink TwistStamped','trajectories':trajectories,'watchdog_stopped':True,'odom_tf_match':True}
    (a.output/(a.profile+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
finally:
    if runner is not None:runner.sensor_worker.stop();runner.simulator.close()
    autolink.shutdown()
