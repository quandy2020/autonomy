#!/usr/bin/env python3
"""Load real meshes, move every independent DOF, render before/after images."""
import argparse
import json
from pathlib import Path
import sys
import subprocess
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from autosim.robot.profiles import PROFILES, shipped_robot_defaults
from autosim.robot.viewer import viewer_settings, render, frame_robot
from autosim.backends.habitat.simulator import Simulator


def main():
    import cv2
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profiles',nargs='+',default=[
        n for n,p in PROFILES.items()
        if p.articulated or shipped_robot_defaults(n).get('instance')
    ])
    parser.add_argument('--output',type=Path,default=Path('/tmp/autosim-robot-smoke'))
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    if len(args.profiles) > 1:
        results=[]
        for name in args.profiles:
            output=args.output/name
            output.mkdir(exist_ok=True)
            with (output/'habitat.log').open('w') as log:
                process=subprocess.run([sys.executable,'-X','faulthandler',__file__,
                    '--profiles',name,'--output',str(output)],stdout=log,stderr=subprocess.STDOUT)
            result_file=output/'results.json'
            if process.returncode == 0 and result_file.is_file():
                result=json.loads(result_file.read_text())[0]
            else:
                result={'profile':name,'ok':False,'exit_code':process.returncode,
                        'log':str(output/'habitat.log')}
            results.append(result)
            print(json.dumps(result),flush=True)
            (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
        if any(not r['ok'] for r in results):raise SystemExit(1)
        return
    results=[]
    for name in args.profiles:
        sim=None
        try:
            sim=Simulator.create(viewer_settings(name,width=640,height=480))
            if sim.articulation is None:raise RuntimeError('URDF not instanced')
            camera=frame_robot(sim)
            before=render(sim,camera)
            if float(before.std())<1:raise RuntimeError('blank render')
            home=sim.joint_states()
            updates={}
            for joint_name,joint in sim.articulation.joints.items():
                if joint.mimic:continue
                value=home[joint_name]+(.015 if joint.kind=='prismatic' else .12)
                if joint.upper is not None and value>joint.upper:value=home[joint_name]-(.015 if joint.kind=='prismatic' else .12)
                if joint.lower is not None:value=max(joint.lower,value)
                if abs(value-home[joint_name])<1e-8:continue
                sim.set_joint_positions({joint_name:value})
                if abs(sim.joint_states()[joint_name]-value)>1e-5:raise RuntimeError(f'joint did not move: {joint_name}')
                updates[joint_name]=value
            after=render(sim,camera)
            difference=float(np.abs(after.astype(float)-before.astype(float)).mean())
            if updates and difference<=.001:raise RuntimeError('joint changes did not affect rendered mesh')
            cv2.imwrite(str(args.output/f'{name}_before.png'),before[...,::-1])
            cv2.imwrite(str(args.output/f'{name}_after.png'),after[...,::-1])
            result={'profile':name,'ok':True,'joints':len(home),'controlled':len(updates),'image_delta':difference}
        except Exception as exc:
            result={'profile':name,'ok':False,'error':repr(exc)}
        finally:
            if sim is not None:sim.close()
        results.append(result)
        print(json.dumps(result),flush=True)
        (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    if any(not result['ok'] for result in results):raise SystemExit(1)

if __name__=='__main__':main()
