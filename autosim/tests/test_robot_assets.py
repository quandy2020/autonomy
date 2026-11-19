"""Validate shipped robot assets, reproducible provenance and all example configs."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from autosim.robot.profiles import PROFILES
from autosim.robot.urdf import UrdfModel
from autosim.robot.joints import load_joints,complete_positions
from autosim.runtime.settings import Config

ARTICULATED=[name for name,p in PROFILES.items() if p.articulated]


@pytest.mark.parametrize('name',ARTICULATED)
def test_shipped_articulated_assets(name):
    path=UrdfModel.resolve(PROFILES[name].urdf)
    root=ET.parse(path).getroot()
    joints=load_joints(path)
    assert joints
    complete_positions(joints,{})
    provenance=json.loads((path.parent/'source.json').read_text())
    for mesh in root.findall('.//mesh'):
        relative=mesh.attrib['filename']
        assert not relative.startswith(('package://','/'))
        assert (path.parent/relative).is_file()
        assert relative in provenance['files']
    for relative,record in provenance['files'].items():
        file=path.parent/relative
        assert hashlib.sha256(file.read_bytes()).hexdigest()==record['sha256']
    if 'normalized_urdf_sha256' in provenance:
        assert hashlib.sha256(path.read_bytes()).hexdigest()==provenance['normalized_urdf_sha256']
    config=Config.load(UrdfModel.package_root()/'config/robots'/f'{name}.yaml')
    assert config.channel_map()['joint_command']=='/joint_commands'
    assert config.channel_map()['joint_states']=='/joint_states'
    model=UrdfModel.load(PROFILES[name].urdf)
    assert model.odom_child_frame()==model.root
