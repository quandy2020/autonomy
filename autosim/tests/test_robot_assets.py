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


def test_go2_display_keeps_all_upstream_triangles():
    """Open CAD surfaces must not lose panels during mesh decimation."""
    import struct
    folder=UrdfModel.resolve(PROFILES['unitree_go2'].urdf).parent
    provenance=json.loads((folder/'source.json').read_text())
    mesh_records={name:record for name,record in provenance['files'].items() if name.endswith('.glb')}
    assert len(mesh_records)==7
    for name,record in mesh_records.items():
        assert record['display_faces']==record['original_faces']
        data=(folder/name).read_bytes()
        assert data[:4]==b'glTF'
        size=struct.unpack_from('<I',data,12)[0]
        gltf=json.loads(data[20:20+size])
        triangles=0
        for mesh in gltf['meshes']:
            for primitive in mesh['primitives']:
                assert primitive.get('mode',4)==4
                triangles+=gltf['accessors'][primitive['indices']]['count']//3
        assert triangles==record['original_faces']

def test_google_visuals_use_unlit_shells_and_lit_decals():
    """Habitat 0.3.3 skips sRGB decoding; only the 040 decals stay lit."""
    import struct
    folder=UrdfModel.resolve(PROFILES['google_robot'].urdf).parent/'meshes/habitat'
    files=sorted(folder.glob('*.glb'))
    assert len(files)>=16
    for path in files:
        data=path.read_bytes()
        assert data[:4]==b'glTF'
        size=struct.unpack_from('<I',data,12)[0]
        gltf=json.loads(data[20:20+size])
        assert 'KHR_materials_unlit' in gltf.get('extensionsUsed',[]),path.name
        for material in gltf['materials']:
            unlit='KHR_materials_unlit' in material.get('extensions',{})
            if path.name=='base_trim.glb' and material.get('name') in {'number_left','number_right'}:
                assert not unlit,path.name
            else:
                assert unlit,path.name


def test_google_head_panel_is_modelled_and_attached():
    """The five-lens sensor face must ship and stay on the head link."""
    import struct
    folder=UrdfModel.resolve(PROFILES['google_robot'].urdf).parent
    root=ET.parse(folder/'google_robot.urdf').getroot()
    link=next(item for item in root.findall('link') if item.attrib['name']=='link_head_tilt')
    meshes=[mesh.attrib['filename'] for mesh in link.findall('visual/geometry/mesh')]
    assert 'meshes/habitat/head_sensor_panel.glb' in meshes
    data=(folder/'meshes/habitat/head_sensor_panel.glb').read_bytes()
    size=struct.unpack_from('<I',data,12)[0]
    gltf=json.loads(data[20:20+size])
    triangles=sum(gltf['accessors'][primitive['indices']]['count']//3
                  for mesh in gltf['meshes'] for primitive in mesh['primitives'])
    assert triangles>=1500
