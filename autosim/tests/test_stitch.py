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

from pathlib import Path

import pytest

from autosim.config import Config
from autosim.stitch import PANORAMA_UUID, panorama_spec


def test_panorama_disabled_by_default():
    assert panorama_spec({"cameras": {}}) is None
    assert panorama_spec({"panorama": {"enabled": False}}) is None


def test_panorama_requires_xyz_triple():
    with pytest.raises(ValueError, match="xyz"):
        panorama_spec({"panorama": {"enabled": True, "xyz": [0.0, 1.0]}})


def test_bev_panorama_is_full_sphere():
    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "bev.yaml")
    pano = panorama_spec(settings["habitat"]["sensors"])
    assert pano is not None
    assert pano["channel"] == "/surround/panorama"
    assert pano["uuid"] == PANORAMA_UUID
    assert pano["width"] == 1920
    assert pano["height"] == 960
    assert pano["xyz"] == (0.0, 0.0, 1.2)
