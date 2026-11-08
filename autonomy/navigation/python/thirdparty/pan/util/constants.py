"""Shared constants: repo paths, the platform and its sensor, discrete actions, DeFM token shapes."""
import os
from enum import IntEnum
from pathlib import Path


# ---------------------------------------------------------------- repo paths
REPO_ROOT = Path(__file__).resolve().parents[3]          # <repo>/thirdparty/pan/util/constants.py
DATASET_ROOT = Path(os.environ.get("VNT_PA_DATASET", REPO_ROOT / "dataset"))
HABITAT_DATA_ROOT = DATASET_ROOT / "habitat"
HM3D_SCENE_DATASET_CONFIG = (HABITAT_DATA_ROOT / "raw/scene_datasets/hm3d"
                             / "hm3d_annotated_basis.scene_dataset_config.json")


# ---------------------------------------------------------------- platform
# Wheeled Go2, body ~0.70 x 0.31 m.
AGENT_RADIUS = 0.20         # navmesh erosion; below the 0.383 m the body sweeps in a point turn
AGENT_HEIGHT = 0.70         # standing height, sensor mast included
SENSOR_HEIGHT = 0.60        # depth camera above the ground
NAVMESH_SUFFIX = ".go2.navmesh"   # baked for AGENT_RADIUS / AGENT_HEIGHT, next to each scene .glb


# ---------------------------------------------------------------- depth sensor
# 110 x 80.07 deg (square pixels).
SENSOR_RES = (160, 272)     # (H, W)
SENSOR_HFOV_DEG = 110.0


# ---------------------------------------------------------------- discrete embodiment
FORWARD_STEP_M = 0.25
TURN_STEP_DEG = 30.0


class Action(IntEnum):
    """Habitat PointNav discrete actions."""

    STOP = 0
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3


N_ACTIONS = len(Action)


# ---------------------------------------------------------------- DeFM token shapes
# DeFM ViT-S/14 (leggedrobotics/defm) over metric depth: the last block's normed patch tokens.
DEFM_IMAGE_RESOLUTION = 224          # the ViT's pretrain square; frames are resized to it
DEFM_PATCH_SIZE = 14
DEFM_TOKEN_DIM = 384                 # ViT-S embed_dim
DEFM_GRID = DEFM_IMAGE_RESOLUTION // DEFM_PATCH_SIZE       # 16
