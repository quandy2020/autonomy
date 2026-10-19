# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""BicMap scene catalog — SDK vendored under static/vendor/bic-map (offline)."""

from __future__ import annotations

from typing import Any

# Built from local source: third_party/bic-map-src (scripts/build_bicmap.sh).
# Served by Flask only: /static/vendor/bic-map/bicMap.ext.min.js — never fetches CDN.
BICMAP_LOCAL_JS = "vendor/bic-map/bicMap.ext.min.js"
BICMAP_VERSION = "0.0.8"

# Map benchmark missions → BicMap visualization intent (aligned with upstream examples).
BICMAP_SCENES: list[dict[str, Any]] = [
    {
        "id": "navigate",
        "mission": "navigate",
        "label": "单点导航",
        "bicmap_example": "indoor/singleNavigation",
        "blurb": "目标点导航 · 机器人 Marker + 路径折线",
        "layers": ["robot", "path", "goal"],
    },
    {
        "id": "dock",
        "mission": "dock",
        "label": "回充停靠",
        "bicmap_example": "scene/hotelDelivery",
        "blurb": "驶向充电桩 · 精停目标 POI",
        "layers": ["robot", "path", "goal", "dock"],
    },
    {
        "id": "explore_mapping",
        "mission": "explore_mapping",
        "label": "探索建图",
        "bicmap_example": "indoor/buildMap",
        "blurb": "覆盖探索 · 轨迹热区（对齐建图示例）",
        "layers": ["robot", "path", "coverage"],
    },
    {
        "id": "patrol",
        "mission": "patrol",
        "label": "巡检巡逻",
        "bicmap_example": "scene/securityPatrol",
        "blurb": "多航点巡检 · 对齐园区安防巡逻",
        "layers": ["robot", "path", "waypoints"],
    },
    {
        "id": "teleop",
        "mission": "teleop",
        "label": "遥操跟随",
        "bicmap_example": "indoor/robotFollow",
        "blurb": "开环指令 · 实时位置跟随视角",
        "layers": ["robot", "path", "follow"],
    },
    {
        "id": "location",
        "mission": "navigate",
        "label": "实时定位",
        "bicmap_example": "indoor/location",
        "blurb": "位姿流更新（SSE → Marker）",
        "layers": ["robot", "follow"],
    },
    {
        "id": "path_replay",
        "mission": "navigate",
        "label": "路径回放",
        "bicmap_example": "base/PathReplay",
        "blurb": "回放 episode 轨迹折线",
        "layers": ["robot", "path", "replay"],
    },
]


# Shared topic / channel catalogs for Image panel selects.
BICMAP_IMAGE_TOPICS: list[str] = [
    "/camera/front/rgb",
    "/camera/front/depth",
    "/camera/front/compressed",
    "/camera/rear/rgb",
    "/camera/rear/depth",
    "/camera/top/rgb",
    "/camera/left/rgb",
    "/camera/right/rgb",
    "/sensing/camera/traffic_light/image_raw",
    "/perception/image",
]

BICMAP_IMAGE_CHANNELS: list[dict[str, str]] = [
    {"id": "rgb", "label": "rgb"},
    {"id": "bgr8", "label": "bgr8"},
    {"id": "rgb8", "label": "rgb8"},
    {"id": "mono8", "label": "mono8"},
    {"id": "depth", "label": "depth"},
    {"id": "depth16", "label": "depth16"},
    {"id": "compressed", "label": "compressed"},
    {"id": "raw", "label": "raw"},
]

# Foxglove-style Image panel slots (map right rail).
BICMAP_IMAGES: list[dict[str, Any]] = [
    {
        "id": "slot_a",
        "label": "Camera A",
        "topic": "/camera/front/rgb",
        "channel": "rgb8",
        "kind": "rgb",
        "on": True,
        "topics": [
            "/camera/front/rgb",
            "/camera/front/compressed",
            "/camera/left/rgb",
            "/sensing/camera/traffic_light/image_raw",
        ],
    },
    {
        "id": "slot_b",
        "label": "Camera B",
        "topic": "/camera/front/depth",
        "channel": "depth",
        "kind": "depth",
        "on": True,
        "topics": [
            "/camera/front/depth",
            "/camera/rear/depth",
            "/camera/top/rgb",
        ],
    },
    {
        "id": "slot_c",
        "label": "Camera C",
        "topic": "/camera/rear/rgb",
        "channel": "rgb8",
        "kind": "rgb",
        "on": True,
        "topics": [
            "/camera/rear/rgb",
            "/camera/right/rgb",
            "/perception/image",
        ],
    },
    {
        "id": "slot_d",
        "label": "Camera D",
        "topic": "/camera/top/rgb",
        "channel": "rgb8",
        "kind": "rgb",
        "on": False,
        "topics": [
            "/camera/top/rgb",
            "/camera/front/rgb",
            "/camera/front/compressed",
        ],
    },
]


def bicmap_payload() -> dict[str, Any]:
    return {
        "version": BICMAP_VERSION,
        "local_js": BICMAP_LOCAL_JS,
        "offline": True,
        "origin_lnglat": [116.3912, 39.9073],
        "scenes": BICMAP_SCENES,
        "default_scene": "navigate",
        "images": BICMAP_IMAGES,
        "image_topics": BICMAP_IMAGE_TOPICS,
        "image_channels": BICMAP_IMAGE_CHANNELS,
    }
