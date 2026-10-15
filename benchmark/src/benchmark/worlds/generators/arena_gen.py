# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena-Gen lite: keyword / prompt → procedural world (no LLM required).

Full Arena-Gen uses LLM + 3DSG; this maps common social-nav prompts to the
existing dynamic layout generators so challenge YAMLs stay reproducible.
"""

from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple

from benchmark.simulators.world_state import ObstacleDisk
from benchmark.worlds.generators.dynamic_map import generate_dynamic_layout, generate_world

_KEYWORD_MAP: List[Tuple[Tuple[str, ...], str, Dict[str, Any]]] = [
    (("hospital", "clinic", "ward", "医疗", "医院"), "hospital", {"pedestrians": 8, "difficulty": 0.6}),
    (("canteen", "cafeteria", "restaurant", "食堂", "餐厅"), "canteen", {"pedestrians": 10, "difficulty": 0.5}),
    (("warehouse", "industrial", "factory", "仓库", "厂房"), "industrial", {"pedestrians": 4, "difficulty": 0.7}),
    (("office", "workplace", "办公"), "office", {"pedestrians": 6, "difficulty": 0.45}),
    (("maze", "labyrinth", "迷宫"), "maze", {"pedestrians": 2, "difficulty": 0.8}),
    (("barn", "outdoor", "outdoor", "户外"), "barn", {"pedestrians": 3, "difficulty": 0.4}),
    (("rescue", "emergency", "evacuat", "救援", "疏散"), "warehouse", {"pedestrians": 12, "difficulty": 0.9}),
    (("corridor", "hallway", "走廊"), "posts", {"pedestrians": 2, "difficulty": 0.3}),
]


def parse_prompt(prompt: str) -> Dict[str, Any]:
    """Parse a free-text prompt into world kind + spawn hints."""
    text = (prompt or "").strip().lower()
    kind = "posts"
    hints: Dict[str, Any] = {"pedestrians": 4, "difficulty": 0.4}
    for keys, mapped, extra in _KEYWORD_MAP:
        if any(k.lower() in text for k in keys):
            kind = mapped
            hints = dict(extra)
            break
    # Difficulty adjectives
    if any(w in text for w in ("hard", "difficult", "challenging", "困难", "难")):
        hints["difficulty"] = min(1.0, float(hints.get("difficulty", 0.5)) + 0.25)
        hints["pedestrians"] = int(hints.get("pedestrians", 4)) + 3
    if any(w in text for w in ("easy", "simple", "简单", "容易")):
        hints["difficulty"] = max(0.1, float(hints.get("difficulty", 0.5)) - 0.2)
    # Explicit counts: "12 pedestrians" / "8 obstacles"
    m = re.search(r"(\d+)\s*ped", text)
    if m:
        hints["pedestrians"] = int(m.group(1))
    m = re.search(r"(\d+)\s*obs", text)
    if m:
        hints["obstacles"] = int(m.group(1))
    # Level N
    m = re.search(r"level\s*(\d+)", text)
    if m:
        level = int(m.group(1))
        hints["difficulty"] = min(1.0, 0.2 * level)
        hints["level"] = level
    return {"kind": kind, "prompt": prompt, **hints}


def generate_from_prompt(
    prompt: str,
    *,
    seed: int = 0,
    count: Optional[int] = None,
) -> Dict[str, Any]:
    """Return world YAML dict + layout metadata from a text prompt."""
    spec = parse_prompt(prompt)
    kind = str(spec["kind"])
    n = int(count if count is not None else 12 + 10 * float(spec.get("difficulty", 0.4)))
    disks = generate_dynamic_layout(seed=seed, kind=kind, count=n)
    world = generate_world(kind if kind != "posts" else "posts", seed=seed)
    world.update(
        {
            "arena_gen": True,
            "prompt": prompt,
            "kind": kind,
            "pedestrians": int(spec.get("pedestrians", 4)),
            "difficulty": float(spec.get("difficulty", 0.4)),
            "layout_disks": len(disks),
            "seed": seed,
        }
    )
    return world


def disks_from_prompt(prompt: str, *, seed: int = 0, count: Optional[int] = None) -> List[ObstacleDisk]:
    spec = parse_prompt(prompt)
    n = int(count if count is not None else 16)
    return generate_dynamic_layout(seed=seed, kind=str(spec["kind"]), count=n)
