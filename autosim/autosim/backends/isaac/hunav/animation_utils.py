# Copyright 2026 The Openbot Authors (duyongquan)
#
# Port of Hunav_isaac_wrapper.animation_utils (AnimationGraph / retarget helpers).
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

"""SkelRoot / Skeleton / AnimationGraph helpers (Hunav_isaac_wrapper)."""

from __future__ import annotations

import logging
from typing import Any, Mapping

logger = logging.getLogger(__name__)


def find_skeleton_path(agent_prim: Any) -> Any:
    """Recursively search for a prim of type ``Skeleton`` within ``agent_prim``."""
    from pxr import Sdf

    if agent_prim is None or not agent_prim.IsValid():
        return Sdf.Path()
    if agent_prim.GetTypeName() == "Skeleton":
        return agent_prim.GetPath()
    for child in agent_prim.GetChildren():
        if not child.IsValid():
            continue
        child_path_str = child.GetPath().pathString
        if ("Looks" in child_path_str) or ("CharacterAnimation" in child_path_str):
            continue
        if child.GetTypeName() == "Skeleton":
            return child.GetPath()
        result = find_skeleton_path(child)
        if result:
            return result
    logger.warning(
        "No Skeleton found for %s, using /Root by default",
        agent_prim.GetPath(),
    )
    return Sdf.Path(f"{agent_prim.GetPath()}/Root")


def find_skelroot_path(agent_prim: Any) -> Any:
    """Recursively search for a ``SkelRoot`` within the children of ``agent_prim``."""
    from pxr import Sdf

    if agent_prim is None or not agent_prim.IsValid():
        return Sdf.Path()
    for child in agent_prim.GetChildren():
        if not child.IsValid():
            continue
        child_path_str = child.GetPath().pathString
        if ("Looks" in child_path_str) or ("CharacterAnimation" in child_path_str):
            continue
        if "ManRoot" in child_path_str:
            for grandchild in child.GetChildren():
                if grandchild.IsValid() and grandchild.GetTypeName() == "SkelRoot":
                    return grandchild.GetPath()
        if child.GetTypeName() == "SkelRoot":
            return child.GetPath()
        result = find_skelroot_path(child)
        if result:
            return result
    logger.warning(
        "No SkelRoot found within %s, using fallback /Root",
        agent_prim.GetPath(),
    )
    return Sdf.Path(f"{agent_prim.GetPath()}/Root")


def create_animation(stage: Any, animation_path: Any, source_path: str) -> Any:
    """Create a ``SkelAnimation`` prim referencing ``source_path``."""
    animation = stage.DefinePrim(animation_path, "SkelAnimation")
    animation.GetReferences().AddReference(source_path)
    return animation


def create_agent_animation_graph(
    stage: Any,
    agent_prim: Any,
    idle_anim_path: Any,
    walk_anim_path: Any,
) -> Any:
    """Create an AnimationGraph that blends idle / walk (wrapper layout)."""
    from pxr import Sdf

    graph_path = Sdf.Path(f"{str(agent_prim.GetPath())}/AnimationGraph")
    anim_graph_prim = stage.DefinePrim(graph_path, "AnimationGraph")

    blend_path = graph_path.AppendChild("Blend")
    blend_prim = stage.DefinePrim(blend_path, "Blend")
    idle_clip_path = graph_path.AppendChild("IdleLoop")
    idle_clip_prim = stage.DefinePrim(idle_clip_path, "AnimationClip")
    idle_clip_prim.CreateRelationship("inputs:animationSource").SetTargets(
        [idle_anim_path]
    )
    walk_clip_path = graph_path.AppendChild("WalkLoop")
    walk_clip_prim = stage.DefinePrim(walk_clip_path, "AnimationClip")
    walk_clip_prim.CreateRelationship("inputs:animationSource").SetTargets(
        [walk_anim_path]
    )
    speed_node_path = graph_path.AppendChild("speed")
    speed_node_prim = stage.DefinePrim(speed_node_path, "ReadVariable")
    speed_node_prim.CreateAttribute(
        "inputs:variableName", Sdf.ValueTypeNames.Token
    ).Set("speed")

    blend_prim.CreateRelationship("inputs:blendWeight").SetTargets([speed_node_path])
    blend_prim.CreateRelationship("inputs:pose0").SetTargets([idle_clip_path])
    blend_prim.CreateRelationship("inputs:pose1").SetTargets([walk_clip_path])
    anim_graph_prim.CreateRelationship("inputs:pose").SetTargets([blend_path])

    skel_path = find_skeleton_path(agent_prim)
    anim_graph_prim.CreateRelationship("skel:skeleton").SetTargets([skel_path])
    logger.info("Created AnimationGraph for %s at %s", agent_prim.GetPath(), graph_path)
    return graph_path


def apply_animation_graph(agent_prim: Any, graph_path: Any) -> bool:
    """Apply an AnimationGraph via Kit ``ApplyAnimationGraphAPICommand``."""
    from pxr import Sdf

    if agent_prim is None or not agent_prim.IsValid():
        return False
    try:
        import omni.kit.commands

        result = omni.kit.commands.execute(
            "ApplyAnimationGraphAPICommand",
            paths=[Sdf.Path(agent_prim.GetPath())],
            animation_graph_path=graph_path,
        )
        if isinstance(result, tuple) and result and result[0] is False:
            logger.warning("ApplyAnimationGraphAPICommand returned failure")
            return False
        logger.info(
            "Applied AnimationGraph (%s) to %s", graph_path, agent_prim.GetPath()
        )
        return True
    except Exception as exc:
        logger.warning("apply_animation_graph failed: %s", exc)
        return False


def setup_anim_retargeting(
    stage: Any,
    agent_prim: Any,
    source_animation_dict: Mapping[int, Any],
    target_animation_parent_path: str,
    *,
    source_biped_path: str = "/World/Biped_Setup/biped_demo_meters",
) -> None:
    """Retarget source biped idle/walk clips onto ``agent_prim`` skeleton."""
    try:
        import omni.kit.commands
    except Exception as exc:
        logger.warning("setup_anim_retargeting: omni unavailable (%s)", exc)
        return

    candidates = [
        source_biped_path,
        "/Root/Biped_Setup/biped_demo_meters",
        "/World/Biped_Setup",
        "/Root/Biped_Setup",
    ]
    source_agent_prim = None
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            source_agent_prim = prim
            break
    if source_agent_prim is None:
        logger.warning("Default biped prim not found (tried %s)", candidates)
        return

    source_skel_path = find_skeleton_path(source_agent_prim)
    target_skel_path = find_skeleton_path(agent_prim)
    if not source_skel_path or not target_skel_path:
        logger.warning("Retarget skipped: missing skeleton")
        return

    for key in (1, 0):
        anim = source_animation_dict.get(key)
        if anim is None:
            continue
        try:
            omni.kit.commands.execute(
                "CreateRetargetAnimationsCommand",
                source_skeleton_path=str(source_skel_path),
                target_skeleton_path=str(target_skel_path),
                source_animation_paths=[str(anim)],
                target_animation_parent_path=target_animation_parent_path,
                set_root_identity=False,
            )
        except Exception as exc:
            logger.warning("CreateRetargetAnimationsCommand failed: %s", exc)


def set_anim_graph_speed(
    stage: Any,
    anim_graph_character: Any,
    graph_path: Any,
    speed_value: float,
) -> None:
    """Set AnimationGraph ``speed`` variable (0 idle → 1 walk)."""
    from pxr import Sdf

    graph_prim = stage.GetPrimAtPath(graph_path)
    if not graph_prim or not graph_prim.IsValid():
        logger.debug("set_anim_graph_speed: missing graph at %s", graph_path)
        return
    speed_attr = graph_prim.GetAttribute("anim:graph:variable:speed")
    if not speed_attr or not speed_attr.IsValid():
        speed_attr = graph_prim.CreateAttribute(
            "anim:graph:variable:speed", Sdf.ValueTypeNames.Float, custom=True
        )
    speed_attr.Set(float(speed_value))
    if anim_graph_character is not None and hasattr(anim_graph_character, "set_variable"):
        try:
            anim_graph_character.set_variable("speed", float(speed_value))
        except Exception as exc:
            logger.debug("set_variable(speed) failed: %s", exc)
