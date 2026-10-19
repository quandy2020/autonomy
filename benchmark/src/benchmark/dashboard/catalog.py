# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Read experiment YAML / registry for the dashboard UI."""

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Any

import yaml

from benchmark.common.config import load_config, load_yaml, package_root, resolve_path
from benchmark.common.registry import get_registry
from benchmark.dashboard.bicmap_scenes import bicmap_payload
from benchmark.dashboard.rpc_client import describe_rpc_request, find_automsgs_root
from benchmark.simulators.humans import list_human_models
from benchmark.tasks.missions import MISSION_CATALOG, NAMED_RPC_CATALOG
from benchmark.tasks.modes import list_modes
from benchmark.tasks.planning.factory import list_planners

_PACKAGE_RE = re.compile(r"^\s*package\s+([\w.]+)\s*;", re.MULTILINE)
_SERVICE_RE = re.compile(
    r"^\s*service\s+(\w+)\s*\{([^}]*)\}", re.MULTILINE | re.DOTALL
)
_RPC_RE = re.compile(r"^\s*rpc\s+(\w+)\s*\(", re.MULTILINE)

# Service → UI group label
_RPC_GROUP_LABELS: dict[str, str] = {
    "NavigationService": "导航",
    "ChargeService": "充电",
    "FollowService": "跟随",
    "TeleopService": "遥操",
    "ExplorationService": "探索",
    "MapService": "建图",
    "LocalizationService": "定位",
    "SensorService": "传感器",
    "SystemService": "系统",
    "VoiceService": "语音",
}

# Recordable channels as automsgs message types (dashboard Data tab).
# `id` stays the collect/recorder flag; `msg` is the automsgs type name.
COLLECT_MSG_CHANNELS: list[dict[str, Any]] = [
    {
        "id": "pose",
        "msg": "geometry_msgs/Pose2D",
        "label": "Pose2D",
        "icon": "pose",
        "description": "机器人 2D 位姿",
        "default": True,
        "attrs": [
            {"id": "x", "label": "x", "type": "double"},
            {"id": "y", "label": "y", "type": "double"},
            {"id": "theta", "label": "theta", "type": "double"},
        ],
    },
    {
        "id": "velocity",
        "msg": "geometry_msgs/Twist",
        "label": "Twist",
        "icon": "twist",
        "description": "实测线/角速度",
        "default": True,
        "attrs": [
            {"id": "linear.x", "label": "linear.x", "type": "double"},
            {"id": "linear.y", "label": "linear.y", "type": "double"},
            {"id": "linear.z", "label": "linear.z", "type": "double"},
            {"id": "angular.x", "label": "angular.x", "type": "double"},
            {"id": "angular.y", "label": "angular.y", "type": "double"},
            {"id": "angular.z", "label": "angular.z", "type": "double"},
        ],
    },
    {
        "id": "action",
        "msg": "geometry_msgs/TwistStamped",
        "label": "TwistStamped",
        "icon": "cmd",
        "description": "cmd_vel 控制指令",
        "default": True,
        "attrs": [
            {"id": "header.stamp", "label": "header.stamp", "type": "time"},
            {"id": "header.frame_id", "label": "header.frame_id", "type": "string"},
            {"id": "twist.linear.x", "label": "twist.linear.x", "type": "double"},
            {"id": "twist.angular.z", "label": "twist.angular.z", "type": "double"},
        ],
    },
    {
        "id": "include_laser",
        "msg": "sensor_msgs/LaserScan",
        "label": "LaserScan",
        "icon": "scan",
        "description": "平面激光扫描",
        "default": True,
        "attrs": [
            {"id": "angle_min", "label": "angle_min", "type": "float"},
            {"id": "angle_max", "label": "angle_max", "type": "float"},
            {"id": "angle_increment", "label": "angle_increment", "type": "float"},
            {"id": "range_min", "label": "range_min", "type": "float"},
            {"id": "range_max", "label": "range_max", "type": "float"},
            {"id": "ranges", "label": "ranges", "type": "float[]"},
            {"id": "intensities", "label": "intensities", "type": "float[]"},
        ],
    },
    {
        "id": "include_image",
        "msg": "sensor_msgs/Image",
        "label": "Image",
        "icon": "image",
        "description": "相机图像（仿真器提供时）",
        "default": False,
        "attrs": [
            {"id": "height", "label": "height", "type": "uint32"},
            {"id": "width", "label": "width", "type": "uint32"},
            {"id": "encoding", "label": "encoding", "type": "string"},
            {"id": "step", "label": "step", "type": "uint32"},
            {"id": "data", "label": "data", "type": "bytes"},
        ],
    },
    {
        "id": "lerobot",
        "msg": "export/LeRobot",
        "label": "LeRobot",
        "icon": "export",
        "description": "同步导出 LeRobot 数据集",
        "default": False,
        "attrs": [
            {"id": "repo_id", "label": "repo_id", "type": "string"},
            {"id": "fps", "label": "fps", "type": "float"},
        ],
    },
]

# Backward-compatible aliases.
COLLECT_CHANNELS: list[dict[str, Any]] = list(COLLECT_MSG_CHANNELS)
COLLECT_CHANNEL_TREE: list[dict[str, Any]] = COLLECT_MSG_CHANNELS


def _safe_load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data if isinstance(data, dict) else {}


def list_config_files() -> list[dict[str, str]]:
    root = package_root() / "configs"
    items: list[dict[str, str]] = []
    if not root.is_dir():
        return items
    for path in sorted(root.rglob("*.yaml")):
        rel = path.relative_to(package_root()).as_posix()
        items.append(
            {
                "path": rel,
                "name": path.stem,
                "group": path.parent.name if path.parent != root else "root",
            }
        )
    return items


def list_task_configs() -> list[dict[str, Any]]:
    tasks_dir = package_root() / "configs" / "tasks"
    hub = get_registry()
    hub.ensure_builtins()
    registered = set(hub.tasks.names())
    out: list[dict[str, Any]] = []
    if not tasks_dir.is_dir():
        return out
    for path in sorted(tasks_dir.glob("*.yaml")):
        data = _safe_load(path)
        name = str(data.get("name") or path.stem)
        out.append(
            {
                "id": name,
                "file": path.relative_to(package_root()).as_posix(),
                "type": str(data.get("type") or name),
                "description": str(data.get("description") or ""),
                "mode": str(data.get("mode") or "scenario"),
                "episodes": int(data.get("episodes") or 1),
                "registered": name in registered or str(data.get("type") or "") in registered,
                "raw": data,
            }
        )
    return out


# Dashboard UI only exposes these backends (Isaac / Habitat).
DASHBOARD_SIMULATORS: tuple[str, ...] = ("isaac", "habitat")


def get_registry_snapshot() -> dict[str, list[str]]:
    hub = get_registry()
    hub.ensure_builtins()
    available = set(hub.simulators.names())
    sims = [name for name in DASHBOARD_SIMULATORS if name in available]
    if not sims:
        sims = list(DASHBOARD_SIMULATORS)
    return {
        "simulators": sims,
        "tasks": list(hub.tasks.names()),
        "planners": list_planners(),
        "controllers": [c["id"] for c in list_controllers()],
        "modes": list_modes(),
        "humans": list_human_models(),
    }


def load_experiment(
    config_path: str | None = None,
    *,
    overrides: list[str] | None = None,
) -> dict[str, Any]:
    path = config_path or str(package_root() / "configs" / "default.yaml")
    cfg = load_config(path, overrides=overrides or [])
    return {
        "path": str(cfg.path) if cfg.path else path,
        "experiment": cfg.experiment,
        "task_name": cfg.task_name,
        "simulator_name": cfg.simulator_name,
        "planner_name": cfg.planner_name,
        "sections": {
            "task": cfg.section("task"),
            "simulator": cfg.section("simulator"),
            "planner": cfg.section("planner"),
            "robot": cfg.section("robot"),
            "world": cfg.section("world"),
            "humans": cfg.section("humans") if "humans" in cfg.raw else {},
            "evaluation": cfg.section("evaluation") if "evaluation" in cfg.raw else {},
            "modules": dict(cfg.raw.get("modules") or {}),
        },
        "raw": cfg.raw,
    }


def read_config_file(rel_path: str) -> dict[str, Any]:
    path = resolve_path(rel_path)
    root = package_root().resolve()
    resolved = path.resolve()
    try:
        rel = resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"path escapes package root: {rel_path}") from exc
    text = resolved.read_text(encoding="utf-8") if resolved.is_file() else ""
    return {
        "path": rel.as_posix(),
        "text": text,
        "data": load_yaml(resolved) if resolved.is_file() else {},
    }


def collect_options_from_config(config_path: str | None = None) -> dict[str, Any]:
    exp = load_experiment(config_path)
    task = exp["sections"].get("task") or {}
    # Prefer dedicated collect.yaml defaults when current task is not collect.
    collect_yaml = package_root() / "configs" / "tasks" / "collect.yaml"
    defaults = _safe_load(collect_yaml)
    src = task if str(task.get("name") or task.get("type") or "") == "collect" else defaults
    channels: list[dict[str, Any]] = []
    for ch in COLLECT_MSG_CHANNELS:
        key = str(ch["id"])
        value = bool(src.get(key, ch["default"])) if key in src or not ch.get("locked") else True
        if ch.get("locked"):
            value = True
        attrs = [{**a, "enabled": value} for a in (ch.get("attrs") or [])]
        channels.append(
            {
                "id": key,
                "msg": str(ch.get("msg") or ch.get("label") or key),
                "label": str(ch.get("label") or key),
                "icon": str(ch.get("icon") or "msg"),
                "description": str(ch.get("description") or ""),
                "default": bool(ch.get("default", False)),
                "enabled": value,
                "attrs": attrs,
            }
        )
    return {
        "output": str(src.get("output") or "data/datasets/collect"),
        "fps": float(src.get("fps") or 10),
        "episode_frames": int(src.get("episode_frames") or 80),
        "episodes": int(src.get("episodes") or 3),
        "lerobot": bool(src.get("lerobot") or False),
        "channels": channels,
        # Flat list of automsgs channels (one row per message).
        "channel_tree": channels,
    }


def replay_options_from_config(config_path: str | None = None) -> dict[str, Any]:
    """Defaults for the dashboard Replay / Play panel."""
    replay_yaml = package_root() / "configs" / "tasks" / "replay.yaml"
    defaults = _safe_load(replay_yaml)
    exp = load_experiment(config_path)
    task = exp["sections"].get("task") or {}
    src = task if str(task.get("name") or task.get("type") or "") == "replay" else defaults
    return {
        "source": str(src.get("source") or defaults.get("source") or "data/datasets/collect"),
        "rate": float(src.get("rate") if src.get("rate") is not None else defaults.get("rate") or 1.0),
        "loop": bool(src.get("loop") if "loop" in src else defaults.get("loop") or False),
        "dry_run": bool(
            src.get("dry_run") if "dry_run" in src else defaults.get("dry_run") or False
        ),
    }


def list_missions() -> list[dict[str, Any]]:
    """编排任务：导航 / 回充 / 跟随 / 遥操等（含测试次数默认值）."""
    hub = get_registry()
    hub.ensure_builtins()
    registered = set(hub.tasks.names())
    by_id = {t["id"]: t for t in list_task_configs()}
    out: list[dict[str, Any]] = []
    for m in MISSION_CATALOG:
        meta = by_id.get(str(m["id"]), {})
        default_ep = int(m.get("default_episodes") or meta.get("episodes") or 1)
        out.append(
            {
                **m,
                "registered": str(m["id"]) in registered,
                "mode": meta.get("mode", "scenario"),
                "episodes": int(meta.get("episodes") or default_ep),
                "default_episodes": default_ep,
                "file": meta.get("file", f"configs/tasks/{m['id']}.yaml"),
                "group": "orchestration",
            }
        )
    return out


def _parse_proto_rpcs(proto: Path) -> list[dict[str, str]]:
    text = re.sub(r"//.*?$", "", proto.read_text(encoding="utf-8"), flags=re.M)
    pkg_m = _PACKAGE_RE.search(text)
    if not pkg_m:
        return []
    package = pkg_m.group(1)
    out: list[dict[str, str]] = []
    for svc in _SERVICE_RE.finditer(text):
        service, body = svc.group(1), svc.group(2)
        for rpc in _RPC_RE.finditer(body):
            method = rpc.group(1)
            out.append(
                {
                    "package": package,
                    "service": service,
                    "method": method,
                    "proto": proto.name,
                    "rpc": f"{package}.{service}/{method}",
                }
            )
    return out


def scan_automsgs_rpc_methods() -> list[dict[str, str]]:
    """Scan ``automsgs/proto/rpcs/*.proto`` for every Service/Method."""
    root = find_automsgs_root()
    if root is None:
        return []
    rpcs_dir = root / "proto" / "rpcs"
    if not rpcs_dir.is_dir():
        return []
    methods: list[dict[str, str]] = []
    for proto in sorted(rpcs_dir.glob("*.proto")):
        methods.extend(_parse_proto_rpcs(proto))
    return methods


def _autofill_rpc_payload(rpc: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Proto → default JSON + form fields (empty if descriptors unavailable)."""
    if not rpc:
        return {}, []
    info = describe_rpc_request(rpc)
    body = info.get("default_body") if isinstance(info.get("default_body"), dict) else {}
    fields = info.get("fields") if isinstance(info.get("fields"), list) else []
    return body or {}, fields or []


def list_named_rpc_tasks() -> list[dict[str, Any]]:
    """全部 gRPC 命令：扫描 proto，并用 NAMED_RPC_CATALOG 覆盖友好元数据."""
    scanned = scan_automsgs_rpc_methods()
    overrides = {str(t.get("rpc") or ""): t for t in NAMED_RPC_CATALOG if t.get("rpc")}
    # Also index curated by service/method for older entries.
    by_sm = {
        (str(t.get("service") or ""), str(t.get("method") or "")): t
        for t in NAMED_RPC_CATALOG
    }

    items: list[dict[str, Any]] = []
    seen_rpc: set[str] = set()

    for m in scanned:
        rpc = m["rpc"]
        if rpc in seen_rpc:
            continue
        seen_rpc.add(rpc)
        ov = overrides.get(rpc) or by_sm.get((m["service"], m["method"]))
        group = (
            (ov or {}).get("group")
            or _RPC_GROUP_LABELS.get(m["service"])
            or m["service"].replace("Service", "")
        )
        svc_short = m["service"].removesuffix("Service") or m["service"]
        auto_body, auto_fields = _autofill_rpc_payload(rpc)
        base = {
            "id": f"rpc.{svc_short}.{m['method']}".lower(),
            "label": m["method"],
            "description": f"{m['service']}.{m['method']}",
            "group": group,
            "service": m["service"],
            "method": m["method"],
            "rpc": rpc,
            "proto": m["proto"],
            "default_body": auto_body,
            "fields": auto_fields,
            "kind": "named",
        }
        if ov:
            # Curated entry wins for UX fields / Chinese labels / defaults.
            curated_body = ov.get("default_body")
            curated_fields = ov.get("fields")
            merged = {
                **base,
                **{k: v for k, v in ov.items() if k not in {"kind", "default_body", "fields"}},
                "kind": "named",
                "group": ov.get("group") or base["group"],
                "rpc": rpc,
                "service": m["service"],
                "method": m["method"],
                "proto": m["proto"],
                # Prefer curated defaults; fall back to proto autofill.
                "default_body": curated_body if isinstance(curated_body, dict) and curated_body else auto_body,
                "fields": curated_fields if isinstance(curated_fields, list) and curated_fields else auto_fields,
            }
            # Keep curated id when present (e.g. rpc.navigate).
            if ov.get("id"):
                merged["id"] = ov["id"]
            items.append(merged)
        else:
            items.append(base)

    # Append curated-only entries that somehow weren't found in protos.
    for ov in NAMED_RPC_CATALOG:
        rpc = str(ov.get("rpc") or "")
        if rpc and rpc not in seen_rpc:
            auto_body, auto_fields = _autofill_rpc_payload(rpc)
            curated_body = ov.get("default_body")
            curated_fields = ov.get("fields")
            items.append(
                {
                    **ov,
                    "kind": "named",
                    "group": ov.get("group") or "其他",
                    "default_body": curated_body
                    if isinstance(curated_body, dict) and curated_body
                    else auto_body,
                    "fields": curated_fields
                    if isinstance(curated_fields, list) and curated_fields
                    else auto_fields,
                }
            )

    # Stable order: group label, then service, then method.
    items.sort(key=lambda t: (t.get("group") or "", t.get("service") or "", t.get("method") or ""))
    return items


def task_kinds_payload() -> dict[str, Any]:
    """三类任务：定时 / 编排 / 命令."""
    missions = list_missions()
    named = list_named_rpc_tasks()
    return {
        "orchestration": {
            "id": "orchestration",
            "label": "编排任务",
            "description": "导航 · 回充 · 跟随 · 遥操等组合，可设测试次数",
            "items": missions,
        },
        "schedule": {
            "id": "schedule",
            "label": "定时任务",
            "description": "任意任务组合编排；支持同步 / 异步执行与间隔重复",
            "execution_modes": [
                {"id": "sync", "label": "同步执行", "description": "按顺序串行"},
                {"id": "async", "label": "异步执行", "description": "并行调度"},
            ],
            "items": missions,
            "named_items": named,
        },
        "named": {
            "id": "named",
            "label": "命令任务",
            "description": "按 gRPC Service/Method 直接调用",
            "items": named,
        },
    }


_PLANNER_META: dict[str, dict[str, str]] = {
    "pure_pursuit": {"family": "classic", "label": "Pure Pursuit", "blurb": "几何跟踪基线"},
    "dwa": {"family": "classic", "label": "DWA", "blurb": "动态窗口局部规划"},
    "teb": {"family": "classic", "label": "TEB", "blurb": "时弹性带（简化）"},
    "mpc": {"family": "classic", "label": "MPC", "blurb": "模型预测控制"},
    "nmpc": {"family": "classic", "label": "NMPC", "blurb": "非线性 MPC 别名"},
    "mppi": {"family": "classic", "label": "MPPI", "blurb": "对接 autonomy.control"},
    "autonomy_mppi": {"family": "classic", "label": "Autonomy MPPI", "blurb": "栈内 MPPI"},
    "policy": {"family": "learning", "label": "Policy ONNX", "blurb": "学习策略 / ONNX"},
    "cadrl": {"family": "learning", "label": "CADRL", "blurb": "社交避障 DRL"},
    "crowdnav": {"family": "learning", "label": "CrowdNav", "blurb": "人群导航"},
    "rlca": {"family": "learning", "label": "RLCA", "blurb": "强化学习避碰"},
    "sarl": {"family": "learning", "label": "SARL", "blurb": "社交注意力"},
    "sarl_star": {"family": "learning", "label": "SARL*", "blurb": "SARL 增强"},
    "applr": {"family": "learning", "label": "APPLR", "blurb": "自适应偏好"},
    "dragon": {"family": "learning", "label": "Dragon", "blurb": "Arena Dragon"},
    "trail": {"family": "learning", "label": "TRAIL", "blurb": "Arena TRAIL"},
    "drlvo": {"family": "learning", "label": "DRL-VO", "blurb": "视觉里程计 DRL"},
    "hybrid": {"family": "hybrid", "label": "Hybrid", "blurb": "经典 + 学习切换"},
    "switching": {"family": "hybrid", "label": "Switching", "blurb": "切换规划"},
}

_FAMILY_LABEL = {
    "classic": "经典控制",
    "learning": "学习模型",
    "hybrid": "混合策略",
}

# autonomy.control ControllerServer plugins (dir stem without ``_controller``).
_CONTROLLER_META: dict[str, dict[str, str]] = {
    "pure_pursuit": {"family": "classic", "label": "Pure Pursuit", "blurb": "几何跟踪"},
    "mppi": {"family": "classic", "label": "MPPI", "blurb": "采样轨迹优化"},
    "nmpc": {"family": "classic", "label": "NMPC", "blurb": "非线性 MPC"},
    "graceful": {"family": "classic", "label": "Graceful", "blurb": "平滑控制律"},
    "neupan": {"family": "classic", "label": "NeuPAN", "blurb": "神经规划控制"},
    "sando": {"family": "classic", "label": "Sando", "blurb": "Sando 控制器"},
    "policy": {"family": "learning", "label": "Policy", "blurb": "学习策略控制"},
}

# Planner → suggested autonomy.control plugin (empty = local-only).
_PLANNER_DEFAULT_CONTROLLER: dict[str, str] = {
    "pure_pursuit": "pure_pursuit",
    "mppi": "mppi",
    "autonomy_mppi": "mppi",
    "nmpc": "nmpc",
    "mpc": "nmpc",
    "policy": "policy",
    "cadrl": "policy",
    "crowdnav": "policy",
    "rlca": "policy",
    "sarl": "policy",
    "sarl_star": "policy",
    "applr": "policy",
    "dragon": "policy",
    "trail": "policy",
    "drlvo": "policy",
    "dwa": "",
    "teb": "",
    "hybrid": "",
    "switching": "",
}


def _autonomy_control_root() -> Path | None:
    """Locate ``autonomy/control/controller`` relative to the monorepo."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "autonomy" / "control" / "controller"
        if candidate.is_dir():
            return candidate
    env_raw = (os.environ.get("AUTONOMY_ROOT") or "").strip()
    if env_raw:
        env = Path(env_raw)
        for candidate in (
            env / "autonomy" / "control" / "controller",
            env / "control" / "controller",
        ):
            if candidate.is_dir():
                return candidate
    return None


def list_controllers(active: str | None = None) -> list[dict[str, Any]]:
    """Controllers available from autonomy.control (+ known aliases)."""
    discovered: set[str] = set(_CONTROLLER_META)
    root = _autonomy_control_root()
    if root is not None:
        for path in root.iterdir():
            if not path.is_dir() or path.name.startswith("."):
                continue
            name = path.name
            if name.endswith("_controller"):
                name = name[: -len("_controller")]
            if name:
                discovered.add(name)

    active_id = (active or "").strip().lower()
    out: list[dict[str, Any]] = []
    for name in sorted(discovered):
        meta = _CONTROLLER_META.get(
            name, {"family": "classic", "label": name, "blurb": name}
        )
        family = meta.get("family") or "classic"
        out.append(
            {
                "id": name,
                "label": meta.get("label") or name,
                "blurb": meta.get("blurb") or name,
                "family": family,
                "family_label": _FAMILY_LABEL.get(family, family),
                "plugin": f"{name}_controller",
                "active": bool(active_id) and name == active_id,
            }
        )
    order = {"classic": 0, "learning": 1, "hybrid": 2}
    out.sort(key=lambda m: (order.get(m["family"], 9), 0 if m["active"] else 1, m["id"]))
    return out


def _planner_default_controller(name: str, raw: dict[str, Any] | None = None) -> str:
    raw = raw or {}
    from_yaml = str(raw.get("controller") or "").strip().lower()
    if from_yaml:
        return from_yaml.removesuffix("_controller")
    return _PLANNER_DEFAULT_CONTROLLER.get(name, "")


def list_models(active_planner: str | None = None) -> list[dict[str, Any]]:
    """Planner / policy models available to the dashboard."""
    planners_dir = package_root() / "configs" / "planners"
    yaml_by_name: dict[str, dict[str, Any]] = {}
    if planners_dir.is_dir():
        for path in planners_dir.glob("*.yaml"):
            data = _safe_load(path)
            key = str(data.get("name") or path.stem)
            yaml_by_name[key] = {
                "file": path.relative_to(package_root()).as_posix(),
                "raw": data,
            }

    active = (active_planner or "pure_pursuit").strip().lower()
    out: list[dict[str, Any]] = []
    for name in list_planners():
        meta = _PLANNER_META.get(name, {"family": "classic", "label": name, "blurb": name})
        yml = yaml_by_name.get(name, {})
        raw = yml.get("raw") or {}
        model_path = str(raw.get("model_path") or "")
        model_exists = bool(model_path) and resolve_path(model_path).is_file()
        family = str(raw.get("type") or meta["family"])
        if family not in _FAMILY_LABEL:
            family = meta["family"]
        default_controller = _planner_default_controller(name, raw)
        out.append(
            {
                "id": name,
                "label": meta["label"],
                "blurb": meta["blurb"],
                "family": family,
                "family_label": _FAMILY_LABEL.get(family, family),
                "file": yml.get("file"),
                "model_path": model_path,
                "model_ready": model_exists if model_path else True,
                "active": name == active,
                "needs_weights": family == "learning",
                "default_controller": default_controller,
            }
        )
    # Stable order: classic → hybrid → learning, active first within family.
    order = {"classic": 0, "hybrid": 1, "learning": 2}
    out.sort(key=lambda m: (order.get(m["family"], 9), 0 if m["active"] else 1, m["id"]))
    return out


def list_datasets() -> list[dict[str, Any]]:
    """Existing collect outputs under data/datasets."""
    root = package_root() / "data" / "datasets"
    items: list[dict[str, Any]] = []
    if not root.is_dir():
        return items
    for path in sorted(root.iterdir()):
        if not path.is_dir():
            continue
        jsonl = list(path.glob("*.jsonl")) + list(path.rglob("episode_*.jsonl"))
        items.append(
            {
                "name": path.name,
                "path": path.relative_to(package_root()).as_posix(),
                "episodes": len(jsonl),
            }
        )
    return items[:40]


def _config_rel_path(config_path: str | None, experiment: dict[str, Any]) -> str:
    """Best-effort package-relative config path for the UI."""
    if config_path:
        return config_path
    raw = str(experiment.get("path") or "")
    marker = "configs/"
    if marker in raw:
        return marker + raw.split(marker, 1)[1]
    return "configs/default.yaml"


def list_worlds() -> list[str]:
    root = package_root() / "configs" / "worlds"
    if not root.is_dir():
        return []
    return sorted(p.stem for p in root.glob("*.yaml"))


# Allowlisted dotted keys the dashboard may override at run time.
ALLOWED_PARAM_KEYS: frozenset[str] = frozenset(
    {
        "experiment.seed",
        "task.mode",
        "task.episodes",
        "task.timeout_s",
        "task.goal_tolerance_m",
        "task.obstacles",
        "task.pedestrians",
        "task.moving_obstacles",
        "task.difficulty",
        "task.seed",
        "world.name",
        "simulator.name",
        "simulator.obstacles",
        "simulator.pedestrians",
        "simulator.moving_obstacles",
        "planner.name",
        "planner.controller",
        "planner.model_path",
        "humans.name",
        "humans.enabled",
        "humans.behavior",
        "humans.preferred_speed",
        "humans.radius",
    }
)


def _nested_get(data: dict[str, Any], dotted: str, default: Any = None) -> Any:
    cur: Any = data
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def params_schema(experiment: dict[str, Any]) -> dict[str, Any]:
    """UI schema + current values for parameterized overrides."""
    from benchmark.simulators.humans.behaviors import PRESETS

    sections = experiment.get("sections") or {}
    # Flatten section views for lookup: task.timeout_s → sections.task.timeout_s
    flat_src = {
        "experiment": experiment.get("experiment") or {},
        "task": sections.get("task") or {},
        "world": sections.get("world") or {},
        "simulator": sections.get("simulator") or {},
        "planner": sections.get("planner") or {},
        "humans": sections.get("humans") or {},
    }
    modes = list_modes()
    worlds = list_worlds()
    behaviors = sorted(PRESETS.keys())

    fields: list[dict[str, Any]] = [
        {
            "key": "experiment.seed",
            "label": "Seed",
            "type": "int",
            "group": "实验",
            "min": 0,
            "max": 999999,
            "step": 1,
        },
        {
            "key": "task.mode",
            "label": "Mode",
            "type": "select",
            "group": "任务",
            "options": modes,
        },
        {
            "key": "task.episodes",
            "label": "Episodes",
            "type": "int",
            "group": "任务",
            "min": 1,
            "max": 200,
            "step": 1,
        },
        {
            "key": "task.timeout_s",
            "label": "超时 (s)",
            "type": "number",
            "group": "任务",
            "min": 1,
            "max": 600,
            "step": 1,
        },
        {
            "key": "task.goal_tolerance_m",
            "label": "到点容差 (m)",
            "type": "number",
            "group": "任务",
            "min": 0.05,
            "max": 2.0,
            "step": 0.05,
        },
        {
            "key": "task.obstacles",
            "label": "静态障碍",
            "type": "int",
            "group": "场景密度",
            "min": 0,
            "max": 80,
            "step": 1,
        },
        {
            "key": "task.pedestrians",
            "label": "行人",
            "type": "int",
            "group": "场景密度",
            "min": 0,
            "max": 80,
            "step": 1,
        },
        {
            "key": "task.moving_obstacles",
            "label": "动态障碍",
            "type": "int",
            "group": "场景密度",
            "min": 0,
            "max": 40,
            "step": 1,
        },
        {
            "key": "task.difficulty",
            "label": "难度",
            "type": "range",
            "group": "场景密度",
            "min": 0,
            "max": 1,
            "step": 0.05,
            "show_when": {"task.mode": "parametrized"},
            "hint": "parametrized 模式缩放障碍/行人密度",
        },
        {
            "key": "world.name",
            "label": "世界",
            "type": "select",
            "group": "世界",
            "options": worlds,
        },
        {
            "key": "humans.behavior",
            "label": "行人行为",
            "type": "select",
            "group": "行人",
            "options": behaviors,
        },
        {
            "key": "humans.preferred_speed",
            "label": "行人速度",
            "type": "number",
            "group": "行人",
            "min": 0.2,
            "max": 2.5,
            "step": 0.1,
        },
        {
            "key": "humans.radius",
            "label": "行人半径",
            "type": "number",
            "group": "行人",
            "min": 0.1,
            "max": 0.6,
            "step": 0.05,
        },
        {
            "key": "humans.enabled",
            "label": "启用行人",
            "type": "bool",
            "group": "行人",
        },
    ]

    values: dict[str, Any] = {}
    for field in fields:
        key = str(field["key"])
        section, _, name = key.partition(".")
        default: Any
        if field["type"] == "bool":
            default = False
        elif field["type"] in {"int", "number", "range"}:
            default = 0
        else:
            default = ""
        # Prefer section values; fall back to experiment.seed for seed.
        val = _nested_get(flat_src, key, None)
        if val is None and key == "experiment.seed":
            val = (experiment.get("experiment") or {}).get("seed", 42)
        if val is None and key == "task.difficulty":
            val = 0.5
        if val is None:
            val = default
        if field["type"] == "int":
            try:
                val = int(val)
            except (TypeError, ValueError):
                val = int(default or 0)
        elif field["type"] in {"number", "range"}:
            try:
                val = float(val)
            except (TypeError, ValueError):
                val = float(default or 0)
        elif field["type"] == "bool":
            val = bool(val)
        else:
            val = str(val) if val is not None else ""
        values[key] = val

    return {"fields": fields, "values": values, "allowed": sorted(ALLOWED_PARAM_KEYS)}


def sanitize_params(params: dict[str, Any] | None) -> dict[str, Any]:
    """Keep only allowlisted dotted keys with JSON-serializable scalars."""
    if not params:
        return {}
    out: dict[str, Any] = {}
    for key, value in params.items():
        key_s = str(key).strip()
        if key_s not in ALLOWED_PARAM_KEYS:
            continue
        if isinstance(value, bool):
            out[key_s] = value
        elif isinstance(value, int):
            out[key_s] = int(value)
        elif isinstance(value, float):
            out[key_s] = float(value)
        elif isinstance(value, str):
            out[key_s] = value
        elif value is None:
            continue
        else:
            # coerce numbers from JSON that arrived as str
            text = str(value).strip()
            if text.lower() in {"true", "false"}:
                out[key_s] = text.lower() == "true"
            else:
                try:
                    if "." in text:
                        out[key_s] = float(text)
                    else:
                        out[key_s] = int(text)
                except ValueError:
                    out[key_s] = text
    return out


def params_to_overrides(params: dict[str, Any] | None) -> list[str]:
    """Convert sanitized params dict to load_config override strings."""
    clean = sanitize_params(params)
    overrides: list[str] = []
    for key, value in clean.items():
        if isinstance(value, bool):
            overrides.append(f"{key}={'true' if value else 'false'}")
        else:
            overrides.append(f"{key}={value}")
    return overrides


def catalog_payload(config_path: str | None = None) -> dict[str, Any]:
    experiment = load_experiment(config_path)
    rel = _config_rel_path(config_path, experiment)
    try:
        config_file = read_config_file(rel)
    except Exception:  # noqa: BLE001
        config_file = {"path": rel, "text": "", "data": {}}
    humans_sec = experiment.get("sections", {}).get("humans") or {}
    registry = get_registry_snapshot()
    sim_name = str(experiment.get("simulator_name") or "")
    if sim_name not in registry["simulators"]:
        sim_name = registry["simulators"][0] if registry["simulators"] else "isaac"
    registry = {**registry, "worlds": list_worlds()}
    return {
        "registry": registry,
        "configs": list_config_files(),
        "missions": list_missions(),
        "task_kinds": task_kinds_payload(),
        "named_tasks": list_named_rpc_tasks(),
        "tasks": list_task_configs(),
        "models": list_models(experiment.get("planner_name")),
        "controllers": list_controllers(
            str(
                ((experiment.get("sections") or {}).get("planner") or {}).get("controller")
                or ""
            )
        ),
        "datasets": list_datasets(),
        "experiment": experiment,
        "config_path": rel,
        "config_text": str(config_file.get("text") or ""),
        "scene": {
            "simulator": sim_name,
            "humans": str(humans_sec.get("name") or "social_force"),
        },
        "collect": collect_options_from_config(config_path),
        "replay": replay_options_from_config(config_path),
        "params": params_schema(experiment),
        "bicmap": bicmap_payload(),
        "layout": {"compact": True},
        "curve_channels": [
            {"id": "v", "label": "线速度 v", "unit": "m/s"},
            {"id": "w", "label": "角速度 ω", "unit": "rad/s"},
            {"id": "a", "label": "线加速度 a", "unit": "m/s²"},
            {"id": "alpha", "label": "角加速度 α", "unit": "rad/s²"},
            {"id": "jerk", "label": "Jerk", "unit": "m/s³"},
            {"id": "jerk_angular", "label": "角 Jerk", "unit": "rad/s³"},
        ],
    }
