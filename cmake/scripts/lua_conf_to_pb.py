#!/usr/bin/env python3
"""Dump config/*.lua → autonomy/<mod>/conf/*.pb.txt (protobuf text format)."""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config"


def camel_to_snake(name: str) -> str:
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


CRITIC = {
    "ConstraintCritic": "constraint_critic",
    "GoalCritic": "goal_critic",
    "GoalAngleCritic": "goal_angle_critic",
    "PreferForwardCritic": "prefer_forward_critic",
    "CostCritic": "cost_critic",
    "PathAlignCritic": "path_align_critic",
    "PathFollowCritic": "path_follow_critic",
    "PathAngleCritic": "path_angle_critic",
    "ObstaclesCritic": "obstacles_critic",
    "VelocityDeadbandCritic": "velocity_deadband_critic",
    "TwirlingCritic": "twirling_critic",
    "GridObstaclesCritic": "grid_obstacles_critic",
    "PathHandler": "path_handler",
    "TrajectoryVisualizer": "trajectory_visualizer",
    "AckermannConstraints": "ackermann_constraints",
}


def luajit_dump(entry: str) -> dict:
    import tempfile

    script = r"""
local config_dir = os.getenv("AUTONOMY_LUA_CONFIG_DIR")
local entry = os.getenv("AUTONOMY_LUA_ENTRY")
local function is_array(t)
  if type(t) ~= "table" then return false end
  local n = 0
  for k,_ in pairs(t) do
    if type(k) ~= "number" then return false end
    n = n + 1
  end
  if n == 0 then return false end
  return n == #t
end
local function enc(v)
  local tv = type(v)
  if v == nil then return "null"
  elseif tv == "boolean" then return v and "true" or "false"
  elseif tv == "number" then
    if v ~= v then return "null" end
    return string.format("%.17g", v)
  elseif tv == "string" then
    return '"' .. v:gsub('\\','\\\\'):gsub('"','\\"'):gsub('\n','\\n') .. '"'
  elseif tv == "table" then
    if is_array(v) then
      local parts = {}
      for i = 1, #v do parts[i] = enc(v[i]) end
      return "[" .. table.concat(parts, ",") .. "]"
    end
    local parts = {}
    for k, val in pairs(v) do
      parts[#parts+1] = '"' .. tostring(k) .. '":' .. enc(val)
    end
    return "{" .. table.concat(parts, ",") .. "}"
  end
  return "null"
end
local included = {}
function include(name)
  local path = config_dir .. "/" .. name
  if included[path] then return end
  included[path] = true
  assert(loadfile(path))()
end
local chunk = assert(loadfile(config_dir .. "/" .. entry))
local ret = chunk()
local t = (type(ret) == "table" and ret) or AUTONOMY
print(enc(t))
"""
    with tempfile.NamedTemporaryFile("w", suffix=".lua", delete=False) as f:
        f.write(script)
        path = f.name
    try:
        out = subprocess.check_output(
            ["luajit", path],
            cwd=str(ROOT),
            text=True,
            env={
                **dict(**{k: v for k, v in __import__("os").environ.items()}),
                "AUTONOMY_LUA_CONFIG_DIR": str(CONFIG),
                "AUTONOMY_LUA_ENTRY": entry,
            },
        )
    finally:
        Path(path).unlink(missing_ok=True)
    return json.loads(out)


# Lua-only / ignored-by-C++ keys that are not on the protos.
DROP_KEYS = frozenset(
    {
        "obstacle_max_range",  # SensorSource has no range field; C++ ignored it
        "publish_frequency",  # Costmap2DOptions has no publish_frequency
        "name",  # MPPIControllerOptions has no name (costmap.name is kept via context)
        "plugin",  # top-level mppi plugin string not on proto (layer plugins kept)
    }
)


def emit(val, indent: int = 0, parent_key: str = "") -> list[str]:
    lines: list[str] = []
    if isinstance(val, list):
        if not val:
            return lines
        raise TypeError(f"list must be handled by emit_field, got {val!r}")
    if isinstance(val, dict):
        for k, v in val.items():
            # Keep costmap/layer `name` and `plugin`; drop only under mppi opts.
            if k in DROP_KEYS:
                if k in ("name", "plugin") and parent_key in (
                    "",
                    "controller_options",
                    "mppi_controller_options",
                    "graceful_controller_options",
                    "nmpc_controller_options",
                    "pure_pursuit_controller_options",
                ):
                    continue
                if k == "obstacle_max_range":
                    continue
                if k == "publish_frequency" and parent_key in (
                    "costmap",
                    "costmap_2d_options",
                ):
                    continue
            lines.extend(emit_field(k, v, indent, parent_key=parent_key))
    elif val is None:
        pass
    else:
        raise TypeError(f"unexpected top-level value: {type(val)}")
    return lines


MAP_FIELDS = frozenset({"sensor_sources"})


def emit_field(
    key: str, val, indent: int = 0, parent_key: str = ""
) -> list[str]:
    sp = "  " * indent
    lines: list[str] = []
    if val is None:
        return lines
    if isinstance(val, list):
        for item in val:
            if isinstance(item, dict):
                lines.append(f"{sp}{key} {{")
                lines.extend(emit(item, indent + 1, parent_key=key))
                lines.append(f"{sp}}}")
            else:
                if isinstance(item, bool):
                    lines.append(f"{sp}{key}: {'true' if item else 'false'}")
                elif isinstance(item, str):
                    esc = item.replace("\\", "\\\\").replace('"', '\\"')
                    lines.append(f'{sp}{key}: "{esc}"')
                else:
                    lines.append(f"{sp}{key}: {item}")
        return lines
    if isinstance(val, dict):
        if key in MAP_FIELDS and val and all(
            isinstance(v, dict) for v in val.values()
        ):
            for mk, mv in val.items():
                lines.append(f"{sp}{key} {{")
                esc = str(mk).replace("\\", "\\\\").replace('"', '\\"')
                lines.append(f'{sp}  key: "{esc}"')
                lines.append(f"{sp}  value {{")
                lines.extend(emit(mv, indent + 2, parent_key=key))
                lines.append(f"{sp}  }}")
                lines.append(f"{sp}}}")
            return lines
        lines.append(f"{sp}{key} {{")
        lines.extend(emit(val, indent + 1, parent_key=key))
        lines.append(f"{sp}}}")
        return lines
    if isinstance(val, bool):
        lines.append(f"{sp}{key}: {'true' if val else 'false'}")
    elif isinstance(val, str):
        esc = val.replace("\\", "\\\\").replace('"', '\\"')
        lines.append(f'{sp}{key}: "{esc}"')
    elif isinstance(val, float):
        if val == int(val) and abs(val) < 1e9:
            lines.append(f"{sp}{key}: {int(val)}")
        else:
            lines.append(f"{sp}{key}: {val}")
    else:
        lines.append(f"{sp}{key}: {val}")
    return lines


def rename_dict(obj):
    if isinstance(obj, list):
        return [rename_dict(x) for x in obj]
    if not isinstance(obj, dict):
        return obj
    out = {}
    for k, v in obj.items():
        nk = CRITIC.get(k, camel_to_snake(k) if k[:1].isupper() else k)
        out[nk] = rename_dict(v)
    return out


def fix_costmap(c: dict) -> dict:
    c = rename_dict(c)
    if "footprint" in c and isinstance(c["footprint"], list):
        c["footprint"] = {"points": c["footprint"]}
    # drop publish_frequency if present (may not be on Costmap2DOptions)
    return c


def transform_controller(c: dict) -> dict:
    c = rename_dict(c)
    out = {}
    for k in (
        "controller_frequency",
        "failure_tolerance",
        "publish_zero_velocity",
        "controller_plugins",
        "controller_plugin_libraries",
    ):
        if k in c:
            out[k] = c[k]
    if "costmap" in c:
        out["costmap_2d_options"] = fix_costmap(c["costmap"])
    checker = {}
    if "goal_checker" in c:
        checker["goal_checker"] = c["goal_checker"]
    if "progress_checker" in c:
        checker["progress_checker"] = c["progress_checker"]
    if checker:
        out["checker_options"] = checker
    for src, dst in (
        ("mppi_controller", "mppi_controller_options"),
        ("graceful_controller", "graceful_controller_options"),
        ("pure_pursuit_controller", "pure_pursuit_controller_options"),
        ("nmpc_controller", "nmpc_controller_options"),
    ):
        if src in c:
            out[dst] = rename_dict(c[src])
    return out


def transform_planner(p: dict) -> dict:
    p = rename_dict(p)
    out = {}
    # planning_options.proto field names
    mapping = {
        "navfn_planner": "navfn",
        "dijkstra_planner": "dijkstra",
        "theta_star_planner": "theta_star",
        "simple_smoother": "simple_smoother",
        "costmap": "costmap",
    }
    for k, v in p.items():
        if k in mapping:
            key = mapping[k]
            out[key] = fix_costmap(v) if key == "costmap" else v
        else:
            out[k] = v
    return out


def write_msg(path: Path, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = emit(data, 0)
    path.write_text("\n".join(lines) + "\n")
    print(f"wrote {path.relative_to(ROOT)} ({len(lines)} lines)")


def autonomy_msg(raw: dict) -> dict:
    msg = {}
    if "map" in raw and isinstance(raw["map"], dict):
        msg["map_options"] = raw["map"]
    if "planning" in raw and isinstance(raw["planning"], dict):
        msg["planner_options"] = transform_planner(raw["planning"])
    if "controller" in raw and isinstance(raw["controller"], dict):
        msg["controller_options"] = transform_controller(raw["controller"])
    if "navigator" in raw and isinstance(raw["navigator"], dict):
        msg["navigator_options"] = rename_dict(raw["navigator"])
    if "transform" in raw and isinstance(raw["transform"], dict):
        msg["transform_options"] = rename_dict(raw["transform"])
    if "perception" in raw and isinstance(raw["perception"], dict):
        msg["perception_options"] = rename_dict(raw["perception"])
    return msg


def main():
    raw = luajit_dump("autonomy.lua")
    msg = autonomy_msg(raw)
    write_msg(ROOT / "autonomy/system/conf/autonomy.pb.txt", msg)
    if "controller_options" in msg:
        write_msg(
            ROOT / "autonomy/control/conf/controller.pb.txt",
            msg["controller_options"],
        )
    if "planner_options" in msg:
        write_msg(
            ROOT / "autonomy/planning/conf/planner.pb.txt",
            msg["planner_options"],
        )
    if "map_options" in msg:
        write_msg(ROOT / "autonomy/map/conf/map.pb.txt", msg["map_options"])
    if "navigator_options" in msg:
        write_msg(
            ROOT / "autonomy/task/conf/navigator.pb.txt",
            msg["navigator_options"],
        )
    if "perception_options" in msg:
        write_msg(
            ROOT / "autonomy/perception/conf/perception.pb.txt",
            msg["perception_options"],
        )
    if "transform_options" in msg:
        write_msg(
            ROOT / "autonomy/transform/conf/transform.pb.txt",
            msg["transform_options"],
        )

    raw_e = luajit_dump("exploration_autonomy.lua")
    msg_e = autonomy_msg(raw_e)
    write_msg(ROOT / "autonomy/system/conf/exploration.pb.txt", msg_e)
    if "perception_options" in msg_e:
        write_msg(
            ROOT / "autonomy/perception/conf/perception_exploration.pb.txt",
            msg_e["perception_options"],
        )

    # bridge / prediction empty stubs
    write_msg(
        ROOT / "autonomy/bridge/conf/bridge.pb.txt",
        {
            "use_grpc": True,
            "grpc": {
                "host": "127.0.0.1",
                "port": 5005,
                "num_grpc_threads": 5,
                "num_event_threads": 5,
                "enable_ssl_encryption": False,
                "enable_google_auth": False,
            },
        },
    )
    write_msg(ROOT / "autonomy/prediction/conf/prediction.pb.txt", {})
    print("done")


if __name__ == "__main__":
    main()
# Legacy: config/ was removed. Re-run only if you restore a lua tree snapshot.
