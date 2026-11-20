#!/usr/bin/env python3
"""Exercise installed native nodes and read-only RPCs in private Linux namespaces."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", required=True, type=Path)
    parser.add_argument("--config-dir", required=True, type=Path)
    parser.add_argument("--gui", action="store_true", help="Open Autoviz briefly on the local X11 display")
    parser.add_argument("--isolated-from", nargs=4, help=argparse.SUPPRESS)
    args = parser.parse_args()
    prefix = args.prefix.absolute()
    config = args.config_dir.absolute()
    namespaces = ("user", "ipc", "net", "mnt")
    if not args.isolated_from:
        return subprocess.call([
            "unshare", "--user", "--map-root-user", "--ipc", "--net", "--mount",
            "--propagation", "private", "--fork", sys.executable, __file__,
            *(["--gui"] if args.gui else []),
            "--prefix", str(prefix), "--config-dir", str(config), "--isolated-from",
            *[os.readlink(f"/proc/self/ns/{name}") for name in namespaces],
        ])

    for namespace, original in zip(namespaces, args.isolated_from):
        if os.readlink(f"/proc/self/ns/{namespace}") == original:
            raise RuntimeError(f"Refusing to run without private {namespace} namespace")
    # Fail before starting nodes if isolation setup is unavailable.
    subprocess.run(["mount", "-t", "tmpfs", "-o", "mode=1777", "tmpfs", "/dev/shm"], check=True)
    subprocess.run(["ip", "link", "set", "lo", "up"], check=True)
    logs = Path(tempfile.mkdtemp(prefix="autonomy-runtime-smoke-", dir=prefix.parent))
    private_tmp = logs / "tmp"
    private_tmp.mkdir()
    if args.gui:
        if not os.environ.get("DISPLAY") or not Path("/tmp/.X11-unix").is_dir():
            raise RuntimeError("--gui requires a local X11 display")
        (private_tmp / ".X11-unix").mkdir()
        subprocess.run(["mount", "--bind", "/tmp/.X11-unix", str(private_tmp / ".X11-unix")], check=True)
    # The local discovery backend uses a fixed /tmp event file, independent of domain ID.
    subprocess.run(["mount", "--rbind", str(private_tmp), "/tmp"], check=True)
    print(f"Logs: {logs}", flush=True)
    env = dict(os.environ, AUTOLINK_DOMAIN_ID="201",
               AUTOLINK_TOPOLOGY_BACKEND="local", AUTOLINK_IP="127.0.0.1",
               XDG_CONFIG_HOME=str(logs / "config"), XDG_CACHE_HOME=str(logs / "cache"),
               GLOG_logtostderr="1")
    for name in ("LD_LIBRARY_PATH", "AUTONOMY_PATH", "AUTOLINK_PATH", "AUTOLINK_DISTRIBUTION_HOME",
                 "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH", "AUTOVIZ_OGRE_PLUGIN_DIR", "AUTOVIZ_OGRE_MEDIA_PATH"):
        env.pop(name, None)
    bridge = str(prefix / "bin/autonomy.bridge")
    processes = []
    handles = []

    def run(name, command, extra_env=None):
        try:
            result = subprocess.run(command, cwd=logs, env=env | (extra_env or {}), text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
        except subprocess.TimeoutExpired as error:
            (logs / f"{name}.log").write_bytes((error.stdout or b'') + (error.stderr or b''))
            raise
        (logs / f"{name}.log").write_text(result.stdout + result.stderr)
        result.check_returncode()
        return result.stdout

    def start(name, command):
        handle = (logs / f"{name}.log").open("w")
        handles.append(handle)
        process = subprocess.Popen(command, cwd=logs, env=env, stdout=handle,
                                   stderr=subprocess.STDOUT)
        processes.append((name, process))

    def check_alive():
        for name, process in processes:
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited with {process.returncode}; see {logs}")

    def rpc(method, payload="{}"):
        response = json.loads(run(method.replace("/", "-"), [
            bridge, "call", method, "-d", payload,
            "--target", "127.0.0.1:5005", "--timeout", "3",
        ]))
        assert isinstance(response.get("status"), dict), f"{method} omitted application status"
        if response.get("status", {}).get("code", 0) not in (0, "STATUS_OK", "OK"):
            raise RuntimeError(f"{method} returned a non-OK application status; see {logs}")
        return response

    bridge_config = logs / "bridge.pb.txt"
    bridge_config.write_text('identity { robot_id: "demo-smoke" model: "demo-smoke" }\n'
                             'grpc { host: "127.0.0.1" port: 5005 '
                             'num_grpc_threads: 2 num_event_threads: 2 }\n')
    try:
        autoviz = prefix / "bin/autoviz"
        if autoviz.exists():
            run("autoviz-help", [str(autoviz), "--help"], {"QT_QPA_PLATFORM": "offscreen"})
            print("PASS autoviz launcher and Qt platform plugin without LD_LIBRARY_PATH", flush=True)
        if args.gui:
            # shortcut: use an idle bus here; live topic rendering needs a separate data acceptance test.
            # Autoviz ignores ordinary close events during its first five seconds.
            run("autoviz-gui", [str(autoviz), "--splash-screen", ""],
                {"QT_QPA_PLATFORM": "xcb", "AUTOVIZ_SMOKE_QUIT_MS": "6000"})
            assert "Autoviz RenderSystem ready" in (logs / "autoviz-gui.log").read_text()
            print("PASS autoviz window, Ogre rendering initialization and clean exit", flush=True)
        for mode in ("dry-run", "self-test"):
            run(mode, [bridge, "--conf", str(bridge_config), f"--{mode}"])
            print(f"PASS bridge {mode}", flush=True)
        start("task", [str(prefix / "bin/autonomy.task"),
                       "--config_directory", str(config / "task")])
        start("localization", [str(prefix / "bin/autonomy.localization"),
              "--configuration_directory", str(config / "localization/cartographer"),
              "--configuration_basename", "backpack_2d.lua",
              "--start_trajectory_with_default_topics=false"])
        start("bridge", [bridge, "--conf", str(bridge_config)])
        deadline = time.monotonic() + 30
        while True:
            check_alive()
            try:
                heartbeat = rpc("SystemService/Heartbeat", '{"sequence":42}')
                break
            except subprocess.CalledProcessError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.25)
        assert int(heartbeat["sequence"]) == 42
        assert int(heartbeat["robot_time_ns"]) > 0
        print("PASS RPC heartbeat sequence and timestamp", flush=True)
        assert rpc("SystemService/GetInfo")["model"] == "demo-smoke"
        print("PASS RPC identity", flush=True)
        assert rpc("SystemService/GetStatus")["state"] == "SYSTEM_STATE_IDLE"
        assert not rpc("SystemService/GetActiveGoal").get("goal_id")
        assert rpc("LocalizationService/GetStatus").get("state", "LOCALIZATION_STATE_UNKNOWN") == "LOCALIZATION_STATE_UNKNOWN"
        print("PASS RPC system status, active goal and localization status", flush=True)
        time.sleep(5)
        check_alive()
        assert "TaskServer configured" in (logs / "task.log").read_text()
        assert "Cartographer backend started" in (logs / "localization.log").read_text()
        print("PASS task and localization startup; all three nodes remain alive", flush=True)
    finally:
        cleanup_failed = False
        for name, process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT if name == "task" else signal.SIGTERM)
        for name, process in reversed(processes):
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                cleanup_failed = True
                print(f"FAIL {name} required SIGKILL during cleanup", file=sys.stderr)
            if process.returncode != 0:
                cleanup_failed = True
            print(f"STOP {name}: exit={process.returncode}", flush=True)
        for handle in handles:
            handle.close()
        if cleanup_failed:
            raise RuntimeError(f"Node shutdown failed; see {logs}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
