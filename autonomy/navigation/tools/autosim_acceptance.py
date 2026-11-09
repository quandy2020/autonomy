#!/usr/bin/env python3
"""Opt-in multiprocess Habitat acceptance; run in isolated IPC, mount and network namespaces."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import autolink
from google.protobuf import text_format
from automsgs.actions.nav_actions_pb2 import NavigateToPoseAction as Action
from automsgs.msgs.builtin_interfaces.time_pb2 import Time
from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.status_msgs.status_msgs_pb2 import OK, NAVIGATION_CANCELLED, NAVIGATION_PATH_BLOCKED
from navigation.proto.lifecycle_pb2 import (
    Degradation, DEGRADATION_MODE_NORMAL, DEGRADATION_MODE_REFLEX_ONLY,
    DEGRADATION_MODE_SAFE_STOP,
)
from navigation.proto.navigation_pb2 import GoalSpec, NavigationStatus, BehaviorRequest, RECOVERY_RUNNING
from navigation.proto.config_pb2 import PlatformProfile, NavigationConfig, SafetyConfig
from navigation.proto.observation_pb2 import Observation

from autosim_acceptance_scene import atomic_json


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def modules(text):
    """Split the source DAG at top-level module_config braces."""
    chunks = []
    start = 0
    while True:
        begin = text.find("module_config", start)
        if begin < 0:
            return chunks
        brace = text.index("{", begin)
        depth = 1
        end = brace + 1
        while depth:
            depth += (text[end] == "{") - (text[end] == "}")
            end += 1
        chunks.append(text[begin:end])
        start = end


class Acceptance:
    def __init__(self, root: Path, build: Path, output: Path):
        self.root, self.build, self.output = root, build, output
        self.navigation = root / "autonomy/navigation"
        self.processes = {}
        self.files = []
        self.counts = {}
        self.latest = {}
        self.mode = None
        self.degradations = []
        self.feedbacks = []
        self.command_samples = []
        self.status_transitions = []
        self.command_id = 0
        self.goal_sequence = 0
        self.results = []
        self.output.mkdir(parents=True, exist_ok=True)
        require(not (output / "scene_state.json").exists(),
                "use a fresh output directory for each run")
        namespaces = {name: os.readlink(f"/proc/self/ns/{name}") for name in ["ipc", "mnt", "net"]}
        require(namespaces["net"] != os.readlink("/proc/1/ns/net"),
                "acceptance requires run_component_integration.sh network isolation")
        atomic_json(self.output / "isolation.json", {
            "namespaces": namespaces,
            "container_network_namespace": os.readlink("/proc/1/ns/net"),
            "domain_id": os.environ.get("AUTOLINK_DOMAIN_ID", "80"),
            "interfaces": json.loads(subprocess.check_output(["ip", "-j", "link"], text=True)),
        })
        autolink.init("navigation_autosim_acceptance")
        self.node = autolink.Node("navigation_autosim_acceptance")
        for name, channel, dtype in [
            ("depth", "/camera/depth/image_raw", Image),
            ("rgb", "/camera/rgb/image_raw", Image),
            ("observation", "/navigation/observation", Observation),
            ("command", "/cmd_vel", TwistStamped),
            ("raw", "/navigation/cmd_vel_raw", Twist),
            ("status", "/navigation/status", NavigationStatus),
            ("clock", "/clock", Time),
            ("degradation", "/guardian/degradation", Degradation),
        ]:
            def callback(message, name=name):
                self.latest[name] = message
                self.counts[name] = self.counts.get(name, 0) + 1
                if name == "degradation":
                    self.mode = message.to
                    self.degradations.append({"from": getattr(message, "from"),
                                              "to": message.to,
                                              "reason": message.reason})
                elif name == "status":
                    transition = {"session": message.session_id, "goal_epoch": message.goal_epoch,
                                  "active": message.has_active_goal, "stage": message.stage,
                                  "recovery_state": message.recovery_state, "recoveries": message.recovery_count,
                                  "reason": message.reason}
                    if not self.status_transitions or any(
                            self.status_transitions[-1].get(k) != v for k, v in transition.items()):
                        transition["received"] = time.monotonic()
                        self.status_transitions.append(transition)
                        atomic_json(self.output / "status_transitions.json", self.status_transitions)
                elif name == "command":
                    self.command_samples.append({
                        "received": time.monotonic(),
                        "velocity": [message.twist.linear.x, message.twist.linear.y,
                                     message.twist.angular.z],
                    })
            self.node.create_reader(channel, callback, dtype)
        self.goal_writer = self.node.create_writer("/navigation/goal", GoalSpec)
        self.behavior_writer = self.node.create_writer("/navigation/behavior_request", BehaviorRequest)
        self.client = autolink.ActionClient(
            self.node, "/navigate_to_pose", goal_type=Action.Goal,
            feedback_type=Action.Feedback, result_type=Action.Result)
        self.prepare_dags()

    def prepare_dags(self):
        for filename in ["navigation.pb.txt", "safety.pb.txt", "guardian.pb.txt"]:
            shutil.copy2(self.navigation / "conf" / filename, self.output / filename)
        # The independent scene oracle represents a 0.30 m robot. Snapshot
        # its matching profile instead of borrowing a mutable deployment size.
        profile = PlatformProfile()
        text_format.Parse((self.navigation / "conf/platform/differential_drive.pb.txt").read_text(), profile)
        profile.footprint_radius = 0.3
        profile_path = self.output / "differential_drive.pb.txt"
        profile_path.write_text(text_format.MessageToString(profile))
        for filename, dtype in [("navigation.pb.txt", NavigationConfig), ("safety.pb.txt", SafetyConfig)]:
            path = self.output / filename
            config = dtype()
            text_format.Parse(path.read_text(), config)
            config.platform_profile = str(profile_path)
            if filename == "navigation.pb.txt":
                config.policy.exploration.log_transitions = True
            path.write_text(text_format.MessageToString(config))
        nav = modules((self.navigation / "dag/navigation.dag").read_text())
        require(len(nav) == 3, "navigation DAG layout changed")
        content = {"navigation": "\n".join(nav[:2]), "behavior": nav[2]}
        for name in ["safety", "guardian"]:
            content[name] = (self.navigation / f"dag/{name}.dag").read_text()
        for name, text in content.items():
            text = text.replace('"navigation/conf/',
                                '"' + str(self.output) + "/")
            (self.output / f"{name}.dag").write_text(text)

    def start(self, name):
        log = (self.output / f"{name}-{time.monotonic_ns()}.log").open("w")
        self.files.append(log)
        if name == "autosim":
            args = [sys.executable, str(self.navigation / "tools/autosim_acceptance_scene.py"),
                    "--directory", str(self.output), "--autosim-root", str(self.root / "autosim")]
        else:
            args = [str(self.build / "bin/mainboard"), "-d", str(self.output / f"{name}.dag"),
                    "-p", "acceptance_" + name, "--disable_plugin_autoload"]
        self.processes[name] = subprocess.Popen(args, cwd=self.root, stdout=log, stderr=log)

    def kill(self, name):
        process = self.processes.pop(name)
        process.kill()
        process.wait(timeout=5)

    def state(self):
        path = self.output / "scene_state.json"
        return json.loads(path.read_text()) if path.exists() else {}

    def wait(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            dead = {n: p.returncode for n, p in self.processes.items() if p.poll() is not None}
            require(not dead, f"process exited: {dead}")
            if predicate():
                return
            time.sleep(0.02)
        raise RuntimeError(message)

    def scene(self, **command):
        self.command_id += 1
        atomic_json(self.output / "scene_command.json", {"id": self.command_id, **command})
        self.wait(lambda: self.state().get("command_id") == self.command_id, 3,
                  "scene command not acknowledged")

    def feedback(self, value, handle):
        self.feedbacks.append({"goal_id": str(handle.goal_id), "received": time.monotonic(),
                               "time": value.navigation_time.sec + value.navigation_time.nanosec * 1e-9,
                               "distance": value.distance_remaining,
                               "recoveries": value.number_of_recoveries})

    def send(self, x, y, yaw=0.0):
        goal = Action.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        handle = self.client.send_goal(goal, feedback_cb=self.feedback, timeout_sec=3)
        require(handle is not None, "action acceptance failed")
        return handle

    def result(self, handle, code, timeout=3):
        result = handle.get_result(timeout_sec=timeout)
        require(handle.is_succeeded() or handle.is_canceled() or handle.is_aborted(),
                "action result timed out")
        require(result.error_code == code, f"unexpected result {result}")
        return result

    def cancel(self, handle):
        require(handle.cancel(timeout_sec=3), "action cancellation failed")
        self.result(handle, NAVIGATION_CANCELLED)

    def reset(self):
        state = self.state()
        handle = self.send(*state["pose"])
        self.result(handle, OK)
        self.wait(lambda: max(abs(v) for v in self.state()["velocity"]) < 0.01,
                  3, "robot did not stop before reset")
        self.scene(pose=[0, 0, 0], obstacle=None, depth_paused=False)
        count = self.counts.get("observation", 0)
        self.wait(lambda: self.counts.get("observation", 0) >= count + 10, 3,
                  "observations did not settle after reset")

    def record(self, name, **metrics):
        self.results.append({"check": name, "passed": True, **metrics})
        atomic_json(self.output / "summary.json", {"checks": self.results,
                    "degradations": self.degradations, "feedbacks": self.feedbacks})
        print("PASS " + name + " " + json.dumps(metrics), flush=True)

    def startup(self):
        self.start("autosim")
        self.wait(lambda: self.counts.get("depth", 0) > 3, 30, "Habitat depth startup timeout")
        for name in ["navigation", "behavior", "safety", "guardian"]:
            self.start(name)
        self.wait(lambda: self.counts.get("observation", 0) > 10 and
                  self.mode == DEGRADATION_MODE_NORMAL, 15, "component startup timeout")
        require(self.client.wait_for_server(timeout_sec=5), "action server not ready")
        obs = self.latest["observation"]
        require(obs.validity & 15 == 15, f"RGB-D/odom/IMU validity incomplete: {obs.validity}")
        depth_stamp = obs.header.stamp.sec + obs.header.stamp.nanosec * 1e-9
        clock = self.latest["clock"]
        require(abs(clock.sec + clock.nanosec * 1e-9 - depth_stamp) < 0.3,
                "clock and depth stamps are not aligned")
        self.wait(lambda: self.state()["received_commands"] > 20 and
                  math.hypot(*self.state()["pose"][:2]) > 0.03, 18, "initial exploration did not move after scan")
        require(self.state()["collisions"] == 0, "initial exploration collided")
        trajectory = [json.loads(line) for line in (self.output / "trajectory.jsonl").read_text().splitlines()]
        scan_turn = 0.0
        for previous, current in zip(trajectory, trajectory[1:]):
            if math.hypot(*current["pose"][:2]) > .03:
                break
            delta = current["pose"][2] - previous["pose"][2]
            scan_turn += math.atan2(math.sin(delta), math.cos(delta))
        require(scan_turn >= 5.8, f"initial exploration moved before signed scan: {scan_turn} rad")
        self.record("sensor_actuator_initial_exploration", validity=obs.validity, scan_turn_rad=scan_turn,
                    pose=self.state()["pose"], received_commands=self.state()["received_commands"])
        import numpy as np
        depth = self.latest["depth"]
        np.save(self.output / "depth.npy", np.frombuffer(depth.data, dtype=np.float32)
                .reshape(depth.height, depth.width))
        if "rgb" in self.latest:
            from PIL import Image as PillowImage
            rgb = self.latest["rgb"]
            pixels = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
            if rgb.encoding == "bgr8":
                pixels = pixels[:, :, ::-1]
            PillowImage.fromarray(pixels).save(self.output / "rgb.png")

    def point(self):
        self.reset()
        start = time.monotonic()
        handle = self.send(2, 1)
        self.result(handle, OK, timeout=10)
        elapsed = time.monotonic() - start
        state = self.state()
        error = math.hypot(state["pose"][0] - 2, state["pose"][1] - 1)
        require(elapsed < 10 and error < 0.3, f"point goal failed: {elapsed}s {error}m")
        require(state["collisions"] == 0, "point goal collided")
        self.record("point", elapsed_sec=elapsed, error_m=error,
                    min_clearance_m=state["min_clearance"])

    def obstacle(self):
        self.reset()
        self.scene(obstacle=[1., -1., 1.3, 1.])
        handle = self.send(3, 0)
        self.wait(lambda: self.state()["pose"][0] > 0.1 and
                  abs(self.state()["velocity"][0]) < 0.01, 8, "front obstacle did not stop robot")
        time.sleep(1)
        state = self.state()
        require(state["collisions"] == 0, "front obstacle collided")
        require(abs(state["velocity"][0]) < 0.01, "front obstacle stop was not sustained")
        x = state["pose"][0]
        self.scene(obstacle=None)
        self.wait(lambda: self.state()["pose"][0] > x + 0.05, 3,
                  "obstacle removal did not resume motion")
        self.cancel(handle)
        self.record("obstacle_stop_resume", stop_x_m=x, min_clearance_m=state["min_clearance"])

    def clock(self):
        self.reset()
        handle = self.send(3, 0)
        self.wait(lambda: self.state()["velocity"][0] > 0.05, 3, "no motion before depth pause")
        start = time.monotonic()
        self.scene(depth_paused=True)
        self.wait(lambda: max(abs(v) for v in self.state()["velocity"]) < 0.01,
                  1, "depth pause did not stop plant")
        stopped = time.monotonic() - start
        count = self.counts["command"]
        zero_start = time.monotonic()
        time.sleep(0.3)
        samples = [c for c in self.command_samples if c["received"] >= zero_start]
        require(self.counts["command"] >= count + 8 and len(samples) >= 8 and
                all(max(abs(v) for v in c["velocity"]) < 1e-5 for c in samples),
                "zero TwistStamped stream missing during pause")
        self.scene(epoch=0, depth_paused=False)
        self.wait(lambda: self.state()["velocity"][0] > 0.05, 3, "epoch reset did not resume")
        require(self.state()["stamp"] < 3, "simulator epoch did not reset")
        require(self.latest["command"].header.frame_id == "base_link" and
                self.latest["command"].header.stamp.sec < 3,
                "command header did not follow depth epoch reset")
        self.cancel(handle)
        self.record("depth_pause_epoch_reset", stopped_sec=stopped)

    def preemption(self):
        self.reset()
        a = self.send(3, 0)
        time.sleep(0.2)
        start = len(self.feedbacks)
        b = self.send(2, 1)
        self.result(a, NAVIGATION_CANCELLED)
        self.wait(lambda: len(self.feedbacks) > start, 2, "new goal feedback missing")
        require(self.feedbacks[start]["time"] < 0.5, "feedback timer did not reset")
        self.goal_sequence += 1
        topic = GoalSpec(sequence=self.goal_sequence, goal_id="acceptance-topic", type=GoalSpec.TYPE_POSE)
        topic.pose.pose.position.x = 3
        topic.pose.pose.orientation.w = 1
        self.goal_writer.write(topic)
        self.result(b, NAVIGATION_CANCELLED)
        c = self.send(3, 1)
        time.sleep(0.2)
        self.cancel(c)
        self.wait(lambda: abs(self.latest["raw"].linear.x) < 1e-5 and
                  abs(self.latest["raw"].angular.z) < 1e-5, 1, "cancel raw command not zero")
        time.sleep(0.3)
        require(abs(self.latest["raw"].linear.x) < 1e-5, "cached topic goal revived")
        self.record("action_topic_preemption_cancel")

    def recovery(self):
        self.reset()
        self.scene(obstacle=[1., -1., 1.3, 1.])
        begin = time.monotonic()
        handle = self.send(3, 0)
        self.wait(lambda: self.state()["pose"][0] > 0.1 and
                  abs(self.state()["velocity"][0]) < 0.01, 8, "no blocked state")
        start = time.monotonic()
        feedback_start = len(self.feedbacks)
        self.wait(lambda: any(f["recoveries"] > 0 for f in self.feedbacks[feedback_start:]),
                  10, "8 second no-progress recovery missing from action feedback")
        elapsed = time.monotonic() - start
        require(6 < elapsed < 10, f"unexpected recovery delay: {elapsed}s")
        require(self.state()["collisions"] == 0, "blocked recovery collided")
        print("Recovery acknowledged; obstacle stays in place", flush=True)
        import numpy as np
        depth = self.latest["depth"]
        pixels = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
        np.save(self.output / "blocked_depth.npy", pixels)
        print("Blocked center depths " + str([float(pixels[depth.height//2, depth.width*(2*i+1)//10]) for i in range(5)]), flush=True)
        # Keep the real Habitat obstacle in place for the entire action.
        self.result(handle, OK, timeout=70)
        state = self.state()
        error = math.hypot(state["pose"][0] - 3, state["pose"][1])
        require(error < 0.3 and state["collisions"] == 0, "autonomous detour failed")
        trajectory = [json.loads(line) for line in (self.output / "trajectory.jsonl").read_text().splitlines()]
        trajectory = [row for row in trajectory if row["time"] >= begin]
        require(trajectory and max(abs(row["pose"][1]) for row in trajectory) > 1.3,
                "recovery did not travel around the obstacle")
        self.record("bt_autonomous_detour", delay_sec=elapsed,
                    elapsed_sec=time.monotonic() - begin, error_m=error,
                    min_clearance_m=state["min_clearance"],
                    recoveries=max(f["recoveries"] for f in self.feedbacks[feedback_start:]),
                    obstacle_removed=False,
                    stages=[t for t in self.status_transitions if t["received"] >= begin])

    def recovery_control(self):
        self.reset()
        self.scene(obstacle=[1., -1., 1.3, 1.])
        handle = self.send(3, 0)
        self.wait(lambda: self.latest.get("status") is not None and
                  self.latest["status"].recovery_state == RECOVERY_RUNNING, 15,
                  "recovery did not start before control tests")
        old = NavigationStatus()
        old.CopyFrom(self.latest["status"])
        self.wait(lambda: abs(self.latest["raw"].angular.z) > 0.1, 2,
                  "recovery did not turn")
        self.kill("behavior")
        stopped_at = time.monotonic()
        self.wait(lambda: max(abs(v) for v in
                  [self.latest["raw"].linear.x, self.latest["raw"].linear.y,
                   self.latest["raw"].angular.z]) < 1e-5, 0.8,
                  "behavior lease expiry did not suppress recovery commands")
        lease_stop = time.monotonic() - stopped_at
        # The 500 ms lease suppresses raw commands first; the Safety layer
        # then decelerates with the configured 0.8 acceleration / 2.0 jerk.
        self.wait(lambda: self.mode == DEGRADATION_MODE_REFLEX_ONLY and
                  max(abs(v) for v in self.state()["velocity"]) < 0.01,
                  max(0.01, 3.0 - lease_stop), "behavior loss did not stop the plant")
        plant_stop = time.monotonic() - stopped_at
        self.start("behavior")
        self.wait(lambda: self.mode == DEGRADATION_MODE_NORMAL and
                  abs(self.latest["raw"].angular.z) > 0.1, 5,
                  "behavior restart did not resume the maneuver")
        require(self.latest["status"].recovery_count == 1,
                "behavior restart consumed another recovery attempt")
        self.cancel(handle)
        self.wait(lambda: max(abs(v) for v in self.state()["velocity"]) < 0.01, 3,
                  "recovery cancel did not stop the plant")
        count = self.counts["status"]
        self.wait(lambda: self.counts["status"] > count and
                  not self.latest["status"].has_active_goal, 2,
                  "cancel did not clear the navigation session")
        require(self.latest["status"].recovery_count == 0, "cancel retained recovery count")
        # A stale FAIL_GOAL with a freshly re-stamped transport payload must
        # still be rejected after the goal epoch changes.
        new_handle = self.send(0.5, 1.0)
        self.wait(lambda: self.latest["status"].goal_epoch != old.goal_epoch, 2,
                  "new goal epoch was not published")
        request = BehaviorRequest(kind=BehaviorRequest.FAIL_GOAL,
            session_id=old.session_id, goal_epoch=old.goal_epoch,
            expected_recovery_count=0, reason="stale_test", steady_stamp_ns=time.monotonic_ns())
        self.behavior_writer.write(request)
        self.result(new_handle, OK, timeout=12)
        require(self.state()["collisions"] == 0, "recovery control scenario collided")
        self.record("bt_restart_cancel_stale_request", original_recoveries=old.recovery_count,
                    min_clearance_m=self.state()["min_clearance"],
                    raw_stop_sec=lease_stop, plant_stop_sec=plant_stop)

    def recovery_exhaustion(self):
        self.reset()
        self.kill("behavior")
        self.kill("navigation")
        path = self.output / "navigation.pb.txt"
        original = path.read_text()
        require("max_recovery_attempts: 3" in original, "recovery fixture configuration changed")
        path.write_text(original.replace("max_recovery_attempts: 3", "max_recovery_attempts: 1"))
        try:
            self.start("navigation")
            self.start("behavior")
            self.wait(lambda: self.mode == DEGRADATION_MODE_NORMAL, 5, "restart for budget check failed")
            self.scene(obstacle=[1., -3.9, 1.3, 4.9])
            handle = self.send(3, 0)
            begin = time.monotonic()
            self.result(handle, NAVIGATION_PATH_BLOCKED, timeout=70)
            self.wait(lambda: max(abs(v) for v in self.state()["velocity"]) < 0.01, 3,
                      "exhausted recovery did not stop")
            state = self.state()
            require(state["collisions"] == 0 and state["pose"][0] < 0.7,
                    "wall recovery penetrated the blocking wall")
            self.record("bt_recovery_exhausted", elapsed_sec=time.monotonic()-begin,
                        min_clearance_m=state["min_clearance"], attempts=1,
                        result_code=NAVIGATION_PATH_BLOCKED)
        finally:
            self.kill("behavior")
            self.kill("navigation")
            path.write_text(original)
            self.start("navigation")
            self.start("behavior")
            self.wait(lambda: self.mode == DEGRADATION_MODE_NORMAL, 5, "restore after budget check failed")

    def exploration(self):
        self.reset()
        self.scene(pose=[0, 0, 0], obstacle=[1.3, -0.6, 1.8, 0.6], map_mode="depth")
        self.wait(lambda: self.latest["observation"].HasField("map") and
                  self.latest["observation"].validity & 64, 4, "partial map did not reach Observation")
        initial_known = self.state()["known_cells"]
        self.goal_sequence += 1
        self.goal_writer.write(GoalSpec(type=GoalSpec.TYPE_EXPLORATION,
                                       goal_id=f"map-explore-{self.goal_sequence}"))
        start = time.monotonic()
        samples = []
        navigation_log = next(self.output.glob("navigation-*.log"))
        duration = float(os.environ.get("NAVIGATION_EXPLORATION_SECONDS", "180"))
        require(duration >= 120, "long exploration requires at least 120 seconds")
        while time.monotonic() - start < duration:
            self.wait(lambda: bool(self.state()), .5, "missing scene state")
            state = self.state()
            require(state["collisions"] == 0, "map exploration penetrated geometry")
            require(self.mode == DEGRADATION_MODE_NORMAL, "exploration degraded")
            require(self.latest["observation"].validity & 64, "map validity disappeared")
            transitions = [line.split("mode=", 1)[1].split()[0]
                           for line in navigation_log.read_text(errors="replace").splitlines()
                           if "[exploration] mode=" in line]
            state["exploration_mode"] = transitions[-1] if transitions else "unknown"
            samples.append(state)
            time.sleep(.1)
        distance = sum(math.dist(a["pose"][:2], b["pose"][:2]) for a, b in zip(samples, samples[1:]))
        xs = [s["pose"][0] for s in samples]; ys = [s["pose"][1] for s in samples]
        logs = "\n".join(p.read_text(errors="replace") for p in self.output.glob("navigation-*.log"))
        targets = logs.count("mode=map_target")
        history_sizes = [int(part.split()[0]) for part in logs.split("history=")[1:]]
        # Sliding 15 s windows detect persistent translation stalls while an
        # unexplored map remains. Deliberate completed-map waiting is tested below.
        stalls = []
        completed_map_waits = []
        for i in range(0, max(0, len(samples) - 150), 50):
            window = samples[i:i + 150]
            travel = sum(math.dist(a["pose"][:2], b["pose"][:2]) for a, b in zip(window, window[1:]))
            if travel < .1:
                metric = {"at": window[0]["time"] - start, "travel_m": travel}
                # Explicit scan/wait on a largely observed map is intentional.
                # A stationary active target remains a failure regardless of coverage.
                if all(s["exploration_mode"] in {"scan", "waiting_for_map"} for s in window) and \
                        min(s["known_cells"] for s in window) >= 8100:
                    completed_map_waits.append(metric)
                else:
                    stalls.append(metric)
        atomic_json(self.output / "exploration_samples.json", samples)
        final = self.state()
        require(distance > 4, f"exploration travel too small: {distance}")
        require(max(xs) - min(xs) > 2 or max(ys) - min(ys) > 2, "exploration stayed near spawn")
        require(final["known_cells"] > initial_known + 500, "map coverage did not increase")
        require(targets >= 3, f"frontier targets not renewed: {targets}")
        require(not stalls, f"persistent exploration translation stalls: {stalls}")
        require(not history_sizes or max(history_sizes) <= 64, "unbounded frontier history")
        self.record("map_frontier_long_exploration", duration_sec=time.monotonic() - start,
                    travel_m=distance, span_m=[max(xs)-min(xs), max(ys)-min(ys)],
                    known_cells=[initial_known, final["known_cells"]], map_frames=final["map_frames"],
                    map_targets=targets, max_history_entries=max(history_sizes, default=0),
                    min_clearance_m=final["min_clearance"], stalled_windows=stalls,
                    completed_map_wait_windows=completed_map_waits,
                    map_observation_validity=self.latest["observation"].validity)
        # A fully observed, finite room has no frontier. Scan in place once,
        # then remain stopped rather than wandering into known walls.
        self.reset()
        self.scene(pose=[0, 0, 0], obstacle=None, map_mode="full")
        self.goal_sequence += 1
        self.goal_writer.write(GoalSpec(type=GoalSpec.TYPE_EXPLORATION,
                                       goal_id=f"no-frontier-{self.goal_sequence}"))
        self.wait(lambda: abs(self.state()["velocity"][1]) > .1, 5, "no-frontier scan did not turn")
        self.wait(lambda: max(abs(v) for v in self.state()["velocity"]) < .01, 20,
                  "no-frontier scan never completed")
        time.sleep(2)
        state = self.state()
        require(math.hypot(*state["pose"][:2]) < .1 and
                max(abs(v) for v in state["velocity"]) < .01, "no-frontier map did not hold position")
        require(state["collisions"] == 0, "no-frontier scan collided")
        self.record("no_frontier_scan_then_wait", pose=state["pose"], velocity=state["velocity"])

    def guardian(self):
        self.reset()
        self.kill("behavior")
        self.wait(lambda: self.mode == DEGRADATION_MODE_REFLEX_ONLY, 2,
                  "behavior kill did not degrade")
        self.start("behavior")
        self.wait(lambda: self.mode == DEGRADATION_MODE_NORMAL, 5,
                  "behavior restart did not recover")
        self.send(3, 0)
        self.wait(lambda: self.state()["velocity"][0] > 0.1, 3, "no motion before navigation kill")
        start = time.monotonic()
        self.kill("navigation")
        self.wait(lambda: max(abs(v) for v in self.state()["velocity"]) < 0.01, 1,
                  "navigation kill left robot moving")
        nav_stop = time.monotonic() - start
        require(self.counts["command"] > 0 and
                max(abs(v) for v in self.command_samples[-1]["velocity"]) < 1e-5,
                "navigation kill did not produce a Safety zero command")
        self.wait(lambda: self.mode == DEGRADATION_MODE_REFLEX_ONLY, 2,
                  "navigation kill did not degrade")
        self.start("navigation")
        self.wait(lambda: self.mode == DEGRADATION_MODE_NORMAL, 5,
                  "navigation restart did not recover")
        self.wait(lambda: self.state()["velocity"][0] > 0.1, 4, "no motion after navigation restart")
        start = time.monotonic()
        self.kill("safety")
        self.wait(lambda: max(abs(v) for v in self.state()["velocity"]) < 0.01, 1.5,
                  "safety kill did not trigger plant watchdog")
        safety_stop = time.monotonic() - start
        self.wait(lambda: self.mode == DEGRADATION_MODE_SAFE_STOP, 2,
                  "safety kill did not classify SAFE_STOP")
        self.start("safety")
        self.wait(lambda: self.mode == DEGRADATION_MODE_NORMAL, 5,
                  "safety restart did not recover")
        require(all(d["from"] != d["to"] and d["reason"] for d in self.degradations),
                "invalid degradation event")
        self.record("process_kill_restart", navigation_stop_sec=nav_stop,
                    safety_stop_sec=safety_stop)

    def close(self):
        for name in list(self.processes):
            self.kill(name)
        autolink.shutdown()
        for log in self.files:
            log.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checks", default="point,obstacle,clock,preemption,recovery,recovery_control,recovery_exhaustion,guardian")
    args = parser.parse_args()
    run = Acceptance(args.root, args.build, args.output)
    try:
        run.startup()
        for check in args.checks.split(","):
            getattr(run, check)()
    except Exception as error:
        atomic_json(args.output / "failure.json", {"error": str(error), "state": run.state(),
                    "counts": run.counts, "mode": run.mode, "feedbacks": run.feedbacks[-5:],
                    "raw": [run.latest["raw"].linear.x, run.latest["raw"].angular.z]
                        if "raw" in run.latest else None,
                    "command": [run.latest["command"].twist.linear.x,
                                run.latest["command"].twist.angular.z]
                        if "command" in run.latest else None})
        raise
    finally:
        run.close()


if __name__ == "__main__":
    main()
