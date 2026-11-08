"""autosim bridge: autolink depth/odom/goal -> VNT-PA -> /cmd_vel."""

from __future__ import annotations

import argparse
import time
from collections import deque
from typing import Deque, Optional

import numpy as np

from .action_adapter import Action, primitive, primitive_to_twist
from ...utils.config import AutosimConfig, DeployConfig, PlannerConfig
from ...data.dataset import ContextMemory
from ...models.encoder import OnnxDefmEncoder
from ...models.vntpa import OnnxPlanner
from .observation_adapter import depth_array, make_twist_stamped, pose_from_odom


class AutosimApp:
    """Single-threaded autolink node; encode at depth rate, publish at timer rate."""

    def __init__(self, deploy: DeployConfig, autosim: AutosimConfig,
                 *, goal_type: Optional[int] = None) -> None:
        import autolink
        from automsgs.msgs.geometry_msgs import twist_stamped_pb2
        from automsgs.msgs.nav_msgs import odometry_pb2
        from automsgs.msgs.sensor_msgs import image_pb2
        from navigation.proto import navigation_pb2

        self.deploy, self.cfg = deploy, autosim
        self._autolink = autolink
        self._twist_type = twist_stamped_pb2.TwistStamped
        self._navigation_pb2 = navigation_pb2
        self.defm = OnnxDefmEncoder(deploy.defm_onnx, deploy.providers)
        self.planner = OnnxPlanner(deploy.planner_onnx, deploy.planner, deploy.providers)
        self.memory = ContextMemory(
            max_keyframes=autosim.max_keyframes,
            min_distance=autosim.min_keyframe_distance,
            min_interval_s=autosim.min_keyframe_interval_s)
        self.goal_type = autosim.goal_type if goal_type is None else goal_type

        autolink.init(autosim.node_name)
        self.node = autolink.Node(autosim.node_name)
        self.cmd = self.node.create_writer(autosim.cmd_topic, self._twist_type)
        self.node.create_reader(autosim.depth_topic, self._on_depth,
                                data_type=image_pb2.Image)
        self.node.create_reader(autosim.odom_topic, self._on_odom,
                                data_type=odometry_pb2.Odometry)
        self.node.create_reader(autosim.goal_topic, self._on_goal,
                                data_type=navigation_pb2.GoalSpec)
        self._pose: Optional[np.ndarray] = None
        self._goal: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._depth_stamp = 0.0
        self._encoded_stamp = -1.0
        self._last_plan = -1.0
        self._last_encode = -1.0
        self._queue: Deque[Action] = deque()
        self._next_action = 0.0
        self._last_twist = primitive_to_twist(primitive(Action.STOP))

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_depth(self, message) -> None:
        depth = depth_array(message)
        if depth is not None:
            self._depth = depth
            self._depth_stamp = self._now()

    def _on_odom(self, message) -> None:
        self._pose = pose_from_odom(message)

    def _on_goal(self, message) -> None:
        if not message.HasField("pose"):
            return
        p = message.pose.pose.position
        self._goal = np.array([p.x, p.y], dtype=np.float32)
        self._queue.clear()

    def _publish(self, twist) -> None:
        message = make_twist_stamped(
            self._twist_type, self._autolink.Time.now().to_nsec(),
            self.cfg.base_frame, twist)
        self.cmd.write(message)
        self._last_twist = twist

    def _encode(self) -> None:
        if self._depth is None or self._pose is None:
            return
        now = self._now()
        interval = 1.0 / max(self.deploy.planner.decision_rate_hz, 1e-3)
        if now - self._last_encode < interval or self._depth_stamp <= self._encoded_stamp:
            return
        encoded = self.defm.encode(self._depth)
        stamp = time.time()
        self._last_encode = now
        self.memory.add(encoded.tokens, self._pose, stamp, encoded.pooled)
        self._encoded_stamp = self._depth_stamp
        if self._goal is None or self._queue:
            return
        result = self.planner.plan(self._pose, self._goal, self.memory)
        self._last_plan = now
        self._queue = deque(result.actions[: self.deploy.planner.execute_horizon])
        self._next_action = 0.0

    def tick(self) -> None:
        self._encode()
        now = self._now()
        if self._queue and now >= self._next_action:
            action = self._queue.popleft()
            item = primitive(action, self.deploy.planner.forward_step,
                             self.deploy.planner.turn_step_rad)
            self._last_twist = primitive_to_twist(item, self.deploy.planner.action_dt_s)
            self._next_action = now + self.deploy.planner.action_dt_s
        self._publish(self._last_twist)

    def run(self) -> None:
        rate = self._autolink.Rate(self.cfg.timer_hz)
        try:
            while self._autolink.ok():
                self.tick()
                rate.sleep()
        finally:
            self.stop()

    def stop(self) -> None:
        self._publish(primitive_to_twist(primitive(Action.STOP)))
        self._autolink.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", default="conf/models/vntpa.onnx")
    parser.add_argument("--defm", default="conf/models/defm_vit_s14.onnx")
    parser.add_argument("--goal-topic", default="/navigation/goal")
    parser.add_argument("--context-size", type=int, default=64)
    parser.add_argument("--provider", action="append", default=[])
    args = parser.parse_args()
    providers = tuple(args.provider or ("CPUExecutionProvider",))
    deploy = DeployConfig(
        planner_onnx=args.planner, defm_onnx=args.defm,
        planner=PlannerConfig(context_size=args.context_size, providers=providers),
        providers=providers)
    app = AutosimApp(deploy, AutosimConfig(goal_topic=args.goal_topic))
    app.run()


if __name__ == "__main__":
    main()
