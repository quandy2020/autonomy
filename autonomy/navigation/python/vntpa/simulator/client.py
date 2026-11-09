# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""autosim bridge: autolink depth/odom/goal -> VNT-PA -> /cmd_vel."""

from __future__ import annotations

import argparse
import time
from collections import deque
from typing import Deque, Optional

import numpy as np

from vntpa.simulator.action import (
    Action,
    primitive,
    twist,
)
from vntpa.utils.config import AutosimConfig, DeployConfig, PlannerConfig
from vntpa.data.dataset import ContextMemory
from vntpa.models.encoder import OnnxDefmEncoder
from vntpa.models.vntpa import OnnxPlanner
from vntpa.simulator.observation import (
    depth,
    make_twist_stamped,
    odom,
)


class AutosimApp:
    """Single-threaded autolink node; encode at depth rate, publish at timer rate."""

    def __init__(
        self, deploy: DeployConfig, autosim: AutosimConfig, *, goal_type: Optional[int] = None
    ) -> None:
        import autolink
        from automsgs.msgs.geometry_msgs import twist_stamped_pb2
        from automsgs.msgs.nav_msgs import odometry_pb2
        from automsgs.msgs.sensor_msgs import image_pb2
        from navigation.proto import navigation_pb2

        self.deploy, self.cfg = deploy, autosim
        self.autolink_ = autolink
        self.twist_type_ = twist_stamped_pb2.TwistStamped
        self.navigation_pb2_ = navigation_pb2
        self.defm = OnnxDefmEncoder(deploy.defm_onnx, deploy.providers)
        self.planner = OnnxPlanner(deploy.planner_onnx, deploy.planner, deploy.providers)
        self.memory = ContextMemory(
            max_keyframes=autosim.max_keyframes,
            min_distance=autosim.min_keyframe_distance,
            min_interval_s=autosim.min_keyframe_interval_s,
        )
        self.goal_type = autosim.goal_type if goal_type is None else goal_type

        autolink.init(autosim.node_name)
        self.node = autolink.Node(autosim.node_name)
        self.cmd = self.node.create_writer(autosim.cmd_topic, self.twist_type_)
        self.node.create_reader(autosim.depth_topic, self.handle_depth, data_type=image_pb2.Image)
        self.node.create_reader(
            autosim.odom_topic, self.handle_odom, data_type=odometry_pb2.Odometry
        )
        self.node.create_reader(
            autosim.goal_topic, self.handle_goal, data_type=navigation_pb2.GoalSpec
        )
        self.pose_: Optional[np.ndarray] = None
        self.goal_: Optional[np.ndarray] = None
        self.depth_: Optional[np.ndarray] = None
        self.depth_stamp_ = 0.0
        self.encoded_stamp_ = -1.0
        self.last_plan_ = -1.0
        self.last_encode_ = -1.0
        self.queue_: Deque[Action] = deque()
        self.next_action_ = 0.0
        self.last_twist_ = twist(primitive(Action.STOP))

    @staticmethod
    def now() -> float:
        return time.monotonic()

    def handle_depth(self, message) -> None:
        depth_data = depth(message)
        if depth_data is not None:
            self.depth_ = depth_data
            self.depth_stamp_ = self.now()

    def handle_odom(self, message) -> None:
        self.pose_ = odom(message)

    def handle_goal(self, message) -> None:
        if not message.HasField("pose"):
            return
        p = message.pose.pose.position
        self.goal_ = np.array([p.x, p.y], dtype=np.float32)
        self.queue_.clear()

    def publish(self, twist) -> None:
        message = make_twist_stamped(
            self.twist_type_, self.autolink_.Time.now().to_nsec(), self.cfg.base_frame, twist
        )
        self.cmd.write(message)
        self.last_twist_ = twist

    def encode(self) -> None:
        if self.depth_ is None or self.pose_ is None:
            return
        now = self.now()
        interval = 1.0 / max(self.deploy.planner.decision_rate_hz, 1e-3)
        if now - self.last_encode_ < interval or self.depth_stamp_ <= self.encoded_stamp_:
            return
        encoded = self.defm.encode(self.depth_)
        stamp = time.time()
        self.last_encode_ = now
        self.memory.add(encoded.tokens, self.pose_, stamp, encoded.pooled)
        self.encoded_stamp_ = self.depth_stamp_
        if self.goal_ is None or self.queue_:
            return
        result = self.planner.plan(self.pose_, self.goal_, self.memory)
        self.last_plan_ = now
        self.queue_ = deque(result.actions[: self.deploy.planner.execute_horizon])
        self.next_action_ = 0.0

    def tick(self) -> None:
        self.encode()
        now = self.now()
        if self.queue_ and now >= self.next_action_:
            action = self.queue_.popleft()
            item = primitive(
                action, self.deploy.planner.forward_step, self.deploy.planner.turn_step_rad
            )
            self.last_twist_ = twist(item, self.deploy.planner.action_dt_s)
            self.next_action_ = now + self.deploy.planner.action_dt_s
        self.publish(self.last_twist_)

    def run(self) -> None:
        rate = self.autolink_.Rate(self.cfg.timer_hz)
        try:
            while self.autolink_.ok():
                self.tick()
                rate.sleep()
        finally:
            self.stop()

    def stop(self) -> None:
        self.publish(twist(primitive(Action.STOP)))
        self.autolink_.shutdown()


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
        planner_onnx=args.planner,
        defm_onnx=args.defm,
        planner=PlannerConfig(context_size=args.context_size, providers=providers),
        providers=providers,
    )
    app = AutosimApp(deploy, AutosimConfig(goal_topic=args.goal_topic))
    app.run()


if __name__ == "__main__":
    main()
