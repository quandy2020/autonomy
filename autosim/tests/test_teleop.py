# Copyright 2026 The Openbot Authors (duyongquan)
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

from autosim.messages import Messages
from autosim.teleop import Teleop, constrain, make_simple_profile


class FakeWriter:
    def __init__(self):
        self.msgs = []

    def write(self, msg):
        self.msgs.append(msg)


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.writers = {}

    def create_writer(self, channel, dtype, qos_depth=1):
        writer = FakeWriter()
        self.writers[channel] = writer
        return writer


class FakeLink:
    def __init__(self):
        self.node = None
        self._shutdown = False

    def init(self, name):
        return None

    def shutdown(self):
        self._shutdown = True

    def is_shutdown(self):
        return self._shutdown

    def Node(self, name):
        self.node = FakeNode(name)
        return self.node


def test_encode_twist_stamped():
    message = Messages.encode_twist_stamped(0.2, -0.1, (1, 0), "base_link")
    assert message.header.frame_id == "base_link"
    assert abs(message.twist.linear.x - 0.2) < 1e-9
    assert abs(message.twist.angular.z - (-0.1)) < 1e-9


def test_constrain_and_profile():
    assert constrain(2.0, -1.0, 0.5) == 0.5
    assert abs(make_simple_profile(0.0, 0.1, 0.025) - 0.025) < 1e-9
    assert abs(make_simple_profile(0.1, 0.0, 0.025) - 0.075) < 1e-9


def test_teleop_apply_key_and_publish():
    link = FakeLink()
    teleop = Teleop(
        channel="/cmd_vel",
        max_linear=0.5,
        max_angular=1.0,
        linear_step=0.05,
        angular_step=0.1,
        link=link,
    )
    assert teleop.apply_key("w") is True
    assert abs(teleop.linear - 0.05) < 1e-9
    teleop.apply_key("w")
    assert abs(teleop.linear - 0.10) < 1e-9
    teleop.apply_key("a")
    assert abs(teleop.angular - 0.1) < 1e-9
    teleop.publish_twist()
    assert len(link.node.writers["/cmd_vel"].msgs) == 1
    assert teleop.apply_key(" ") is True
    assert teleop.linear == 0.0
    assert teleop.angular == 0.0
    assert teleop.apply_key("\x03") is False


def test_teleop_x_and_d_and_s_stop():
    link = FakeLink()
    teleop = Teleop(channel="/cmd_vel", linear_step=0.05, angular_step=0.1, link=link)
    teleop.apply_key("w")
    teleop.apply_key("x")
    assert abs(teleop.linear) < 1e-9
    teleop.apply_key("a")
    teleop.apply_key("d")
    assert abs(teleop.angular) < 1e-9
    teleop.apply_key("w")
    teleop.apply_key("s")
    assert teleop.linear == 0.0
    assert teleop.angular == 0.0


def test_teleop_latches_velocity():
    link = FakeLink()
    teleop = Teleop(channel="/cmd_vel", linear_step=0.05, link=link)
    teleop.apply_key("w")
    teleop.clock.tick(1.0)
    assert abs(teleop.linear - 0.05) < 1e-9
