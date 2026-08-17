from autosim.messages import Messages
from autosim.telop import Telop


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


def test_telop_apply_key_and_publish():
    link = FakeLink()
    telop = Telop(
        channel="/cmd_vel", max_linear=0.5, max_angular=1.0, key_timeout=0.3, link=link
    )
    assert telop.apply_key("w") is True
    assert telop.linear == 1.0
    telop.publish_twist()
    assert len(link.node.writers["/cmd_vel"].msgs) == 1
    msg = link.node.writers["/cmd_vel"].msgs[0]
    assert abs(msg.twist.linear.x - 0.5) < 1e-9
    assert telop.apply_key(" ") is True
    assert telop.linear == 0.0
    assert telop.apply_key("\x03") is False


def test_telop_decay_motion_on_timeout():
    link = FakeLink()
    telop = Telop(channel="/cmd_vel", key_timeout=0.2, link=link)
    telop.apply_key("w")
    assert telop.linear == 1.0
    telop.clock.tick(0.25)
    telop.decay_motion()
    assert telop.linear == 0.0
    assert telop.angular == 0.0
