# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.bridge import AutolinkBridge
from benchmark.core.types import Twist2D


def test_null_transport_publish() -> None:
    bridge = AutolinkBridge(node_name="test_null")
    ok = bridge.connect()
    # May or may not connect depending on env; publish must not raise.
    bridge.publish_twist(Twist2D(linear=0.2, angular=0.1))
    assert len(bridge._null_log) == 1
    bridge.shutdown()
    assert ok in (True, False)
