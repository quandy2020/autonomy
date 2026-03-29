#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan). All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
# -*- coding: utf-8 -*-
"""Module for example of talker."""

import time

from _bootstrap_autolink import setup_autolink_pythonpath

setup_autolink_pythonpath()

try:
    from autolink_py3 import autolink
    from autolink_py3 import autolink_time
except ModuleNotFoundError:
    # Backward compatibility for legacy package layout.
    from autolink.python.autolink_py3 import autolink  # type: ignore
    from autolink.python.autolink_py3 import autolink_time  # type: ignore
from autolink.proto.unit_test_pb2 import Chatter


def test_talker_class():
    """
    Test talker.
    """
    test_node = autolink.Node("node_name1")
    g_count = 1

    writer = test_node.create_writer("channel/chatter", Chatter, 6)
    # Use Rate to maintain accurate 1 Hz publishing frequency
    rate = autolink_time.Rate(1.0)  # 1 Hz = 1 message per second
    
    print("Starting to publish at 1 Hz (1 message per second)")
    while not autolink.is_shutdown():
        msg = Chatter()
        msg.timestamp = autolink_time.Time.now().to_nsec()
        msg.lidar_timestamp = msg.timestamp
        msg.seq = g_count
        msg.content = b"I am python talker."  # bytes type
        current_time = time.time()
        print("=" * 80)
        print("[%.3f] write msg -> seq: %d, content: %s" % (current_time, msg.seq, msg.content))
        writer.write(msg)
        g_count = g_count + 1
        rate.sleep()  # Sleep to maintain 1 Hz rate


if __name__ == '__main__':
    autolink.init("talker_sample")
    test_talker_class()
    autolink.shutdown()
