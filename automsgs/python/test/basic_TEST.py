# Copyright 2025 The Openbot Authors (duyongquan)
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

from automsgs.msgs.geometry_msgs.vector3_pb2 import Vector3

import unittest


class BasicTest(unittest.TestCase):

    def test_serialization(self):
        msg = Vector3()
        msg.x = 1.0
        msg.y = 2.0
        msg.z = 3.0

        serialized_msg = msg.SerializeToString()
        self.assertGreater(len(serialized_msg), 0)

        msg_from_serialized = Vector3()
        self.assertNotEqual(msg_from_serialized, msg)
        msg_from_serialized.ParseFromString(serialized_msg)
        self.assertEqual(msg_from_serialized.x, msg.x)
        self.assertEqual(msg_from_serialized.y, msg.y)
        self.assertEqual(msg_from_serialized.z, msg.z)


if __name__ == '__main__':
    unittest.main()
