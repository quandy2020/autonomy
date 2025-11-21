/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/common/message_util.hpp"

#include <vector>

#include "gtest/gtest.h"


namespace autonomy {
namespace common {

// TEST(MessageUtilTest, DumpMessage) {
//   auto simple_msg = std::make_shared<test::SimpleMessage>();
//   FillHeader("test", simple_msg.get());
//   EXPECT_TRUE(DumpMessage(simple_msg));
//   EXPECT_TRUE(cyber::common::PathExists(
//       "/tmp/autonomy.common.util.test.SimpleMessage/0.pb.txt"));
// }

// TEST(MessageUtilTest, MessageFingerprint) {
//   test::SimpleMessage msg;
//   const size_t fp0 = MessageFingerprint(msg);

//   msg.set_integer(1);
//   const size_t fp1 = MessageFingerprint(msg);
//   EXPECT_NE(fp0, fp1);

//   msg.set_integer(2);
//   EXPECT_NE(fp1, MessageFingerprint(msg));

//   msg.set_integer(1);
//   EXPECT_EQ(fp1, MessageFingerprint(msg));

//   msg.clear_integer();
//   EXPECT_EQ(fp0, MessageFingerprint(msg));
// }

// TEST(MessageUtilTest, get_desy_sec) {
//   auto simple_msg = std::make_shared<test::SimpleMessage>();
//   FillHeader("test", simple_msg.get());
//   EXPECT_GT(GetDelaySec(simple_msg), 0);
// }

}  // namespace common
}  // namespace autonomy
