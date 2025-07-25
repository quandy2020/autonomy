
/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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


#include <string>

#include "gtest/gtest.h"

#include "eventpp/eventqueue.h"
#include "autonomy/map/map_server.hpp"

 
namespace autonomy {
namespace map {
namespace {

TEST(MapServer, Event)
{
	eventpp::EventQueue<std::string, void (const std::string &)> queue;

	int a = 1;
	int b = 5;

	queue.appendListener("event1", [&a](const std::string &) {
		a = 2;
	});

    queue.appendListener("event1", [&b](const std::string &) {
		b = 8;
	});

	EXPECT_NE(a, 2);
	EXPECT_NE(b, 8);

	queue.enqueue("event1");
	queue.process();
	EXPECT_EQ(a, 2);
	EXPECT_EQ(b, 8);
}

}  // namespace
}  // namespace map
}  // namespace autonomy