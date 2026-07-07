/*
 * Copyright 2016 The Cartographer Authors
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

#pragma once

#include <string>
#include <vector>

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kNavSatFixTopic[] = "fix";
constexpr char kLandmarkTopic[] = "landmark";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kTrackedPoseTopic[] = "tracked_pose";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
constexpr char kWriteStateServiceName[] = "write_state";
constexpr char kTfTopic[] = "tf";
constexpr double kDefaultOccupancyGridResolution = 0.05;
constexpr double kDefaultOccupancyGridPublishPeriodSec = 1.0;

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   int num_topics);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
