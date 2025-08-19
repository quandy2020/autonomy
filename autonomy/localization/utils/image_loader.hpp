/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "autonomy/commsgs/sensor_msgs.hpp"

namespace autonomy {
namespace localization {
namespace utils {

/**
 * @brief  Read image from directory_path 
 * 
 * @param directory_path 
 * @param images 
 * @return true 
 * @return false 
 */
bool ReadImage(const std::string& directory_path, std::vector<cv::Mat>& images);

/**
 * @brief Read image from directory_path  convert `commsgs::sensor_msgs::Image`
 * 
 * @param directory_path 
 * @param images 
 * @return true 
 * @return false 
 */
bool ReadImage(const std::string& directory_path, std::vector<commsgs::sensor_msgs::Image>& images);


}  // namespace utils
}  // namespace localization
}  // namespace autonomy