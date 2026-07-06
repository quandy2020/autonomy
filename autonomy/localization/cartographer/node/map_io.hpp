/*
 * Copyright 2017 The Cartographer Authors
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

#include "Eigen/Core"
#include "autonomy/localization/cartographer/io/file_writer.hpp"
#include "autonomy/localization/cartographer/io/image.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

void WritePgm(const ::cartographer::io::Image& image, double resolution,
              ::cartographer::io::FileWriter* file_writer);

void WriteYaml(double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::cartographer::io::FileWriter* file_writer);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
