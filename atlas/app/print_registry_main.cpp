/*
 * Copyright 2026 The Openbot Authors
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

#include <iostream>
#include <string>

#include "slam_system_builder.hpp"

int main(int argc, char** argv) {
  for (int index = 1; index < argc; ++index) {
    const std::string flag = argv[index];
    if (flag == "--help" || flag == "-h") {
      std::cerr << "usage: " << argv[0] << "\n";
      return 0;
    }
    std::cerr << "usage: " << argv[0] << "\n";
    return 2;
  }
  for (const std::string& type :
       atlas::app::SlamSystemBuilder::BuiltinTypes()) {
    std::cout << type << "\n";
  }
  return 0;
}
