/*
 * Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file main.cpp
 * @brief Process entry for the ballbot example.
 *
 * Loads the packaged URDF, solves the translate-and-balance problem, and
 * logs the open-loop trajectory with glog.
 */

#include "ballbot.hpp"

#include <glog/logging.h>

/**
 * @brief Process entry.
 *
 * INFO and ERROR records are written to stderr so the trajectory stays on
 * the terminal.
 *
 * @return Exit code from Ballbot::Run. 0 when the target is reached, 1 otherwise.
 */
int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  ballbot::Ballbot robot;
  return robot.Run();
}
