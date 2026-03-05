/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <QApplication>  // NOLINT: cpplint is unable to handle the include order here
#include <memory>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/visualizer_app.hpp"
#include "autonomy/tools/aviz/common/autolink_integration/autolink_client_abstraction.hpp"

int main(int argc, char** argv) {
  // Initialize autolink.
  ::autolink::Init(argv[0]);

  // Create the QApplication.
  QApplication qapp(argc, argv);

  // Create the visualizer app.
  autonomy::common::VisualizerApp vapp(std::make_unique<autonomy::common::AutolinkClientAbstraction>());
  vapp.setApp(&qapp);

  // Initialize the visualizer app.
  if (vapp.init(argc, argv)) {
    AINFO << "Initialized the visualizer app.";
    vapp.run();
  } else {
    AERROR << "Failed to initialize the visualizer app.";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}