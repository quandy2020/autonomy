/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file main.cpp
 * @brief autoloop-convert CLI.
 */
#include <iostream>
#include "cloud/conversion/cli/check_command.hpp"
#include "cloud/conversion/cli/convert_command.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: autoloop-convert convert|check ...\n";
        return 2;
    }
    const std::string cmd = argv[1];
    autoloop::common::Status st;
    if (cmd == "convert" && argc >= 4) {
        st = autoloop::cloud::RunConvert(argv[2], argv[3]);
    } else if (cmd == "check" && argc >= 3) {
        st = autoloop::cloud::RunCheck(argv[2]);
    } else {
        return 2;
    }
    if (!st.ok()) {
        std::cerr << st.message() << "\n";
        return 1;
    }
    return 0;
}
