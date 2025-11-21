/**
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

#include "autolink/logger/log_file_object.hpp"

#include <string>

#include "gtest/gtest.h"

#include "glog/logging.h"

// TODO: Uncomment when autolink and time modules are implemented
// #include "autolink/autolink.hpp"
// #include "autolink/time/time.hpp"

namespace autolink {
namespace logger {

TEST(LogFileObjectTest, init_and_write) {
    std::string basename = "logfile";
    LogFileObject logfileobject(google::INFO, basename.c_str());
    logfileobject.SetBasename("base");
    time_t timep;
    time(&timep);
    std::string message = "autolink logger test";
    logfileobject.Write(false, timep, message.c_str(), 20);
    logfileobject.SetExtension("unittest");
    logfileobject.Flush();
}

}  // namespace logger
}  // namespace autolink
