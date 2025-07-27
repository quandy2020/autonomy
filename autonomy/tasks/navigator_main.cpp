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

#include <iostream>
#include <glog/logging.h>

#include "autonomy/common/version.hpp"

namespace autonomy {
namespace tasks {
namespace {

void Run()
{
    autonomy::common::ShowVersion();
    LOG(INFO) << "Autonomy open robot for everyone enjoy !!!";
}

} // namespace 
} // namespace tasks
} // namespace autonomy

int main(int argc, char **argv)
{
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;  

	google::InitGoogleLogging(argv[0]);
    autonomy::tasks::Run();
    google::ShutdownGoogleLogging();  
    return 0;
}