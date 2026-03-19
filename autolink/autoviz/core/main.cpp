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

#include <signal.h>
#include <unistd.h>

namespace autoviz {
namespace {
 
void Run() {
    // register signal handler with lambda
    signal(SIGINT, [](int) { exit(0); });
    signal(SIGTERM, [](int) { exit(0); });
}

}  // namespace
}  // namespace autoviz
 
 int main(int argc, char** argv) {
   autolink::Init(argv[0]);
   google::ParseCommandLineFlags(&argc, &argv, true);
   if (autonomy::common::FLAGS_verbose) {
     autonomy::common::ShowVersion();
     return 0;
   }
   autoviz::Run();
   google::ShutDownCommandLineFlags();
   return 0;
 }
 