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

#include "autonomy/system/options.hpp"

#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "autolink/common/file.hpp"

namespace autonomy {
namespace system {

proto::AutonomyOptions CreateOptions(const std::string& conf_file) {
    proto::AutonomyOptions options;
    const std::string file =
        conf_file.empty() ? std::string("autonomy.pb.txt") : conf_file;

    if (autolink::common::PathIsAbsolute(file) ||
        file.find('/') != std::string::npos) {
        std::string path;
        if (autolink::common::GetFilePathWithEnv(file, "AUTONOMY_CONF_PATH",
                                                 &path) ||
            autolink::common::PathExists(file)) {
            if (path.empty()) {
                path = file;
            }
            CHECK(autolink::common::GetProtoFromFile(path, &options))
                << "Failed to load AutonomyOptions from " << path;
            return options;
        }
    }

    CHECK(common::LoadModuleConf("system", file, &options))
        << "Failed to load system conf: " << file
        << " (set AUTONOMY_PATH or AUTONOMY_CONF_PATH)";
    return options;
}

}  // namespace system
}  // namespace autonomy
