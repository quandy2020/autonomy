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
 * @file upload.cpp
 * @brief Copy episode dirs whose manifest.upload is true.
 */

#include "robot/recorder/upload.hpp"

#include "nlohmann/json.hpp"

#include "autolink/common/file.hpp"

namespace autoloop {
namespace robot {
namespace recorder {
namespace {

void CopyIfExists(const std::string& src, const std::string& dest, int* copied,
                  int* failed) {
    if (!autolink::common::PathExists(src)) {
        return;
    }
    if (!autolink::common::CopyFile(src, dest)) {
        ++*failed;
        return;
    }
    ++*copied;
}

}  // namespace

bool UploadEpisodes(const std::string& src_dir, const std::string& dest_dir,
                    UploadReport* report) {
    UploadReport local;
    UploadReport* out = report != nullptr ? report : &local;
    *out = UploadReport{};
    if (src_dir.empty() || dest_dir.empty()) {
        out->reason = "empty path";
        return false;
    }
    if (!autolink::common::DirectoryExists(src_dir)) {
        out->reason = "source missing";
        return false;
    }
    if (!autolink::common::EnsureDirectory(dest_dir)) {
        out->reason = "dest create failed";
        return false;
    }
    constexpr char kSuffix[] = ".manifest.json";
    for (const auto& name :
         autolink::common::ListSubPaths(src_dir, DT_REG)) {
        if (name.size() < sizeof(kSuffix) - 1 ||
            name.compare(name.size() - (sizeof(kSuffix) - 1),
                         sizeof(kSuffix) - 1, kSuffix) != 0) {
            continue;
        }
        const std::string path = src_dir + "/" + name;
        std::string content;
        if (!autolink::common::GetContent(path, &content)) {
            ++out->failed;
            continue;
        }
        auto j = nlohmann::json::parse(content, nullptr, false);
        if (j.is_discarded()) {
            ++out->failed;
            continue;
        }
        if (!j.value("upload", false) || !j.value("admitted", false)) {
            ++out->skipped;
            continue;
        }
        std::string stem = j.value("episode_id", "");
        if (stem.empty()) {
            stem = name.substr(0, name.size() - (sizeof(kSuffix) - 1));
        }
        auto Copy = [&](const std::string& suffix) {
            CopyIfExists(src_dir + "/" + stem + suffix,
                         dest_dir + "/" + stem + suffix, &out->copied,
                         &out->failed);
        };
        Copy(".mcap");
        Copy(".sidecar.pb");
        Copy(".sidecar.json");
        Copy(".manifest.json");
        Copy(".overlay.json");
        Copy(".observations.jsonl");
    }
    out->reason = "ok";
    return out->failed == 0;
}

}  // namespace recorder
}  // namespace robot
}  // namespace autoloop
