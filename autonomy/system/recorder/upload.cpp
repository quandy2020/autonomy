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

#include "autonomy/system/recorder/upload.hpp"

#include <filesystem>
#include <fstream>
#include <system_error>

#include "nlohmann/json.hpp"

namespace autonomy {
namespace system {
namespace recorder {
namespace fs = std::filesystem;

namespace {

void CopyIfExists(const fs::path& src, const fs::path& dest, int* copied,
                  int* failed) {
    std::error_code ec;
    if (!fs::exists(src, ec)) {
        return;
    }
    fs::copy_file(src, dest, fs::copy_options::overwrite_existing, ec);
    if (ec) {
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
    std::error_code ec;
    if (!fs::exists(src_dir, ec) || !fs::is_directory(src_dir, ec)) {
        out->reason = "source missing";
        return false;
    }
    fs::create_directories(dest_dir, ec);
    if (ec) {
        out->reason = "dest create failed";
        return false;
    }
    for (const auto& entry : fs::directory_iterator(src_dir, ec)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto path = entry.path();
        if (path.extension() != ".json" ||
            path.stem().extension() != ".manifest") {
            continue;
        }
        std::ifstream in(path);
        if (!in) {
            ++out->failed;
            continue;
        }
        auto j = nlohmann::json::parse(in, nullptr, false);
        if (j.is_discarded()) {
            ++out->failed;
            continue;
        }
        if (!j.value("upload", false) || !j.value("admitted", false)) {
            ++out->skipped;
            continue;
        }
        std::string stem = j.value("episode_id", path.stem().stem().string());
        if (stem.empty()) {
            stem = path.stem().stem().string();
        }
        const fs::path src = path.parent_path();
        const fs::path dest = fs::path(dest_dir);
        CopyIfExists(src / (stem + ".mcap"), dest / (stem + ".mcap"),
                     &out->copied, &out->failed);
        CopyIfExists(src / (stem + ".sidecar.pb"),
                     dest / (stem + ".sidecar.pb"), &out->copied, &out->failed);
        CopyIfExists(src / (stem + ".sidecar.json"),
                     dest / (stem + ".sidecar.json"), &out->copied,
                     &out->failed);
        CopyIfExists(src / (stem + ".manifest.json"),
                     dest / (stem + ".manifest.json"), &out->copied,
                     &out->failed);
        CopyIfExists(src / (stem + ".overlay.json"),
                     dest / (stem + ".overlay.json"), &out->copied,
                     &out->failed);
        CopyIfExists(src / (stem + ".observations.jsonl"),
                     dest / (stem + ".observations.jsonl"), &out->copied,
                     &out->failed);
    }
    out->reason = "ok";
    return out->failed == 0;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
