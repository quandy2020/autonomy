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

#pragma once
/**
 * @file hash_verifier.hpp
 * @brief Compare sidecar content_sha256 with file digest.
 */
#include <string>
#include "common/proto/version_tuple.pb.h"

namespace autoloop {
namespace cloud {

bool VerifySidecarHash(const common::EpisodeSidecar& sc, const std::string& mcap_path);

}  // namespace cloud
}  // namespace autoloop
