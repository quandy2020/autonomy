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
 * @file hash_util.hpp
 * @brief SHA-256 helpers. Autolink has no hash API; this is first-party.
 */

#include <cstdint>
#include <string>
#include <string_view>

namespace autoloop {
namespace common {

/**
 * @brief Hex-encoded SHA-256 of a byte buffer.
 * @param [in] data: Bytes.
 * @param [in] len: Length.
 * @return 64-char lowercase hex digest.
 */
std::string Sha256Hex(const uint8_t* data, size_t len);

/**
 * @brief Hex-encoded SHA-256 of a string view.
 * @param [in] bytes: Input.
 * @return 64-char lowercase hex digest.
 */
std::string Sha256Hex(std::string_view bytes);

/**
 * @brief SHA-256 of a file's contents.
 * @param [in] path: File path.
 * @param [out] digest: Hex digest on success.
 * @return True if the file was read.
 */
bool Sha256File(const std::string& path, std::string* digest);

}  // namespace common
}  // namespace autoloop
