/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#pragma once

#include <string>

namespace autolink {
namespace transport {

// Opt-in RTPS/DDS Security settings from environment.
// Env:
//   AUTOLINK_RTPS_SECURITY=1 — enable (any other value / unset = disabled)
//   AUTOLINK_RTPS_SECURITY_DIR — certificate directory (trimmed)
struct SecurityConfig {
    bool enabled = false;
    std::string dir;

    static SecurityConfig FromEnv();

    // Returns false and sets *err when enabled but dir/files are invalid.
    bool Validate(std::string* err) const;

    static const char* kIdentityCa;     // "identity_ca.crt"
    static const char* kPermissionsCa;  // "permissions_ca.crt"
    static const char* kCert;           // "cert.pem"
    static const char* kKey;            // "key.pem"
    static const char* kGovernance;     // "governance.smime"
    static const char* kPermissions;    // "permissions.smime"

    std::string Path(const char* filename) const;
};

}  // namespace transport
}  // namespace autolink
