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
 * @file version_stamp.cpp
 * @brief Env-backed quadruple (AUTOLOOP_* then AUTONOMY_*).
 */

#include "robot/data_agent/version_stamp/version_stamp.hpp"

#include "common/base/path_util.hpp"

namespace autoloop {
namespace robot {

bool IsAdmitted(const common::VersionTuple& tuple) {
    return !tuple.policy_version().empty() && !tuple.env_version().empty() &&
           !tuple.git_sha().empty() && !tuple.seed().empty();
}

VersionCheck MakeVersionTuple() {
    VersionCheck c;
    auto field = [](const char* a, const char* b, const char* fb) {
        std::string v = common::GetEnv(a);
        if (v.empty()) {
            v = common::GetEnv(b, fb);
        }
        return v;
    };
    c.tuple.set_policy_version(
        field("AUTOLOOP_POLICY_VERSION", "AUTONOMY_SOFTWARE_VERSION", ""));
    c.tuple.set_env_version(field("AUTOLOOP_ENV_VERSION", "AUTONOMY_ENV_VERSION", ""));
#ifdef AUTOLOOP_GIT_SHA
    const char* compiled = AUTOLOOP_GIT_SHA;
#else
    const char* compiled = "";
#endif
    std::string sha = field("AUTOLOOP_GIT_SHA", "AUTONOMY_GIT_SHA", compiled);
    c.tuple.set_git_sha(sha);
    c.tuple.set_seed(field("AUTOLOOP_SEED", "AUTONOMY_SEED", ""));
    if (c.tuple.policy_version().empty()) {
        c.missing.emplace_back("policy_version");
    }
    if (c.tuple.env_version().empty()) {
        c.missing.emplace_back("env_version");
    }
    if (c.tuple.git_sha().empty()) {
        c.missing.emplace_back("git_sha");
    }
    if (c.tuple.seed().empty()) {
        c.missing.emplace_back("seed");
    }
    c.admitted = c.missing.empty();
    return c;
}

}  // namespace robot
}  // namespace autoloop
