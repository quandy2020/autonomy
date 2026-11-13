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
 * @file version_tuple.cpp
 * @brief Assemble and admit the policy/env/git/seed version quadruple.
 */

#include "autonomy/system/recorder/version_tuple.hpp"

#include "autolink/common/environment.hpp"
#include "autonomy/common/version.hpp"

namespace autonomy {
namespace system {
namespace recorder {
namespace {

std::string EnvOr(const char* key, const std::string& fallback) {
    const std::string env = autolink::common::GetEnv(key);
    return env.empty() ? fallback : env;
}

std::string ParseGitSha(const std::string& configured) {
    if (!configured.empty()) {
        return configured;
    }
    const std::string from_env = EnvOr("AUTONOMY_GIT_SHA", "");
    if (!from_env.empty()) {
        return from_env;
    }
    const std::string raw = common::GetGitCommitID();
    const std::string marker = "ID: ";
    const auto pos = raw.find(marker);
    if (pos == std::string::npos) {
        return {};
    }
    auto start = pos + marker.size();
    const auto on = raw.find(" on ", start);
    std::string sha = on == std::string::npos ? raw.substr(start)
                                             : raw.substr(start, on - start);
    if (sha == "unknown" || sha.empty()) {
        return {};
    }
    return sha;
}

bool Missing(const std::string& value) {
    return value.empty() || value == "unknown";
}

}  // namespace

bool IsAdmitted(const proto::VersionTuple& tuple) {
    return !Missing(tuple.policy_version()) && !Missing(tuple.env_version()) &&
           !Missing(tuple.git_sha()) && !Missing(tuple.seed());
}

VersionCheck MakeVersionTuple(const proto::RecorderOptions& options) {
    VersionCheck check;
    auto* tuple = &check.tuple;
    tuple->set_policy_version(EnvOr(
        "AUTONOMY_POLICY_VERSION",
        EnvOr("AUTONOMY_SOFTWARE_VERSION", options.policy_version())));
    tuple->set_env_version(
        EnvOr("AUTONOMY_ENV_VERSION", options.env_version()));
    tuple->set_seed(EnvOr("AUTONOMY_SEED", options.seed()));
    tuple->set_git_sha(ParseGitSha(options.git_sha()));

    if (Missing(tuple->policy_version())) {
        check.missing.emplace_back("policy_version");
    }
    if (Missing(tuple->env_version())) {
        check.missing.emplace_back("env_version");
    }
    if (Missing(tuple->git_sha())) {
        check.missing.emplace_back("git_sha");
    }
    if (Missing(tuple->seed())) {
        check.missing.emplace_back("seed");
    }
    check.admitted = check.missing.empty();
    return check;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
