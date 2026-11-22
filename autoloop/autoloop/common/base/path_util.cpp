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
 * @file path_util.cpp
 * @brief Forwards to autolink::common file/env helpers.
 */

#include "common/base/path_util.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#if defined(AUTOLOOP_WITH_AUTOLINK)
#include "autolink/common/environment.hpp"
#include "autolink/common/file.hpp"
#endif

namespace autoloop {
namespace common {

std::string GetEnv(const std::string& name, const std::string& fallback) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::GetEnv(name, fallback);
#else
    const char* v = std::getenv(name.c_str());
    if (v == nullptr || v[0] == '\0') {
        return fallback;
    }
    return std::string(v);
#endif
}

bool PathExists(const std::string& path) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::PathExists(path);
#else
    std::ifstream in(path);
    return in.good();
#endif
}

bool EnsureDirectory(const std::string& directory_path) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::EnsureDirectory(directory_path);
#else
    if (directory_path.empty()) {
        return false;
    }
    std::string cmd = "mkdir -p \"" + directory_path + "\"";
    return std::system(cmd.c_str()) == 0;
#endif
}

bool CopyFile(const std::string& from, const std::string& to) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::CopyFile(from, to);
#else
    std::ifstream in(from, std::ios::binary);
    if (!in) {
        return false;
    }
    std::ofstream out(to, std::ios::binary);
    out << in.rdbuf();
    return static_cast<bool>(out);
#endif
}

bool ReadFile(const std::string& path, std::string* content) {
    if (content == nullptr) {
        return false;
    }
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::GetContent(path, content);
#else
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    *content = ss.str();
    return true;
#endif
}

bool WriteFile(const std::string& path, const std::string& content) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::SetStringToASCIIFile(content, path);
#else
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    out << content;
    return static_cast<bool>(out);
#endif
}

bool WriteProtoBinary(const google::protobuf::Message& message,
                      const std::string& path) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::SetProtoToBinaryFile(message, path);
#else
    return WriteFile(path, message.SerializeAsString());
#endif
}

bool ReadProtoBinary(const std::string& path,
                     google::protobuf::Message* message) {
    if (message == nullptr) {
        return false;
    }
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::GetProtoFromBinaryFile(path, message);
#else
    std::string bytes;
    if (!ReadFile(path, &bytes)) {
        return false;
    }
    return message->ParseFromString(bytes);
#endif
}

std::string DirName(const std::string& path) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::GetDirName(path);
#else
    const auto pos = path.find_last_of('/');
    if (pos == std::string::npos) {
        return ".";
    }
    if (pos == 0) {
        return "/";
    }
    return path.substr(0, pos);
#endif
}

std::string JoinPath(const std::string& a, const std::string& b) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::GetAbsolutePath(a, b);
#else
    if (a.empty()) {
        return b;
    }
    if (a.back() == '/') {
        return a + b;
    }
    return a + "/" + b;
#endif
}

std::vector<std::string> ListFiles(const std::string& directory_path) {
#if defined(AUTOLOOP_WITH_AUTOLINK)
    return autolink::common::ListSubPaths(directory_path, DT_REG);
#else
    (void)directory_path;
    return {};
#endif
}

}  // namespace common
}  // namespace autoloop
