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
 * @file json_util.cpp
 * @brief Flat JSON object encode/decode (no third-party JSON).
 */

#include "common/base/json_util.hpp"

#include <sstream>

namespace autoloop {
namespace common {

std::string JsonEscape(std::string_view s) {
    std::string o;
    o.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '"':
                o += "\\\"";
                break;
            case '\\':
                o += "\\\\";
                break;
            case '\n':
                o += "\\n";
                break;
            case '\r':
                o += "\\r";
                break;
            case '\t':
                o += "\\t";
                break;
            default:
                o.push_back(c);
                break;
        }
    }
    return o;
}

std::string JsonString(std::string_view s) {
    return "\"" + JsonEscape(s) + "\"";
}

std::string JsonObject(const std::map<std::string, std::string>& fields) {
    std::ostringstream ss;
    ss << '{';
    bool first = true;
    for (const auto& [k, v] : fields) {
        if (!first) {
            ss << ',';
        }
        first = false;
        ss << JsonString(k) << ':' << JsonString(v);
    }
    ss << '}';
    return ss.str();
}

bool JsonGetString(std::string_view json, std::string_view key,
                   std::string* value) {
    if (value == nullptr) {
        return false;
    }
    const std::string needle = "\"" + std::string(key) + "\":\"";
    const auto pos = json.find(needle);
    if (pos == std::string_view::npos) {
        return false;
    }
    size_t i = pos + needle.size();
    std::string out;
    while (i < json.size()) {
        const char c = json[i];
        if (c == '\\' && i + 1 < json.size()) {
            const char n = json[i + 1];
            if (n == '"' || n == '\\') {
                out.push_back(n);
            } else if (n == 'n') {
                out.push_back('\n');
            } else {
                out.push_back(n);
            }
            i += 2;
            continue;
        }
        if (c == '"') {
            *value = std::move(out);
            return true;
        }
        out.push_back(c);
        ++i;
    }
    return false;
}

}  // namespace common
}  // namespace autoloop
