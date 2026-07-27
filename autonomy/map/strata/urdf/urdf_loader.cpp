/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <fstream>
#include <regex>
#include <sstream>

#include "autonomy/map/strata/urdf/urdf_loader.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace urdf {

namespace {

std::string NormalizePath(const std::string& url) {
    if (url.rfind("file://", 0) == 0) {
        return url.substr(7);
    }
    return url;
}

std::string ExtractAttr(const std::string& tag, const std::string& attr) {
    const std::regex pattern(attr + R"regex(="([^"]*)")regex");
    std::smatch match;
    if (std::regex_search(tag, match, pattern) && match.size() > 1) {
        return match[1].str();
    }
    return {};
}

std::string ReadFile(const std::string& path, std::string* error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        if (error != nullptr) {
            *error = "Failed to open URDF: " + path;
        }
        return {};
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

}  // namespace

bool LoadUrdfFile(const std::string& url, UrdfModel& model, std::string* error) {
    const std::string path = NormalizePath(url);
    const std::string xml = ReadFile(path, error);
    if (xml.empty()) {
        return false;
    }

    model = UrdfModel{};
    model.sourcePath = path;

    const std::regex robot_tag(R"regex(<robot\b[^>]*name="([^"]*)")regex");
    std::smatch robot_match;
    if (std::regex_search(xml, robot_match, robot_tag) && robot_match.size() > 1) {
        model.name = robot_match[1].str();
    }

    const std::regex link_tag(R"regex(<link\b[^>]*name="([^"]*)")regex");
    for (std::sregex_iterator it(xml.begin(), xml.end(), link_tag), end; it != end; ++it) {
        if ((*it).size() > 1) {
            model.links.push_back((*it)[1].str());
        }
    }

    const std::regex joint_open(R"regex(<joint\b[^>]*>)regex");
    for (std::sregex_iterator it(xml.begin(), xml.end(), joint_open), end; it != end; ++it) {
        const std::string open_tag = (*it)[0].str();
        UrdfJoint joint;
        joint.name = ExtractAttr(open_tag, "name");
        joint.type = ExtractAttr(open_tag, "type");
        const auto start = static_cast<size_t>((*it).position() + (*it).length());
        const auto close_pos = xml.find("</joint>", start);
        if (close_pos == std::string::npos) {
            continue;
        }
        const std::string body = xml.substr(start, close_pos - start);
        const std::regex parent_tag(R"regex(<parent\s+link="([^"]*)")regex");
        const std::regex child_tag(R"regex(<child\s+link="([^"]*)")regex");
        std::smatch parent_match;
        std::smatch child_match;
        if (std::regex_search(body, parent_match, parent_tag) && parent_match.size() > 1) {
            joint.parent = parent_match[1].str();
        }
        if (std::regex_search(body, child_match, child_tag) && child_match.size() > 1) {
            joint.child = child_match[1].str();
        }
        if (!joint.name.empty()) {
            model.joints.push_back(std::move(joint));
        }
    }

    if (model.links.empty() && model.joints.empty()) {
        if (error != nullptr) {
            *error = "URDF contains no links or joints: " + path;
        }
        return false;
    }
    return true;
}

}  // namespace urdf
}  // namespace strata
}  // namespace map
}  // namespace autonomy
