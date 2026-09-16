/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/core/urdf_joints.hpp"

#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>

namespace autonomy {
namespace manipulation {
namespace core {
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

bool IsMovable(const std::string& type) {
  return type == "revolute" || type == "continuous" || type == "prismatic";
}

}  // namespace

bool LoadUrdfJoints(const std::string& path_in,
                    std::vector<UrdfJointInfo>* joints, std::string* error) {
  if (!joints) {
    return false;
  }
  joints->clear();

  const std::string path = NormalizePath(path_in);
  std::ifstream input(path);
  if (!input.is_open()) {
    if (error) {
      *error = "Failed to open URDF: " + path;
    }
    return false;
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  const std::string xml = buffer.str();

  const std::regex joint_block(
      R"regex(<joint\b([^>]*)>([\s\S]*?)</joint>)regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), joint_block), end;
       it != end; ++it) {
    const std::string open_attrs = (*it)[1].str();
    const std::string body = (*it)[2].str();
    UrdfJointInfo joint;
    joint.name = ExtractAttr(open_attrs, "name");
    joint.type = ExtractAttr(open_attrs, "type");
    if (joint.name.empty() || !IsMovable(joint.type)) {
      continue;
    }
    if (joint.type == "continuous") {
      joint.has_position_limits = false;
    } else {
      const std::regex limit_re(R"regex(<limit\b[^>]*/?>)regex");
      std::smatch lm;
      if (std::regex_search(body, lm, limit_re)) {
        const std::string tag = lm[0].str();
        const std::string lo = ExtractAttr(tag, "lower");
        const std::string hi = ExtractAttr(tag, "upper");
        const std::string vel = ExtractAttr(tag, "velocity");
        const std::string eff = ExtractAttr(tag, "effort");
        if (!lo.empty() && !hi.empty()) {
          joint.has_position_limits = true;
          joint.lower = std::strtod(lo.c_str(), nullptr);
          joint.upper = std::strtod(hi.c_str(), nullptr);
        }
        if (!vel.empty()) {
          joint.velocity = std::strtod(vel.c_str(), nullptr);
        }
        if (!eff.empty()) {
          joint.effort = std::strtod(eff.c_str(), nullptr);
        }
      } else {
        joint.has_position_limits = true;
      }
    }
    const std::regex mimic_re(R"regex(<mimic\b[^>]*/?>)regex");
    std::smatch mm;
    if (std::regex_search(body, mm, mimic_re)) {
      const std::string tag = mm[0].str();
      joint.mimic_joint = ExtractAttr(tag, "joint");
      const std::string mult = ExtractAttr(tag, "multiplier");
      const std::string off = ExtractAttr(tag, "offset");
      if (!mult.empty()) {
        joint.mimic_multiplier = std::strtod(mult.c_str(), nullptr);
      }
      if (!off.empty()) {
        joint.mimic_offset = std::strtod(off.c_str(), nullptr);
      }
    }
    joints->push_back(std::move(joint));
  }

  if (joints->empty()) {
    if (error) {
      *error = "URDF has no movable joints: " + path;
    }
    return false;
  }
  return true;
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
