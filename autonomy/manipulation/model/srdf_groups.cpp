/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/srdf_groups.hpp"

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

std::string ReadFile(const std::string& path_in, std::string* error) {
  const std::string path = NormalizePath(path_in);
  std::ifstream input(path);
  if (!input.is_open()) {
    if (error) {
      *error = "Failed to open SRDF: " + path;
    }
    return {};
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

}  // namespace

bool LoadSrdfGroups(const std::string& path_in,
                    std::unordered_map<std::string, JointModelGroup>* groups,
                    std::string* error) {
  if (!groups) {
    return false;
  }
  groups->clear();
  if (path_in.empty()) {
    return true;
  }

  const std::string xml = ReadFile(path_in, error);
  if (xml.empty() && error && !error->empty()) {
    return false;
  }

  const std::regex group_re(
      R"regex(<group\b([^>]*)>([\s\S]*?)</group>)regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), group_re), end;
       it != end; ++it) {
    JointModelGroup group;
    group.name = ExtractAttr((*it)[1].str(), "name");
    if (group.name.empty()) {
      continue;
    }
    const std::string body = (*it)[2].str();
    const std::regex joint_re(R"regex(<joint\b[^>]*/?>)regex");
    for (std::sregex_iterator jt(body.begin(), body.end(), joint_re), jend;
         jt != jend; ++jt) {
      const std::string jn = ExtractAttr((*jt)[0].str(), "name");
      if (!jn.empty()) {
        group.joint_names.push_back(jn);
      }
    }
    const std::regex chain_re(R"regex(<chain\b[^>]*/?>)regex");
    std::smatch cm;
    if (std::regex_search(body, cm, chain_re)) {
      group.base_frame = ExtractAttr(cm[0].str(), "base_link");
      group.tip_frame = ExtractAttr(cm[0].str(), "tip_link");
      group.is_chain = true;
    }
    (*groups)[group.name] = std::move(group);
  }
  return true;
}

bool LoadSrdfDisableCollisions(
    const std::string& path_in,
    std::vector<std::pair<std::string, std::string>>* pairs,
    std::string* error) {
  if (!pairs) {
    return false;
  }
  pairs->clear();
  if (path_in.empty()) {
    return true;
  }
  const std::string xml = ReadFile(path_in, error);
  if (xml.empty() && error && !error->empty()) {
    return false;
  }
  const std::regex dc_re(R"regex(<disable_collisions\b[^>]*/?>)regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), dc_re), end; it != end;
       ++it) {
    const std::string tag = (*it)[0].str();
    const std::string a = ExtractAttr(tag, "link1");
    const std::string b = ExtractAttr(tag, "link2");
    if (!a.empty() && !b.empty()) {
      pairs->emplace_back(a, b);
    }
  }
  return true;
}

bool LoadSrdfEndEffectors(const std::string& path_in,
                          std::vector<SrdfEndEffector>* effectors,
                          std::string* error) {
  if (!effectors) {
    return false;
  }
  effectors->clear();
  if (path_in.empty()) {
    return true;
  }
  const std::string xml = ReadFile(path_in, error);
  if (xml.empty() && error && !error->empty()) {
    return false;
  }
  const std::regex ee_re(R"regex(<end_effector\b[^>]*/?>)regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), ee_re), end; it != end;
       ++it) {
    const std::string tag = (*it)[0].str();
    SrdfEndEffector ee;
    ee.name = ExtractAttr(tag, "name");
    ee.group = ExtractAttr(tag, "group");
    ee.parent_link = ExtractAttr(tag, "parent_link");
    ee.parent_group = ExtractAttr(tag, "parent_group");
    if (!ee.name.empty()) {
      effectors->push_back(std::move(ee));
    }
  }
  return true;
}

bool LoadSrdfPassiveJoints(const std::string& path_in,
                           std::vector<std::string>* joints,
                           std::string* error) {
  if (!joints) {
    return false;
  }
  joints->clear();
  if (path_in.empty()) {
    return true;
  }
  const std::string xml = ReadFile(path_in, error);
  if (xml.empty() && error && !error->empty()) {
    return false;
  }
  const std::regex pj_re(R"regex(<passive_joint\b[^>]*/?>)regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), pj_re), end; it != end;
       ++it) {
    const std::string name = ExtractAttr((*it)[0].str(), "name");
    if (!name.empty()) {
      joints->push_back(name);
    }
  }
  return true;
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
