/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/core/urdf_kdl.hpp"

#include <cmath>
#include <fstream>
#include <queue>
#include <regex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

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

bool ParseVec3(const std::string& text, double* x, double* y, double* z) {
  if (!x || !y || !z) {
    return false;
  }
  std::istringstream iss(text);
  if (!(iss >> *x >> *y >> *z)) {
    *x = *y = *z = 0.0;
    return false;
  }
  return true;
}

KDL::Frame FrameFromOrigin(const std::string& block) {
  double xyz[3] = {0.0, 0.0, 0.0};
  double rpy[3] = {0.0, 0.0, 0.0};
  const std::regex origin_re(R"regex(<origin\b[^>]*/?>)regex");
  std::smatch match;
  if (std::regex_search(block, match, origin_re)) {
    const std::string tag = match[0].str();
    ParseVec3(ExtractAttr(tag, "xyz"), &xyz[0], &xyz[1], &xyz[2]);
    ParseVec3(ExtractAttr(tag, "rpy"), &rpy[0], &rpy[1], &rpy[2]);
  }
  return KDL::Frame(KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]),
                    KDL::Vector(xyz[0], xyz[1], xyz[2]));
}

KDL::Vector AxisFromBlock(const std::string& block) {
  double x = 0.0;
  double y = 0.0;
  double z = 1.0;
  const std::regex axis_re(R"regex(<axis\b[^>]*/?>)regex");
  std::smatch match;
  if (std::regex_search(block, match, axis_re)) {
    ParseVec3(ExtractAttr(match[0].str(), "xyz"), &x, &y, &z);
  }
  const double norm = std::sqrt(x * x + y * y + z * z);
  if (norm < 1e-12) {
    return KDL::Vector(0.0, 0.0, 1.0);
  }
  return KDL::Vector(x / norm, y / norm, z / norm);
}

void LimitsFromBlock(const std::string& block, const std::string& type,
                     double* lower, double* upper) {
  *lower = -3.141592653589793;
  *upper = 3.141592653589793;
  if (type == "continuous") {
    *lower = -1e9;
    *upper = 1e9;
    return;
  }
  const std::regex limit_re(R"regex(<limit\b[^>]*/?>)regex");
  std::smatch match;
  if (!std::regex_search(block, match, limit_re)) {
    return;
  }
  const std::string tag = match[0].str();
  const std::string lo = ExtractAttr(tag, "lower");
  const std::string hi = ExtractAttr(tag, "upper");
  if (!lo.empty()) {
    *lower = std::stod(lo);
  }
  if (!hi.empty()) {
    *upper = std::stod(hi);
  }
}

struct UrdfJointNode {
  std::string name;
  std::string type;
  std::string parent;
  std::string child;
  std::string block;
};

bool ParseJoints(const std::string& xml, std::vector<UrdfJointNode>* joints,
                 std::string* error) {
  joints->clear();
  const std::regex joint_re(
      R"regex(<joint\b([^>]*)>([\s\S]*?)</joint>)regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), joint_re), end;
       it != end; ++it) {
    UrdfJointNode node;
    const std::string open_attrs = (*it)[1].str();
    node.block = (*it)[0].str();
    node.name = ExtractAttr(open_attrs, "name");
    if (node.name.empty()) {
      // Attributes may sit in the full open tag without capture group padding.
      node.name = ExtractAttr("<joint " + open_attrs + ">", "name");
    }
    node.type = ExtractAttr(open_attrs, "type");
    if (node.type.empty()) {
      node.type = ExtractAttr("<joint " + open_attrs + ">", "type");
    }

    const std::regex parent_re(R"regex(<parent\b[^>]*/?>)regex");
    const std::regex child_re(R"regex(<child\b[^>]*/?>)regex");
    std::smatch m;
    if (std::regex_search(node.block, m, parent_re)) {
      node.parent = ExtractAttr(m[0].str(), "link");
    }
    if (std::regex_search(node.block, m, child_re)) {
      node.child = ExtractAttr(m[0].str(), "link");
    }
    if (node.name.empty() || node.parent.empty() || node.child.empty() ||
        node.type.empty()) {
      continue;
    }
    joints->push_back(std::move(node));
  }

  if (joints->empty()) {
    if (error) {
      *error = "URDF has no joints";
    }
    return false;
  }
  return true;
}

KDL::Joint MakeJoint(const UrdfJointNode& node) {
  if (node.type == "fixed" || node.type == "floating" ||
      node.type == "planar") {
    return KDL::Joint(node.name, KDL::Joint::None);
  }
  const KDL::Vector axis = AxisFromBlock(node.block);
  if (node.type == "prismatic") {
    return KDL::Joint(node.name, KDL::Vector::Zero(), axis,
                      KDL::Joint::TransAxis);
  }
  // revolute / continuous
  return KDL::Joint(node.name, KDL::Vector::Zero(), axis, KDL::Joint::RotAxis);
}

}  // namespace

bool BuildKdlChainFromUrdf(const std::string& urdf_path_in,
                           const std::string& base_link,
                           const std::string& tip_link, KdlChainModel* out,
                           std::string* error) {
  if (!out || base_link.empty() || tip_link.empty()) {
    if (error) {
      *error = "invalid BuildKdlChainFromUrdf arguments";
    }
    return false;
  }

  const std::string path = NormalizePath(urdf_path_in);
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

  std::vector<UrdfJointNode> joints;
  if (!ParseJoints(xml, &joints, error)) {
    return false;
  }

  std::unordered_set<std::string> links;
  std::unordered_map<std::string, std::vector<const UrdfJointNode*>> by_parent;
  for (const auto& joint : joints) {
    links.insert(joint.parent);
    links.insert(joint.child);
    by_parent[joint.parent].push_back(&joint);
  }

  std::string root = base_link;
  if (links.find(root) == links.end()) {
    if (error) {
      *error = "base_link not found in URDF: " + base_link;
    }
    return false;
  }

  KDL::Tree tree(root);
  std::queue<std::string> queue;
  queue.push(root);
  std::unordered_set<std::string> visited;
  visited.insert(root);

  while (!queue.empty()) {
    const std::string parent = queue.front();
    queue.pop();
    const auto it = by_parent.find(parent);
    if (it == by_parent.end()) {
      continue;
    }
    for (const UrdfJointNode* joint : it->second) {
      if (visited.count(joint->child)) {
        continue;
      }
      const KDL::Frame tip_frame = FrameFromOrigin(joint->block);
      const KDL::Segment segment(joint->child, MakeJoint(*joint), tip_frame);
      if (!tree.addSegment(segment, parent)) {
        if (error) {
          *error = "Failed to add segment " + joint->name;
        }
        return false;
      }
      visited.insert(joint->child);
      queue.push(joint->child);
    }
  }

  if (!tree.getChain(base_link, tip_link, out->chain)) {
    if (error) {
      *error = "No KDL chain from " + base_link + " to " + tip_link;
    }
    return false;
  }

  out->joint_names.clear();
  std::vector<double> mins;
  std::vector<double> maxs;
  for (unsigned int i = 0; i < out->chain.getNrOfSegments(); ++i) {
    const KDL::Joint& joint = out->chain.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    out->joint_names.push_back(joint.getName());

    // Recover limits from the matching URDF joint block.
    double lower = -3.141592653589793;
    double upper = 3.141592653589793;
    for (const auto& node : joints) {
      if (node.name == joint.getName()) {
        LimitsFromBlock(node.block, node.type, &lower, &upper);
        break;
      }
    }
    mins.push_back(lower);
    maxs.push_back(upper);
  }

  out->q_min = KDL::JntArray(mins.size());
  out->q_max = KDL::JntArray(maxs.size());
  for (std::size_t i = 0; i < mins.size(); ++i) {
    out->q_min(i) = mins[i];
    out->q_max(i) = maxs[i];
  }

  if (out->joint_names.empty()) {
    if (error) {
      *error = "KDL chain has no movable joints";
    }
    return false;
  }
  return true;
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
