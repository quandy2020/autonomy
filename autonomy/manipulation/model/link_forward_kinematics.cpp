/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/link_forward_kinematics.hpp"

#include "autonomy/manipulation/model/apply_mimic_joints.hpp"
#include "autonomy/manipulation/model/pose_math.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>
#include <unordered_set>

namespace autonomy {
namespace manipulation {
namespace model {
namespace {

std::string NormalizeFileUrlPath(const std::string& url) {
  if (url.rfind("file://", 0) == 0) {
    return url.substr(7);
  }
  return url;
}

std::string ExtractXmlAttribute(const std::string& tag,
                                const std::string& attribute) {
  const std::regex pattern(attribute + R"regex(="([^"]*)")regex");
  std::smatch match;
  if (std::regex_search(tag, match, pattern) && match.size() > 1) {
    return match[1].str();
  }
  return {};
}

bool ParseVector3(const std::string& text, double* x, double* y, double* z) {
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

}  // namespace

bool LinkForwardKinematicsTree::LoadFromUrdfFile(const std::string& urdf_path,
                                                 std::string* error) {
  joints_.clear();
  link_names_.clear();
  child_link_to_joint_index_.clear();
  root_link_name_.clear();
  if (urdf_path.empty()) {
    return true;
  }

  const std::string path = NormalizeFileUrlPath(urdf_path);
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

  std::unordered_set<std::string> link_set;
  const std::regex link_re(R"regex(<link\b[^>]*name="([^"]+)")regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), link_re), end; it != end;
       ++it) {
    link_set.insert((*it)[1].str());
  }

  const std::regex joint_re(
      R"regex(<joint\b([^>]*)>([\s\S]*?)</joint>)regex");
  std::unordered_set<std::string> child_links;
  for (std::sregex_iterator it(xml.begin(), xml.end(), joint_re), end;
       it != end; ++it) {
    JointModel joint;
    const std::string attrs = (*it)[1].str();
    const std::string body = (*it)[2].str();
    joint.set_name(ExtractXmlAttribute(attrs, "name"));
    joint.set_type(ExtractXmlAttribute(attrs, "type"));
    const std::regex parent_re(R"regex(<parent\b[^>]*/?>)regex");
    const std::regex child_re(R"regex(<child\b[^>]*/?>)regex");
    std::smatch match;
    if (std::regex_search(body, match, parent_re)) {
      joint.set_parent_link(ExtractXmlAttribute(match[0].str(), "link"));
    }
    if (std::regex_search(body, match, child_re)) {
      joint.set_child_link(ExtractXmlAttribute(match[0].str(), "link"));
    }
    if (joint.name().empty() || joint.parent_link().empty() ||
        joint.child_link().empty()) {
      continue;
    }
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    double oroll = 0.0;
    double opitch = 0.0;
    double oyaw = 0.0;
    const std::regex origin_re(R"regex(<origin\b[^>]*/?>)regex");
    if (std::regex_search(body, match, origin_re)) {
      ParseVector3(ExtractXmlAttribute(match[0].str(), "xyz"), &ox, &oy, &oz);
      ParseVector3(ExtractXmlAttribute(match[0].str(), "rpy"), &oroll, &opitch,
                   &oyaw);
    }
    *joint.mutable_origin() =
        PoseFromRollPitchYawXyz(oroll, opitch, oyaw, ox, oy, oz);
    double ax = 0.0;
    double ay = 0.0;
    double az = 1.0;
    const std::regex axis_re(R"regex(<axis\b[^>]*/?>)regex");
    if (std::regex_search(body, match, axis_re)) {
      ParseVector3(ExtractXmlAttribute(match[0].str(), "xyz"), &ax, &ay, &az);
    }
    joint.mutable_axis()->set_x(ax);
    joint.mutable_axis()->set_y(ay);
    joint.mutable_axis()->set_z(az);
    const std::regex mimic_re(R"regex(<mimic\b[^>]*/?>)regex");
    if (std::regex_search(body, match, mimic_re)) {
      joint.set_mimic_joint(ExtractXmlAttribute(match[0].str(), "joint"));
      const std::string multiplier =
          ExtractXmlAttribute(match[0].str(), "multiplier");
      const std::string offset = ExtractXmlAttribute(match[0].str(), "offset");
      if (!multiplier.empty()) {
        joint.set_mimic_factor(std::strtod(multiplier.c_str(), nullptr));
      } else {
        joint.set_mimic_factor(1.0);
      }
      if (!offset.empty()) {
        joint.set_mimic_offset(std::strtod(offset.c_str(), nullptr));
      }
    }
    child_links.insert(joint.child_link());
    link_set.insert(joint.parent_link());
    link_set.insert(joint.child_link());
    child_link_to_joint_index_[joint.child_link()] = joints_.size();
    joints_.push_back(std::move(joint));
  }

  for (const auto& link : link_set) {
    if (!child_links.count(link)) {
      root_link_name_ = link;
      break;
    }
  }
  if (root_link_name_.empty() && !link_set.empty()) {
    root_link_name_ = *link_set.begin();
  }
  link_names_.assign(link_set.begin(), link_set.end());
  return !joints_.empty();
}

bool LinkForwardKinematicsTree::ComputeAllLinkPoses(
    const automsgs::msgs::sensor_msgs::JointState& joint_state,
    std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose>*
        link_poses) const {
  if (!link_poses) {
    return false;
  }
  link_poses->clear();
  if (root_link_name_.empty()) {
    return false;
  }
  (*link_poses)[root_link_name_] = IdentityPose();

  automsgs::msgs::sensor_msgs::JointState expanded = joint_state;
  ApplyMimicJoints(joints_, &expanded);

  std::unordered_map<std::string, double> positions_by_name;
  for (int i = 0; i < expanded.name_size() && i < expanded.position_size();
       ++i) {
    positions_by_name[expanded.name(i)] = expanded.position(i);
  }

  bool progressed = true;
  int guard = 0;
  while (progressed && guard++ < static_cast<int>(joints_.size()) + 2) {
    progressed = false;
    for (const auto& joint : joints_) {
      if (!link_poses->count(joint.parent_link()) ||
          link_poses->count(joint.child_link())) {
        continue;
      }
      automsgs::msgs::geometry_msgs::Pose joint_motion = IdentityPose();
      const double position = positions_by_name.count(joint.name())
                                  ? positions_by_name[joint.name()]
                                  : 0.0;
      if (joint.type() == "revolute" || joint.type() == "continuous") {
        joint_motion = PoseFromAxisAngle(joint.axis().x(), joint.axis().y(),
                                         joint.axis().z(), position);
      } else if (joint.type() == "prismatic") {
        joint_motion.mutable_position()->set_x(joint.axis().x() * position);
        joint_motion.mutable_position()->set_y(joint.axis().y() * position);
        joint_motion.mutable_position()->set_z(joint.axis().z() * position);
      }
      (*link_poses)[joint.child_link()] = ComposePoses(
          (*link_poses)[joint.parent_link()],
          ComposePoses(joint.origin(), joint_motion));
      progressed = true;
    }
  }
  return link_poses->size() >= 1;
}

bool LinkForwardKinematicsTree::GetLinkPose(
    const automsgs::msgs::sensor_msgs::JointState& joint_state,
    const std::string& link_name,
    automsgs::msgs::geometry_msgs::Pose* link_pose) const {
  if (!link_pose) {
    return false;
  }
  std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose>
      link_poses;
  if (!ComputeAllLinkPoses(joint_state, &link_poses)) {
    return false;
  }
  const auto it = link_poses.find(link_name);
  if (it == link_poses.end()) {
    return false;
  }
  *link_pose = it->second;
  return true;
}

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
