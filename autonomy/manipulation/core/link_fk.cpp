/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/core/link_fk.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>
#include <unordered_set>
#include <algorithm>

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

Transform FromRpyXyz(double roll, double pitch, double yaw, double x, double y,
                     double z) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  Transform t;
  t.qw = cr * cp * cy + sr * sp * sy;
  t.qx = sr * cp * cy - cr * sp * sy;
  t.qy = cr * sp * cy + sr * cp * sy;
  t.qz = cr * cp * sy - sr * sp * cy;
  t.x = x;
  t.y = y;
  t.z = z;
  return t;
}

Transform AxisAngle(double ax, double ay, double az, double angle) {
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  const double nx = n > 1e-12 ? ax / n : 0.0;
  const double ny = n > 1e-12 ? ay / n : 0.0;
  const double nz = n > 1e-12 ? az / n : 1.0;
  const double s = std::sin(angle * 0.5);
  Transform t = IdentityTransform();
  t.qw = std::cos(angle * 0.5);
  t.qx = nx * s;
  t.qy = ny * s;
  t.qz = nz * s;
  return t;
}

void RotateVec(const Transform& t, double x, double y, double z, double* ox,
               double* oy, double* oz) {
  // q * v * q^{-1}
  const double qw = t.qw;
  const double qx = t.qx;
  const double qy = t.qy;
  const double qz = t.qz;
  const double ix = qw * x + qy * z - qz * y;
  const double iy = qw * y + qz * x - qx * z;
  const double iz = qw * z + qx * y - qy * x;
  const double iw = -qx * x - qy * y - qz * z;
  *ox = ix * qw + iw * -qx + iy * -qz - iz * -qy;
  *oy = iy * qw + iw * -qy + iz * -qx - ix * -qz;
  *oz = iz * qw + iw * -qz + ix * -qy - iy * -qx;
}

}  // namespace

Transform IdentityTransform() {
  return {};
}

void ApplyMimicJoints(const std::vector<LinkFkJoint>& joints, JointState* state) {
  if (!state) {
    return;
  }
  std::unordered_map<std::string, double> qmap;
  for (std::size_t i = 0; i < state->names.size() && i < state->positions.size();
       ++i) {
    qmap[state->names[i]] = state->positions[i];
  }
  for (const auto& j : joints) {
    if (j.mimic_joint.empty() || !qmap.count(j.mimic_joint)) {
      continue;
    }
    qmap[j.name] = j.mimic_multiplier * qmap[j.mimic_joint] + j.mimic_offset;
  }
  for (std::size_t i = 0; i < state->names.size() && i < state->positions.size();
       ++i) {
    const auto it = qmap.find(state->names[i]);
    if (it != qmap.end()) {
      state->positions[i] = it->second;
    }
  }
  for (const auto& j : joints) {
    if (j.mimic_joint.empty() || !qmap.count(j.name)) {
      continue;
    }
    if (std::find(state->names.begin(), state->names.end(), j.name) !=
        state->names.end()) {
      continue;
    }
    state->names.push_back(j.name);
    state->positions.push_back(qmap[j.name]);
  }
}

void ApplyMimicJoints(const std::vector<JointModel>& joints, JointState* state) {
  if (!state) {
    return;
  }
  std::vector<LinkFkJoint> proxy;
  proxy.reserve(joints.size());
  for (const auto& j : joints) {
    LinkFkJoint lj;
    lj.name = j.name;
    lj.mimic_joint = j.mimic_joint;
    lj.mimic_multiplier = j.mimic_factor;
    lj.mimic_offset = j.mimic_offset;
    proxy.push_back(std::move(lj));
  }
  ApplyMimicJoints(proxy, state);
}

Transform Compose(const Transform& a, const Transform& b) {
  Transform out;
  out.qw = a.qw * b.qw - a.qx * b.qx - a.qy * b.qy - a.qz * b.qz;
  out.qx = a.qw * b.qx + a.qx * b.qw + a.qy * b.qz - a.qz * b.qy;
  out.qy = a.qw * b.qy - a.qx * b.qz + a.qy * b.qw + a.qz * b.qx;
  out.qz = a.qw * b.qz + a.qx * b.qy - a.qy * b.qx + a.qz * b.qw;
  RotateVec(a, b.x, b.y, b.z, &out.x, &out.y, &out.z);
  out.x += a.x;
  out.y += a.y;
  out.z += a.z;
  return out;
}

bool LinkFkTree::LoadUrdf(const std::string& urdf_path, std::string* error) {
  joints_.clear();
  links_.clear();
  child_to_joint_.clear();
  root_.clear();
  if (urdf_path.empty()) {
    return true;
  }

  const std::string path = NormalizePath(urdf_path);
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
    LinkFkJoint j;
    const std::string attrs = (*it)[1].str();
    const std::string body = (*it)[2].str();
    j.name = ExtractAttr(attrs, "name");
    j.type = ExtractAttr(attrs, "type");
    const std::regex parent_re(R"regex(<parent\b[^>]*/?>)regex");
    const std::regex child_re(R"regex(<child\b[^>]*/?>)regex");
    std::smatch m;
    if (std::regex_search(body, m, parent_re)) {
      j.parent = ExtractAttr(m[0].str(), "link");
    }
    if (std::regex_search(body, m, child_re)) {
      j.child = ExtractAttr(m[0].str(), "link");
    }
    if (j.name.empty() || j.parent.empty() || j.child.empty()) {
      continue;
    }
    const std::regex origin_re(R"regex(<origin\b[^>]*/?>)regex");
    if (std::regex_search(body, m, origin_re)) {
      ParseVec3(ExtractAttr(m[0].str(), "xyz"), &j.ox, &j.oy, &j.oz);
      ParseVec3(ExtractAttr(m[0].str(), "rpy"), &j.oroll, &j.opitch, &j.oyaw);
    }
    const std::regex axis_re(R"regex(<axis\b[^>]*/?>)regex");
    if (std::regex_search(body, m, axis_re)) {
      ParseVec3(ExtractAttr(m[0].str(), "xyz"), &j.ax, &j.ay, &j.az);
    }
    const std::regex mimic_re(R"regex(<mimic\b[^>]*/?>)regex");
    if (std::regex_search(body, m, mimic_re)) {
      j.mimic_joint = ExtractAttr(m[0].str(), "joint");
      const std::string mult = ExtractAttr(m[0].str(), "multiplier");
      const std::string off = ExtractAttr(m[0].str(), "offset");
      if (!mult.empty()) {
        j.mimic_multiplier = std::strtod(mult.c_str(), nullptr);
      }
      if (!off.empty()) {
        j.mimic_offset = std::strtod(off.c_str(), nullptr);
      }
    }
    child_links.insert(j.child);
    link_set.insert(j.parent);
    link_set.insert(j.child);
    child_to_joint_[j.child] = joints_.size();
    joints_.push_back(std::move(j));
  }

  for (const auto& link : link_set) {
    if (!child_links.count(link)) {
      root_ = link;
      break;
    }
  }
  if (root_.empty() && !link_set.empty()) {
    root_ = *link_set.begin();
  }
  links_.assign(link_set.begin(), link_set.end());
  return !joints_.empty();
}

bool LinkFkTree::Compute(
    const JointState& state,
    std::unordered_map<std::string, Transform>* poses) const {
  if (!poses) {
    return false;
  }
  poses->clear();
  if (root_.empty()) {
    return false;
  }
  (*poses)[root_] = IdentityTransform();

  JointState expanded = state;
  ApplyMimicJoints(joints_, &expanded);

  std::unordered_map<std::string, double> qmap;
  for (std::size_t i = 0;
       i < expanded.names.size() && i < expanded.positions.size(); ++i) {
    qmap[expanded.names[i]] = expanded.positions[i];
  }

  // Propagate in joint declaration order repeatedly until stable.
  bool progressed = true;
  int guard = 0;
  while (progressed && guard++ < static_cast<int>(joints_.size()) + 2) {
    progressed = false;
    for (const auto& j : joints_) {
      if (!poses->count(j.parent) || poses->count(j.child)) {
        continue;
      }
      Transform origin = FromRpyXyz(j.oroll, j.opitch, j.oyaw, j.ox, j.oy, j.oz);
      Transform joint_tf = IdentityTransform();
      const double q = qmap.count(j.name) ? qmap[j.name] : 0.0;
      if (j.type == "revolute" || j.type == "continuous") {
        joint_tf = AxisAngle(j.ax, j.ay, j.az, q);
      } else if (j.type == "prismatic") {
        joint_tf.x = j.ax * q;
        joint_tf.y = j.ay * q;
        joint_tf.z = j.az * q;
      }
      (*poses)[j.child] = Compose((*poses)[j.parent], Compose(origin, joint_tf));
      progressed = true;
    }
  }
  return poses->size() >= 1;
}

bool LinkFkTree::GetLinkPose(const JointState& state, const std::string& link,
                             Transform* pose) const {
  if (!pose) {
    return false;
  }
  std::unordered_map<std::string, Transform> poses;
  if (!Compute(state, &poses)) {
    return false;
  }
  const auto it = poses.find(link);
  if (it == poses.end()) {
    return false;
  }
  *pose = it->second;
  return true;
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
