/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tf_buffer_test.cpp
 * @brief TF buffer regression: insert, chain lookup, revision and misses.
 */

#include <cmath>
#include <cstdio>
#include <string>

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

#include "autoviz/transform/buffer.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

automsgs::msgs::geometry_msgs::TransformStamped MakeTransform(
    const std::string& parent, const std::string& child, double x, double y,
    double z) {
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  tf.mutable_header()->set_frame_id(parent);
  tf.mutable_header()->mutable_stamp()->set_sec(1);
  tf.set_child_frame_id(child);
  auto* translation = tf.mutable_transform()->mutable_translation();
  translation->set_x(x);
  translation->set_y(y);
  translation->set_z(z);
  auto* rotation = tf.mutable_transform()->mutable_rotation();
  rotation->set_w(1.0);
  rotation->set_x(0.0);
  rotation->set_y(0.0);
  rotation->set_z(0.0);
  return tf;
}

bool Near(double a, double b) { return std::fabs(a - b) < 1e-6; }

}  // namespace

int main() {
  auto* buffer = autoviz::transform::Buffer::Instance();
  buffer->clear();

  const std::uint64_t revision_before = buffer->revision();
  buffer->setTransform(MakeTransform("map", "odom", 1.0, 0.0, 0.0),
                       "test");
  buffer->setTransform(MakeTransform("odom", "base_link", 2.0, 0.0, 0.0),
                       "test");
  Check(buffer->revision() > revision_before,
        "inserting transforms bumps the revision");

  automsgs::msgs::builtin_interfaces::Time latest;
  Check(buffer->canTransform("map", "base_link", latest),
        "chained transform is resolvable");
  Check(!buffer->canTransform("map", "missing_link", latest),
        "unknown frame is rejected");

  try {
    const auto tf = buffer->lookupTransform("map", "base_link", latest);
    Check(Near(tf.transform().translation().x(), 3.0),
          "chained translation is composed");
    Check(tf.header().frame_id() == "map", "lookup fills the target frame");
    Check(tf.child_frame_id() == "base_link",
          "lookup fills the source frame");
  } catch (...) {
    Check(false, "lookupTransform must not throw for a valid chain");
  }

  const std::uint64_t revision_after = buffer->revision();
  buffer->clear();
  Check(buffer->revision() > revision_after, "clear() bumps the revision");
  Check(!buffer->canTransform("map", "base_link", latest),
        "clear() drops all frames");

  if (g_failures != 0) {
    std::fprintf(stderr, "%d tf buffer test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("tf buffer tests passed\n");
  return 0;
}

