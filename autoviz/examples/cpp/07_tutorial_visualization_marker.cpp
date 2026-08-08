/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <automsgs/msgs/visualization_msgs/interactive_marker.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_control.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_init.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_update.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using Marker = automsgs::msgs::visualization_msgs::Marker;
using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

Marker MakeMarker(int mid, Marker::Type mtype, double x, double y, double z,
                  double sx, double sy, double sz, float r, float g, float b,
                  const char* ns = "demo") {
  Marker m;
  m.set_ns(ns);
  m.set_id(mid);
  m.set_type(mtype);
  m.set_action(Marker::ADD);
  StampHeader(m.mutable_header());
  m.mutable_pose()->mutable_position()->set_x(x);
  m.mutable_pose()->mutable_position()->set_y(y);
  m.mutable_pose()->mutable_position()->set_z(z);
  m.mutable_pose()->mutable_orientation()->set_w(1.0);
  m.mutable_scale()->set_x(sx);
  m.mutable_scale()->set_y(sy);
  m.mutable_scale()->set_z(sz);
  m.mutable_color()->set_r(r);
  m.mutable_color()->set_g(g);
  m.mutable_color()->set_b(b);
  m.mutable_color()->set_a(1.0f);
  return m;
}

std::shared_ptr<Marker> MakeArrow(double t) {
  auto m = std::make_shared<Marker>(
      MakeMarker(0, Marker::ARROW, std::cos(t), std::sin(t), 0.2, 0.6, 0.1,
                 0.1, 1.0f, 0.4f, 0.1f));
  const double yaw = t + M_PI / 2;
  m->mutable_pose()->mutable_orientation()->set_z(std::sin(yaw * 0.5));
  m->mutable_pose()->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  return m;
}

std::shared_ptr<automsgs::msgs::visualization_msgs::MarkerArray> MakeArray() {
  auto arr =
      std::make_shared<automsgs::msgs::visualization_msgs::MarkerArray>();
  *arr->add_markers() = MakeMarker(1, Marker::CUBE, -1.5, 0, 0.2, 0.3, 0.3,
                                   0.3, 0.9f, 0.3f, 0.3f, "shapes");
  *arr->add_markers() = MakeMarker(2, Marker::SPHERE, -1.5, 1, 0.2, 0.3, 0.3,
                                   0.3, 0.3f, 0.9f, 0.4f, "shapes");
  *arr->add_markers() =
      MakeMarker(3, Marker::CYLINDER, -1.5, -1, 0.25, 0.25, 0.25, 0.5, 0.4f,
                 0.4f, 0.95f, "shapes");
  auto text = MakeMarker(4, Marker::TEXT_VIEW_FACING, -1.5, 0, 0.6, 0, 0, 0.2,
                         1.f, 1.f, 1.f, "shapes");
  text.set_text("MarkerArray");
  *arr->add_markers() = text;
  return arr;
}

automsgs::msgs::visualization_msgs::InteractiveMarker MakeInteractive() {
  using IMC = automsgs::msgs::visualization_msgs::InteractiveMarkerControl;
  automsgs::msgs::visualization_msgs::InteractiveMarker im;
  im.set_name("drag_me");
  im.set_description("MOVE_3D");
  im.set_scale(0.5f);
  StampHeader(im.mutable_header());
  im.mutable_pose()->mutable_position()->set_x(1.5);
  im.mutable_pose()->mutable_position()->set_z(0.25);
  im.mutable_pose()->mutable_orientation()->set_w(1.0);
  auto* ctrl = im.add_controls();
  ctrl->set_name("move");
  ctrl->set_interaction_mode(IMC::MOVE_3D);
  ctrl->set_always_visible(true);
  auto* vis = ctrl->add_markers();
  vis->set_type(Marker::CUBE);
  vis->mutable_scale()->set_x(0.25);
  vis->mutable_scale()->set_y(0.25);
  vis->mutable_scale()->set_z(0.25);
  vis->mutable_color()->set_r(1.0f);
  vis->mutable_color()->set_g(0.85f);
  vis->mutable_color()->set_b(0.2f);
  vis->mutable_color()->set_a(1.0f);
  vis->mutable_pose()->mutable_orientation()->set_w(1.0);
  return im;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/marker");
  auto w_m = MakeWriter<Marker>(node, "/fake/marker");
  auto w_a = MakeWriter<automsgs::msgs::visualization_msgs::MarkerArray>(
      node, "/fake/marker_array");
  auto w_i =
      MakeWriter<automsgs::msgs::visualization_msgs::InteractiveMarkerInit>(
          node, "/fake/init");
  auto w_u =
      MakeWriter<automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>(
          node, "/fake/update");

  const auto shapes = MakeArray();
  const auto im = MakeInteractive();

  autolink::Rate rate(rate_hz);
  std::cout << "marker @ " << rate_hz
            << " Hz (Interact → drag yellow cube)\n";

  const double t0 = NowSec();
  uint64_t seq = 0;
  while (autolink::OK()) {
    w_m->Write(MakeArrow(NowSec() - t0));
    w_a->Write(shapes);

    auto init = std::make_shared<
        automsgs::msgs::visualization_msgs::InteractiveMarkerInit>();
    init->set_server_id("tutorial");
    init->set_seq_num(seq);
    *init->add_markers() = im;
    w_i->Write(init);

    auto upd = std::make_shared<
        automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>();
    upd->set_server_id("tutorial");
    upd->set_seq_num(seq);
    upd->set_type(
        automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate::UPDATE);
    *upd->add_markers() = im;
    w_u->Write(upd);

    ++seq;
    rate.Sleep();
  }
  return 0;
}
