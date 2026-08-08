/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>

#include <automsgs/msgs/geometry_msgs/accel_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/point_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/map_msgs/grid_cells.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

std::shared_ptr<automsgs::msgs::map_msgs::GridCells> MakeGridCells(double t) {
  auto msg = std::make_shared<automsgs::msgs::map_msgs::GridCells>();
  StampHeader(msg->mutable_header());
  msg->set_cell_width(0.2f);
  msg->set_cell_height(0.2f);
  for (int i = 0; i < 20; ++i) {
    const double a = t * 0.5 + i * 0.3;
    auto* p = msg->add_cells();
    p->set_x(std::cos(a) * 2.0);
    p->set_y(std::sin(a) * 2.0);
    p->set_z(0.0);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PointStamped> MakePoint(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::PointStamped>();
  StampHeader(msg->mutable_header());
  msg->mutable_point()->set_x(1.5 * std::cos(t));
  msg->mutable_point()->set_y(1.5 * std::sin(t));
  msg->mutable_point()->set_z(0.3);
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PolygonStamped> MakePolygon(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::PolygonStamped>();
  StampHeader(msg->mutable_header());
  const double r = 1.2 + 0.2 * std::sin(t);
  for (int i = 0; i < 6; ++i) {
    const double a = t * 0.3 + i * M_PI / 3;
    auto* p = msg->mutable_polygon()->add_points();
    p->set_x(r * std::cos(a));
    p->set_y(r * std::sin(a));
    p->set_z(0.0);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::TwistStamped> MakeTwist(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::TwistStamped>();
  StampHeader(msg->mutable_header(), "base_link");
  msg->mutable_twist()->mutable_linear()->set_x(0.5 + 0.3 * std::sin(t));
  msg->mutable_twist()->mutable_angular()->set_z(0.4 * std::cos(t));
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::AccelStamped> MakeAccel(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::AccelStamped>();
  StampHeader(msg->mutable_header(), "base_link");
  msg->mutable_accel()->mutable_linear()->set_x(0.2 * std::sin(t * 1.5));
  msg->mutable_accel()->mutable_linear()->set_z(0.1 * std::cos(t));
  msg->mutable_accel()->mutable_angular()->set_y(0.05 * std::sin(t));
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 20.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/geometry");
  auto w_gc = MakeWriter<automsgs::msgs::map_msgs::GridCells>(
      node, "/fake/grid_cells");
  auto w_pt = MakeWriter<automsgs::msgs::geometry_msgs::PointStamped>(
      node, "/fake/point");
  auto w_poly = MakeWriter<automsgs::msgs::geometry_msgs::PolygonStamped>(
      node, "/fake/polygon");
  auto w_tw = MakeWriter<automsgs::msgs::geometry_msgs::TwistStamped>(
      node, "/fake/twist");
  auto w_ac = MakeWriter<automsgs::msgs::geometry_msgs::AccelStamped>(
      node, "/fake/accel");

  autolink::Rate rate(rate_hz);
  std::cout << "geometry @ " << rate_hz
            << " Hz → /fake/{grid_cells,point,polygon,twist,accel}\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    const double t = NowSec() - t0;
    w_gc->Write(MakeGridCells(t));
    w_pt->Write(MakePoint(t));
    w_poly->Write(MakePolygon(t));
    w_tw->Write(MakeTwist(t));
    w_ac->Write(MakeAccel(t));
    rate.Sleep();
  }
  return 0;
}
