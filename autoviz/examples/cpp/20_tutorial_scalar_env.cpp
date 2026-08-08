/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>

#include <automsgs/msgs/sensor_msgs/fluid_pressure.pb.h>
#include <automsgs/msgs/sensor_msgs/illuminance.pb.h>
#include <automsgs/msgs/sensor_msgs/relative_humidity.pb.h>
#include <automsgs/msgs/std_msgs/float64.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 20.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/scalar");
  auto w_lux = MakeWriter<automsgs::msgs::sensor_msgs::Illuminance>(
      node, "/fake/illuminance");
  auto w_p = MakeWriter<automsgs::msgs::sensor_msgs::FluidPressure>(
      node, "/fake/fluid_pressure");
  auto w_h = MakeWriter<automsgs::msgs::sensor_msgs::RelativeHumidity>(
      node, "/fake/relative_humidity");
  auto w_g =
      MakeWriter<automsgs::msgs::std_msgs::Float64>(node, "/fake/gauge");

  autolink::Rate rate(rate_hz);
  std::cout << "scalar @ " << rate_hz
            << " Hz → /fake/{illuminance,fluid_pressure,relative_humidity,"
               "gauge}\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    const double t = NowSec() - t0;

    auto lux = std::make_shared<automsgs::msgs::sensor_msgs::Illuminance>();
    StampHeader(lux->mutable_header());
    lux->set_illuminance(200 + 150 * (0.5 + 0.5 * std::sin(t * 0.4)));
    lux->set_variance(1.0);
    w_lux->Write(lux);

    auto press =
        std::make_shared<automsgs::msgs::sensor_msgs::FluidPressure>();
    StampHeader(press->mutable_header());
    press->set_fluid_pressure(101325 + 200 * std::sin(t * 0.2));
    press->set_variance(10.0);
    w_p->Write(press);

    auto hum =
        std::make_shared<automsgs::msgs::sensor_msgs::RelativeHumidity>();
    StampHeader(hum->mutable_header());
    hum->set_relative_humidity(0.45 + 0.2 * std::sin(t * 0.15));
    hum->set_variance(0.01);
    w_h->Write(hum);

    auto gauge = std::make_shared<automsgs::msgs::std_msgs::Float64>();
    gauge->set_data(50 + 40 * std::sin(t));
    w_g->Write(gauge);

    rate.Sleep();
  }
  return 0;
}
