/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <automsgs/msgs/std_msgs/string.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using Step = std::pair<int, const char*>;

const Step kNav[] = {
    {4, "IDLE"},     {3, "PLANNING"}, {6, "NAVIGATING"},
    {2, "RECOVERY"}, {5, "NAVIGATING"}, {3, "IDLE"},
};
const Step kCtrl[] = {
    {5, "IDLE"}, {8, "TRACKING"}, {3, "HOLD"}, {6, "TRACKING"}, {2, "IDLE"},
};
const Step kMode[] = {
    {10, "MANUAL"}, {12, "AUTO"}, {4, "ESTOP"}, {8, "AUTO"},
};

template <size_t N>
std::string Advance(const Step (&seq)[N], int* i, int* hold) {
  const std::string state = seq[*i].second;
  --(*hold);
  if (*hold <= 0) {
    *i = (*i + 1) % static_cast<int>(N);
    *hold = seq[*i].first;
  }
  return state;
}

}  // namespace

using autoviz::examples::MakeWriter;
using autoviz::examples::ParseRate;

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 2.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/state");
  auto w_nav =
      MakeWriter<automsgs::msgs::std_msgs::String>(node, "/fake/state/nav");
  auto w_ctrl =
      MakeWriter<automsgs::msgs::std_msgs::String>(node, "/fake/state/ctrl");
  auto w_mode =
      MakeWriter<automsgs::msgs::std_msgs::String>(node, "/fake/state/mode");

  autolink::Rate rate(rate_hz);
  std::cout << "state @ " << rate_hz
            << " Hz → /fake/state/{nav,ctrl,mode}\n";

  int ni = 0, ci = 0, mi = 0;
  int nh = kNav[0].first, ch = kCtrl[0].first, mh = kMode[0].first;

  while (autolink::OK()) {
    const std::string nav = Advance(kNav, &ni, &nh);
    const std::string ctrl = Advance(kCtrl, &ci, &ch);
    const std::string mode = Advance(kMode, &mi, &mh);

    auto m_nav = std::make_shared<automsgs::msgs::std_msgs::String>();
    m_nav->set_data(nav);
    w_nav->Write(m_nav);

    auto m_ctrl = std::make_shared<automsgs::msgs::std_msgs::String>();
    m_ctrl->set_data(ctrl);
    w_ctrl->Write(m_ctrl);

    auto m_mode = std::make_shared<automsgs::msgs::std_msgs::String>();
    m_mode->set_data(mode);
    w_mode->Write(m_mode);

    rate.Sleep();
  }
  return 0;
}
