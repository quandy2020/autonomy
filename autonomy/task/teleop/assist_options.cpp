#include "autonomy/task/teleop/assist_options.hpp"
#include "autonomy/common/logging.hpp"
namespace autonomy::task::teleop {
TeleopMppiAssist::Options LoadTeleopAssistOptions(
    const std::string& /*config_directory*/,
    const std::string& /*relative_path*/) {
  TeleopMppiAssist::Options options;
  options.enabled = false;
  AINFO << "Teleop MPPI assist: Lua conf removed; assist disabled "
           "(enable via future task/conf/teleop_assist.pb.txt)";
  return options;
}
}
