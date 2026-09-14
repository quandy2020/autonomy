#include "autonomy/task/navigation/options.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "glog/logging.h"
namespace autonomy { namespace task { namespace navigation {
proto::NavigatorOptions CreateOptions(const std::string& conf_file) {
  proto::NavigatorOptions options;
  const std::string file = conf_file.empty() ? "navigator.pb.txt" : conf_file;
  CHECK(common::LoadModuleConf("task", file, &options));
  return options;
}
}}}
