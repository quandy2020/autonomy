#pragma once
#include <string>
#include "autonomy/task/navigation/proto/navigator_options.pb.h"
namespace autonomy { namespace task { namespace navigation {
proto::NavigatorOptions CreateOptions(const std::string& conf_file = "navigator.pb.txt");
}}}
