#pragma once
#include <string>
#include "autonomy/transform/proto/transform_options.pb.h"
namespace autonomy { namespace transform { namespace common {
proto::TransformOptions CreateOptions(const std::string& conf_file = "transform.pb.txt");
}}}
