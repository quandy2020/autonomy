#include "autonomy/transform/common/transform_interface.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
namespace autonomy { namespace transform { namespace common {
proto::TransformOptions CreateOptions(const std::string& conf_file) {
  proto::TransformOptions options;
  const std::string file = conf_file.empty() ? "transform.pb.txt" : conf_file;
  if (!autonomy::common::LoadModuleConf("transform", file, &options)) {
    AWARN << "transform conf not found: " << file;
  }
  // Default extrinsic under transform/conf/
  if (options.extrinsic_file().empty()) {
    std::string yaml;
    if (autonomy::common::ResolveModuleConfPath("transform", "static_transform.yaml", &yaml)) {
      options.set_extrinsic_file(yaml);
    }
  }
  return options;
}
}}}
