#include "autonomy/localization/atlas/config.hpp"

#include <iostream>
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {

config::config(const std::string& config_file_path)
    : config(YAML::LoadFile(config_file_path), config_file_path) {}

config::config(const YAML::Node& yaml_node, const std::string& config_file_path)
    : config_file_path_(config_file_path), yaml_node_(yaml_node) {
    ADEBUG << "CONSTRUCT: config";

    AINFO << "config file loaded: " << config_file_path_;

    if (yaml_node_["MarkerModel"]) {
        AWARN << "MarkerModel is configured but marker detection is not supported";
    }
}

config::~config() {
    ADEBUG << "DESTRUCT: config";
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    os << cfg.yaml_node_;
    return os;
}

}  // namespace autonomy::localization::atlas
