/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/teleop/teleop_assist_options.hpp"

#include <cstdlib>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy::task::teleop {
namespace {

using ::autonomy::common::ConfigurationFileResolver;
using ::autonomy::common::ConfigurationSearchDirectories;
using ::autonomy::common::LuaParameterDictionary;

PointCloudObstacleFeeder::Options LoadPointCloudOptions(
    LuaParameterDictionary* const dict) {
    PointCloudObstacleFeeder::Options options;
    if (dict->HasKey("cloud_topic")) {
        options.cloud_topic = dict->GetString("cloud_topic");
    }
    if (dict->HasKey("stale_timeout_sec")) {
        options.stale_timeout_sec = dict->GetDouble("stale_timeout_sec");
    }
    return options;
}

TeleopMppiAssist::PathLibraryOptions LoadPathLibraryOptions(
    LuaParameterDictionary* const dict) {
    TeleopMppiAssist::PathLibraryOptions options;
    if (dict->HasKey("num_dirs")) {
        options.num_dirs = static_cast<int>(dict->GetDouble("num_dirs"));
    }
    if (dict->HasKey("num_lengths")) {
        options.num_lengths = static_cast<int>(dict->GetDouble("num_lengths"));
    }
    if (dict->HasKey("max_range")) {
        options.max_range = dict->GetDouble("max_range");
    }
    if (dict->HasKey("ds")) {
        options.ds = dict->GetDouble("ds");
    }
    return options;
}

void ApplyPointCloudFromDict(LuaParameterDictionary* dict,
                             PointCloudObstacleFeeder::Options* out) {
    if (dict->HasKey("point_cloud")) {
        *out = LoadPointCloudOptions(dict->GetDictionary("point_cloud").get());
    } else if (dict->HasKey("rgbd")) {
        *out = LoadPointCloudOptions(dict->GetDictionary("rgbd").get());
    }
}

}  // namespace

TeleopMppiAssist::Options LoadTeleopAssistOptions(
    const std::string& config_directory, const std::string& relative_path) {
    TeleopMppiAssist::Options options;
    options.enabled = false;

    const char* env_path = std::getenv("TELEOP_ASSIST_CONFIG");
    const std::string config_rel =
        (env_path != nullptr && env_path[0] != '\0') ? env_path : relative_path;

    try {
        const auto dirs = ConfigurationSearchDirectories(config_directory);
        auto file_resolver = std::make_unique<ConfigurationFileResolver>(dirs);
        const std::string code =
            file_resolver->GetFileContentOrDie(config_rel);
        LuaParameterDictionary dict(code, std::move(file_resolver));

        if (dict.HasKey("assist_enabled")) {
            options.enabled = dict.GetBool("assist_enabled");
        }
        if (dict.HasKey("global_frame")) {
            options.global_frame = dict.GetString("global_frame");
        }
        if (dict.HasKey("angular_to_dir_gain")) {
            options.angular_to_dir_gain = dict.GetDouble("angular_to_dir_gain");
        }
        if (dict.HasKey("stopped_linear_epsilon")) {
            options.stopped_linear_epsilon =
                dict.GetDouble("stopped_linear_epsilon");
        }
        if (dict.HasKey("in_place_angular_scale")) {
            options.in_place_angular_scale =
                dict.GetDouble("in_place_angular_scale");
        }
        if (dict.HasKey("stale_cloud_timeout_sec")) {
            options.point_cloud.stale_timeout_sec =
                dict.GetDouble("stale_cloud_timeout_sec");
        }
        ApplyPointCloudFromDict(&dict, &options.point_cloud);
        if (dict.HasKey("path_library")) {
            options.path_library = LoadPathLibraryOptions(
                dict.GetDictionary("path_library").get());
        }
    } catch (const std::exception& ex) {
        AINFO << "Teleop assist config not loaded (" << config_rel
              << "): " << ex.what() << " — assist disabled";
        options.enabled = false;
    }

    return options;
}

}  // namespace autonomy::task::teleop
