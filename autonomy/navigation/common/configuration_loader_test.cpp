/* Copyright 2026 The Navigation Authors (navigation) */
#include <filesystem>
#include <fstream>
#include <iostream>

#include "autonomy/common/file.hpp"

#include "navigation/common/configuration_loader.hpp"
#include "navigation/proto/config.pb.h"

int main()
{
    using namespace navigation::common;  // NOLINT

    navigation::config::NavigationConfig configuration;
    if (!ParseTextProto("platform_profile: \"differential_drive\"\n"
                        "scenario { default_scenario: \"destination\" }\n",
                        &configuration)
             .Ok())
    {
        std::cerr << "parse failed\n";
        return 1;
    }
    if (configuration.platform_profile() != "differential_drive" ||
        configuration.scenario().default_scenario() != "destination")
    {
        std::cerr << "parse content failed\n";
        return 1;
    }

    navigation::config::NavigationConfig rejected;
    if (ParseTextProto("unknown_field: 1\n", &rejected).Ok())
    {
        std::cerr << "unknown field accepted\n";
        return 1;
    }

    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "navigation_config_test";
    std::filesystem::create_directories(directory);
    const std::string path =
        autonomy::common::JoinPaths(directory.string(), "navigation.pb.txt");
    std::ofstream stream(path, std::ios::trunc);
    stream << "platform_profile: \"omni\"\n";
    stream.close();
    if (!stream)
    {
        std::cerr << "write failed\n";
        return 1;
    }
    navigation::config::NavigationConfig loaded;
    if (!LoadTextProto(path, &loaded).Ok() || loaded.platform_profile() != "omni")
    {
        std::cerr << "load failed\n";
        return 1;
    }
    std::filesystem::remove_all(directory);
    return 0;
}
