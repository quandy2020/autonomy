/*
 * Copyright 2019 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <iostream>
#include <vector>

#include "autonomy/common/string_util.hpp"
#include "autonomy/localization/cartographer/common/configuration_file_resolver.hpp"
#include "autonomy/localization/cartographer/common/lua_parameter_dictionary.hpp"
#include "gflags/gflags.hpp"
#include "glog/logging.h"

DEFINE_string(configuration_directories, "",
              "Comma separated list of directories in which configuration files"
              " are searched, the last is always the Cartographer installation"
              " to allow including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(subdictionary, "",
              "Only print a subdictionary referenced by its Lua ID, e.g.: "
              "'--subdictionary trajectory_builder.trajectory_builder_3d'");

namespace cartographer {
namespace common {

namespace {

std::vector<std::string> SplitSkipEmpty(const std::string& str, const char delimiter) {
    std::vector<std::string> parts = autonomy::common::StringSplit(str, std::string(1, delimiter));
    parts.erase(std::remove_if(parts.begin(), parts.end(),
                               [](const std::string& part) { return part.empty(); }),
                parts.end());
    return parts;
}

}  // namespace

std::unique_ptr<LuaParameterDictionary> LoadLuaDictionary(const std::vector<std::string>& configuration_directories,
                                                          const std::string& configuration_basename) {
    auto file_resolver = std::make_unique<ConfigurationFileResolver>(configuration_directories);
    const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
    // We use no reference count because we just want to print the
    // configuration.
    return LuaParameterDictionary::NonReferenceCounted(code, std::move(file_resolver));
}

void PrintSubdictionaryById(LuaParameterDictionary* lua_dictionary, const std::string& subdictionary_id) {
    const std::vector<std::string> subdictionary_keys = SplitSkipEmpty(subdictionary_id, '.');
    CHECK(!subdictionary_keys.empty()) << "Failed to parse 'subdictionary_id'.";
    // Keep a stack to avoid memory glitches due to unique_ptr deletion.
    std::vector<std::unique_ptr<LuaParameterDictionary>> stack;
    for (const auto& key : subdictionary_keys) {
        if (stack.empty()) {
            stack.push_back(lua_dictionary->GetDictionary(key));
            continue;
        }
        stack.push_back(stack.back()->GetDictionary(key));
    }
    std::cout << subdictionary_id << " = " << stack.back()->ToString() << std::endl;
}

}  // namespace common
}  // namespace cartographer

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::SetUsageMessage(
        "Resolves and compiles a Lua configuration and prints it to stdout.\n"
        "The output can be restricted to a subdictionary using the optional "
        "'--subdictionary' parameter, which can be given in Lua syntax.\n"
        "The logs of the configuration file resolver are written to stderr if "
        "'--logtostderr' is given.");
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_configuration_directories.empty() || FLAGS_configuration_basename.empty()) {
        google::ShowUsageWithFlagsRestrict(argv[0], "print_configuration_main");
        return EXIT_FAILURE;
    }

    const std::vector<std::string> configuration_directories = SplitSkipEmpty(FLAGS_configuration_directories, ',');

    auto lua_dictionary =
        ::cartographer::common::LoadLuaDictionary(configuration_directories, FLAGS_configuration_basename);

    if (FLAGS_subdictionary.empty()) {
        std::cout << "return " << lua_dictionary->ToString() << std::endl;
        return EXIT_SUCCESS;
    }
    ::cartographer::common::PrintSubdictionaryById(lua_dictionary.get(), FLAGS_subdictionary);
}
