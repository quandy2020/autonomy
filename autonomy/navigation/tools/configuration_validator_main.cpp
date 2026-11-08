/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file configuration_validator_main.cpp
 * @brief Parse every shipped conf text proto into its declared message type.
 */

#include <iostream>
#include <string>

#include "navigation/common/configuration_loader.hpp"
#include "navigation/proto/config.pb.h"
#include "navigation/proto/lifecycle.pb.h"
#include "navigation/proto/navigation.pb.h"
#include "navigation/proto/plugins.pb.h"

namespace
{

template <typename Message>
bool Load(const std::string& path, Message* message)
{
    const auto status = navigation::common::LoadTextProto(path, message);
    if (!status.Ok())
    {
        std::cerr << path << ": " << status.message << "\n";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: configuration_validator <conf-dir>\n";
        return 2;
    }
    const std::string root = argv[1];
    bool ok = true;

    navigation::config::NavigationConfig navigation;
    ok = Load(root + "/navigation.pb.txt", &navigation) && ok;
    navigation::config::SafetyConfig safety;
    ok = Load(root + "/safety.pb.txt", &safety) && ok;
    navigation::config::GuardianConfig guardian;
    ok = Load(root + "/guardian.pb.txt", &guardian) && ok;

    for (const char* name : {"differential_drive", "omnidirectional", "ackermann", "legged"})
    {
        navigation::config::PlatformProfile profile;
        ok = Load(root + "/platform/" + name + ".pb.txt", &profile) && ok;
    }

    navigation::config::ModelManifest manifest;
    ok = Load(root + "/models/vntpa.pb.txt", &manifest) && ok;
    navigation::config::ModelManifest defm;
    ok = Load(root + "/models/defm.pb.txt", &defm) && ok;

    return ok ? 0 : 1;
}
