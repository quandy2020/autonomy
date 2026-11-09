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

#include "navigation/common/configuration_loader.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "autonomy/common/file.hpp"
#include "google/protobuf/text_format.h"

namespace navigation
{
namespace common
{
namespace
{

/** @brief Read a text file without throwing across the public API. */
Status ReadTextFile(const std::string& path, std::string* contents)
{
    if (contents == nullptr)
    {
        return Status::InvalidArgument("contents must not be null");
    }
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        return Status::Unavailable("cannot open file: " + path);
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    if (!stream.good() && !stream.eof())
    {
        return Status::Unavailable("cannot read file: " + path);
    }
    *contents = buffer.str();
    return Status{};
}

}  // namespace

std::string ResolveConfigPath(const std::string& path)
{
    if (path.empty() || std::filesystem::path(path).is_absolute())
    {
        return path;
    }
    const char* root = std::getenv("NAVIGATION_CONF_DIR");
    if (root == nullptr || root[0] == '\0')
    {
        return path;
    }
    const std::string joined =
        autonomy::common::JoinPaths(std::string(root), path);
    const bool cwd_exists = std::filesystem::exists(path);
    const bool joined_exists = std::filesystem::exists(joined);
    if (cwd_exists && !joined_exists)
    {
        return path;
    }
    return joined;
}

Status ParseTextProto(const std::string& text, google::protobuf::Message* message)
{
    if (message == nullptr)
    {
        return Status::InvalidArgument("message must not be null");
    }
    google::protobuf::TextFormat::Parser parser;
    parser.AllowUnknownField(false);
    parser.AllowPartialMessage(false);
    if (!parser.ParseFromString(text, message))
    {
        message->Clear();
        return Status::Configuration("invalid protobuf text configuration");
    }
    return Status{};
}

Status LoadTextProto(const std::string& path, google::protobuf::Message* message)
{
    if (message == nullptr)
    {
        return Status::InvalidArgument("message must not be null");
    }
    const std::string resolved = ResolveConfigPath(path);
    std::string contents;
    const Status status = ReadTextFile(resolved, &contents);
    if (!status.Ok())
    {
        return status;
    }
    const Status parsed = ParseTextProto(contents, message);
    if (!parsed.Ok())
    {
        return Status::Configuration("failed to parse " + resolved + ": " + parsed.message);
    }
    return Status{};
}

}  // namespace common
}  // namespace navigation
