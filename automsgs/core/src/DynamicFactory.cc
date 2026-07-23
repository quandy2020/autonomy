/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <automsgs/msgs/DynamicFactory.hh>
#include <automsgs/msgs/InstallationDirectories.hh>

namespace {

constexpr const char* kDescriptorEnv = "AUTOMSGS_DESCRIPTOR_PATH";
#ifdef _WIN32
constexpr char kPathSeparator = ';';
#else
constexpr char kPathSeparator = ':';
#endif

std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> out;
  size_t pos = 0;
  for (;;) {
    size_t next = s.find(delim, pos);
    if (next == std::string::npos) {
      if (pos < s.size())
        out.push_back(s.substr(pos));
      break;
    }
    out.push_back(s.substr(pos, next - pos));
    pos = next + 1;
  }
  return out;
}

bool hasExtension(const std::string& path, const std::string& ext) {
  return path.size() >= ext.size() &&
         path.compare(path.size() - ext.size(), ext.size(), ext) == 0;
}

}  // namespace

namespace automsgs {
namespace msgs {

DynamicFactory::DynamicFactory() {
  const char* env = std::getenv(kDescriptorEnv);
  if (env && env[0] != '\0')
    LoadDescriptors(env);

  std::filesystem::path global =
      std::filesystem::path(getInstallPrefix()) / "share" / "automsgs" / "protos";
  if (std::filesystem::exists(global))
    LoadDescriptors(global.string());
}

void DynamicFactory::LoadDescriptors(const std::string& paths) {
  if (paths.empty())
    return;

  std::vector<std::string> dirs = split(paths, kPathSeparator);

  for (const std::string& dir_or_file : dirs) {
    std::filesystem::path p(dir_or_file);
    if (!std::filesystem::exists(p))
      continue;
    if (std::filesystem::is_directory(p)) {
      for (const auto& entry : std::filesystem::directory_iterator(p)) {
        std::string path_str = entry.path().string();
        if (!hasExtension(path_str, ".desc") && !hasExtension(path_str, ".gz_desc") &&
            !hasExtension(path_str, ".proto") && !hasExtension(path_str, ".proto.bin"))
          continue;
        std::ifstream ifs(path_str);
        if (!ifs)
          continue;
        google::protobuf::FileDescriptorSet fds;
        if (!fds.ParseFromIstream(&ifs))
          continue;
        for (const auto& fd_proto : fds.file()) {
          if (const google::protobuf::FileDescriptor* fd = pool_.BuildFile(fd_proto))
            db_.Add(fd_proto);
        }
      }
    } else {
      std::string path_str = p.string();
      if (!hasExtension(path_str, ".desc") && !hasExtension(path_str, ".gz_desc") &&
          !hasExtension(path_str, ".proto") && !hasExtension(path_str, ".proto.bin"))
        continue;
      std::ifstream ifs(path_str);
      if (!ifs)
        continue;
      google::protobuf::FileDescriptorSet fds;
      if (!fds.ParseFromIstream(&ifs))
        continue;
      for (const auto& fd_proto : fds.file()) {
        if (pool_.BuildFile(fd_proto))
          db_.Add(fd_proto);
      }
    }
  }
}

void DynamicFactory::Types(std::vector<std::string>& types) const {
  types.clear();
  db_.FindAllMessageNames(&types);
}

DynamicFactory::MessagePtr DynamicFactory::New(const std::string& msg_type) {
  auto it = dynamic_msg_map_.find(msg_type);
  if (it != dynamic_msg_map_.end())
    return it->second();

  const google::protobuf::Descriptor* desc = pool_.FindMessageTypeByName(msg_type);
  if (!desc)
    return nullptr;

  const google::protobuf::Message* prototype =
      dynamic_message_factory_.GetPrototype(desc);
  if (!prototype)
    return nullptr;

  FactoryFn f = [this, desc]() -> MessagePtr {
    const google::protobuf::Message* p =
        dynamic_message_factory_.GetPrototype(desc);
    return p ? MessagePtr(p->New()) : nullptr;
  };
  dynamic_msg_map_[msg_type] = f;
  return f();
}

}  // namespace msgs
}  // namespace automsgs
