/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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


/**
 * @file
 * @brief Some string util functions.
 */

#pragma once

#include <glog/logging.h>
#include <memory>
#include <string>

#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"

/**
 * @namespace autonomy::common
 * @brief autonomy::common
 */
namespace autonomy {
namespace common {

template <typename T, typename std::enable_if<
                          std::is_base_of<google::protobuf::Message, T>::value,
                          int>::type = 0>
static void FillHeader(const std::string& module_name, T* msg) {
  static std::atomic<uint64_t> sequence_num = {0};
  auto* header = msg->mutable_header();
  // double timestamp = ::apollo::cyber::Clock::NowInSeconds();
  // header->mutable_stamp()->set_sec(timestamp);
}

// template <typename T, typename std::enable_if<
//                           std::is_base_of<google::protobuf::Message, T>::value,
//                           int>::type = 0>
// bool DumpMessage(const std::shared_ptr<T>& msg,
//                  const std::string& dump_dir = "/tmp") {
//   if (!msg) {
//     LOG(WARNING) << "Message to be dumped is nullptr!";
//   }

//   auto type_name = T::descriptor()->full_name();
//   std::string dump_path = dump_dir + "/" + type_name;
//   if (!::cyber::common::DirectoryExists(dump_path)) {
//     if (!::cyber::common::EnsureDirectory(dump_path)) {
//       AERROR << "Cannot enable dumping for '" << type_name
//              << "' because the path " << dump_path
//              << " cannot be created or is not a directory.";
//       return false;
//     }
//   }

//   auto sequence_num = msg->header().sequence_num();
//   return ::cyber::common::SetProtoToASCIIFile(
//       *msg, absl::StrCat(dump_path, "/", sequence_num, ".pb.txt"));

//   return false;
// }

inline size_t MessageFingerprint(const google::protobuf::Message& message) {
  static std::hash<std::string> hash_fn;
  std::string proto_bytes;
  message.SerializeToString(&proto_bytes);
  return hash_fn(proto_bytes);
}

}  // namespace common
}  // namespace autonomy
