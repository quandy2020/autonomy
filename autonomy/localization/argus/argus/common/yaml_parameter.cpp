/*
 * Copyright 2026 The Openbot Authors
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

#include "common/yaml_parameter.hpp"

#include "common/logging.hpp"

namespace argus {
namespace common {

YamlParameter::YamlParameter(const std::string& path) { Open(path); }

bool YamlParameter::Open(const std::string& path) {
  Close();
  path_ = path;
  storage_.open(path, cv::FileStorage::READ);
  if (!storage_.isOpened()) {
    ARGUS_ERROR << "Cannot open YAML: " << path;
    return false;
  }
  return true;
}

void YamlParameter::Close() {
  if (storage_.isOpened()) {
    storage_.release();
  }
  path_.clear();
}

cv::FileNode YamlParameter::Resolve(const std::string& key) const {
  if (!storage_.isOpened() || key.empty()) {
    return cv::FileNode();
  }
  cv::FileNode node = storage_.root();
  std::size_t start = 0;
  while (start < key.size()) {
    const std::size_t slash = key.find('/', start);
    const std::string part = key.substr(
        start, slash == std::string::npos ? std::string::npos : slash - start);
    if (!part.empty()) {
      node = node[part];
      if (node.empty()) {
        return cv::FileNode();
      }
    }
    if (slash == std::string::npos) {
      break;
    }
    start = slash + 1;
  }
  return node;
}

bool YamlParameter::Has(const std::string& key) const {
  return !Resolve(key).empty();
}

std::vector<double> YamlParameter::GetDoubleVector(
    const std::string& key) const {
  std::vector<double> out;
  cv::FileNode node = Resolve(key);
  if (node.empty()) {
    return out;
  }
  if (node.isSeq()) {
    for (auto it = node.begin(); it != node.end(); ++it) {
      double v = 0.0;
      try {
        (*it) >> v;
      } catch (const cv::Exception&) {
        continue;
      }
      out.push_back(v);
    }
    return out;
  }
  // Single scalar.
  double v = 0.0;
  node >> v;
  out.push_back(v);
  return out;
}

}  // namespace common
}  // namespace argus
