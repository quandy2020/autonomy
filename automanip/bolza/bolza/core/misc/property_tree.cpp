/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#include <bolza/core/misc/property_tree.hpp>

#include <cctype>
#include <fstream>
#include <stack>

namespace bolza {
namespace property_tree {
namespace {

std::string trim(const std::string& s) {
  size_t begin = 0;
  while (begin < s.size() && std::isspace(static_cast<unsigned char>(s[begin])) != 0) {
    ++begin;
  }
  size_t end = s.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1])) != 0) {
    --end;
  }
  return s.substr(begin, end - begin);
}

std::string stripCommentOutsideQuotes(const std::string& line) {
  bool inQuotes = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      inQuotes = !inQuotes;
    } else if (!inQuotes && c == '/' && i + 1 < line.size() && line[i + 1] == '/') {
      return line.substr(0, i);
    } else if (c == ';' && !inQuotes) {
      return line.substr(0, i);
    }
  }
  return line;
}

std::string parseQuotedValue(const std::string& input, size_t& pos) {
  std::string value;
  ++pos;  // skip opening quote
  while (pos < input.size()) {
    const char c = input[pos++];
    if (c == '"') {
      return value;
    }
    if (c == '\\' && pos < input.size()) {
      value.push_back(input[pos++]);
    } else {
      value.push_back(c);
    }
  }
  throw std::runtime_error("bolza::property_tree: unterminated quoted string in INFO file");
}

bool parseKeyValue(const std::string& line, std::string& key, std::string& value, bool& opensBlock) {
  opensBlock = false;
  const std::string stripped = trim(stripCommentOutsideQuotes(line));
  if (stripped.empty()) {
    return false;
  }
  if (stripped == "}") {
    throw std::runtime_error("bolza::property_tree: unexpected '}' outside of a block");
  }

  size_t pos = 0;
  while (pos < stripped.size() && std::isspace(static_cast<unsigned char>(stripped[pos])) != 0) {
    ++pos;
  }
  if (pos >= stripped.size()) {
    return false;
  }

  const size_t keyStart = pos;
  while (pos < stripped.size() && std::isspace(static_cast<unsigned char>(stripped[pos])) == 0 && stripped[pos] != '{') {
    ++pos;
  }
  key = stripped.substr(keyStart, pos - keyStart);
  if (key.empty()) {
    return false;
  }

  while (pos < stripped.size() && std::isspace(static_cast<unsigned char>(stripped[pos])) != 0) {
    ++pos;
  }

  if (pos < stripped.size() && stripped[pos] == '{') {
    opensBlock = true;
    ++pos;
    while (pos < stripped.size() && std::isspace(static_cast<unsigned char>(stripped[pos])) != 0) {
      ++pos;
    }
    if (pos != stripped.size()) {
      throw std::runtime_error("bolza::property_tree: trailing tokens after '{' on line: " + line);
    }
    return true;
  }

  if (pos >= stripped.size()) {
    value.clear();
    return true;
  }

  if (stripped[pos] == '"') {
    value = parseQuotedValue(stripped, pos);
  } else {
    const size_t valueStart = pos;
    pos = stripped.size();
    value = trim(stripped.substr(valueStart));
  }

  while (pos < stripped.size() && std::isspace(static_cast<unsigned char>(stripped[pos])) != 0) {
    ++pos;
  }
  if (pos != stripped.size()) {
    throw std::runtime_error("bolza::property_tree: trailing tokens on line: " + line);
  }

  return true;
}

}  // namespace

std::vector<std::string> ptree::SplitPath(const std::string& path) {
  std::vector<std::string> parts;
  std::string current;
  int parenDepth = 0;
  int bracketDepth = 0;
  for (char c : path) {
    if (c == '.' && parenDepth == 0 && bracketDepth == 0) {
      if (!current.empty()) {
        parts.push_back(current);
      }
      current.clear();
    } else {
      if (c == '(') {
        ++parenDepth;
      } else if (c == ')') {
        --parenDepth;
      } else if (c == '[') {
        ++bracketDepth;
      } else if (c == ']') {
        --bracketDepth;
      }
      current.push_back(c);
    }
  }
  if (!current.empty()) {
    parts.push_back(current);
  }
  return parts;
}

const std::string* ptree::ResolveData(const std::string& path) const {
  const Node* node = &root_;
  for (const auto& part : SplitPath(path)) {
    const auto it = node->children.find(part);
    if (it == node->children.end() || it->second == nullptr) {
      return nullptr;
    }
    node = it->second.get();
  }
  if (!node->has_data) {
    return nullptr;
  }
  return &node->data;
}

void read_info(const std::string& filename, ptree& pt) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("bolza::property_tree: cannot open INFO file: \"" + filename + "\"");
  }

  pt.root_ = ptree::Node{};
  std::stack<ptree::Node*> scopeStack;
  scopeStack.push(&pt.root_);
  std::string pendingBlockKey;

  std::string line;
  while (std::getline(file, line)) {
    const std::string stripped = trim(stripCommentOutsideQuotes(line));
    if (stripped.empty()) {
      continue;
    }
    if (stripped == "{") {
      if (pendingBlockKey.empty()) {
        throw std::runtime_error("bolza::property_tree: unexpected '{' in INFO file: \"" + filename + "\"");
      }
      auto& block = scopeStack.top()->children[pendingBlockKey];
      if (block == nullptr) {
        block = std::make_unique<ptree::Node>();
      }
      scopeStack.push(block.get());
      pendingBlockKey.clear();
      continue;
    }
    if (stripped == "}") {
      if (scopeStack.size() <= 1) {
        throw std::runtime_error("bolza::property_tree: unexpected '}' in INFO file: \"" + filename + "\"");
      }
      scopeStack.pop();
      pendingBlockKey.clear();
      continue;
    }

    std::string key;
    std::string value;
    bool opensBlock = false;
    if (!parseKeyValue(line, key, value, opensBlock)) {
      continue;
    }

    pendingBlockKey.clear();
    auto& slot = scopeStack.top()->children[key];
    if (slot == nullptr) {
      slot = std::make_unique<ptree::Node>();
    }
    ptree::Node& child = *slot;
    if (opensBlock) {
      scopeStack.push(&child);
    } else if (value.empty()) {
      pendingBlockKey = key;
    } else {
      child.data = value;
      child.has_data = true;
    }
  }

  if (scopeStack.size() != 1) {
    throw std::runtime_error("bolza::property_tree: unclosed block in INFO file: \"" + filename + "\"");
  }
  if (!pendingBlockKey.empty()) {
    throw std::runtime_error("bolza::property_tree: key without value or block in INFO file: \"" + filename + "\"");
  }
}

}  // namespace property_tree
}  // namespace bolza
