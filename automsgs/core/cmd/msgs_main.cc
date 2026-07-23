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
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <automsgs/msgs/config.hh>
#include <automsgs/msgs/InstallationDirectories.hh>

#include <filesystem>
namespace fs = std::filesystem;

static const char* kHelp =
  "Print information about automsgs message types.\n\n"
  "Options:\n"
  "  -l, --list       List all installed .proto message types.\n"
  "  -i, --info TYPE  Print the contents of the .proto file for TYPE\n"
  "                   (e.g. automsgs.msgs.geometry_msgs.Pose).\n"
  "  -h, --help       Show this help.\n"
  "  -v, --version    Print version and exit.\n";

// PascalCase/CamelCase to snake_case (e.g. Pose -> pose, PointField -> point_field)
static std::string toSnake(const std::string& s) {
  std::string out;
  for (size_t i = 0; i < s.size(); ++i) {
    if (std::isupper(static_cast<unsigned char>(s[i])) && i > 0)
      out += '_';
    out += static_cast<char>(std::tolower(static_cast<unsigned char>(s[i])));
  }
  return out;
}

// snake_case to PascalCase (e.g. pose -> Pose, point_field -> PointField)
static std::string toPascal(const std::string& s) {
  std::string out;
  bool cap = true;
  for (char c : s) {
    if (c == '_') {
      cap = true;
    } else {
      out += cap ? static_cast<char>(std::toupper(static_cast<unsigned char>(c))) : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      cap = false;
    }
  }
  return out;
}

static std::string typeToPath(const std::string& type) {
  // automsgs.msgs.geometry_msgs.Pose -> msgs/geometry_msgs/pose.proto (snake_case filename)
  if (type.size() < 8 || type.compare(0, 8, "automsgs.") != 0)
    return {};
  std::string path = type.substr(8);
  size_t last_dot = path.rfind('.');
  std::string dir_part = (last_dot != std::string::npos) ? path.substr(0, last_dot) : std::string();
  std::string name_part = (last_dot != std::string::npos) ? path.substr(last_dot + 1) : path;
  for (size_t i = 0; i < dir_part.size(); ++i)
    if (dir_part[i] == '.') dir_part[i] = '/';
  return dir_part.empty() ? (toSnake(name_part) + ".proto") : (dir_part + "/" + toSnake(name_part) + ".proto");
}

static std::string pathToType(const std::string& path) {
  // msgs/geometry_msgs/pose.proto -> automsgs.msgs.geometry_msgs.Pose (PascalCase type name)
  if (path.size() <= 6 || path.compare(path.size() - 6, 6, ".proto") != 0)
    return {};
  std::string without_ext = path.substr(0, path.size() - 6);
  size_t last_slash = without_ext.rfind('/');
  std::string dir_part = (last_slash != std::string::npos) ? without_ext.substr(0, last_slash) : std::string();
  std::string name_part = (last_slash != std::string::npos) ? without_ext.substr(last_slash + 1) : without_ext;
  for (size_t i = 0; i < dir_part.size(); ++i)
    if (dir_part[i] == '/') dir_part[i] = '.';
  std::string type = "automsgs." + (dir_part.empty() ? "" : dir_part + ".") + toPascal(name_part);
  return type;
}

static void runList(const std::string& prefix) {
  std::string base = prefix + "/share/automsgs/proto";
  if (!fs::is_directory(base)) {
    std::cerr << "Proto directory not found: " << base << "\n";
    return;
  }
  std::vector<std::string> types;
  for (fs::recursive_directory_iterator it(base); it != fs::recursive_directory_iterator(); ++it) {
    if (it->is_regular_file() && it->path().extension() == ".proto") {
      std::string rel = fs::relative(it->path(), base).generic_string();
      std::string t = pathToType(rel);
      if (!t.empty()) types.push_back(t);
    }
  }
  std::sort(types.begin(), types.end());
  for (const auto& t : types)
    std::cout << t << "\n";
}

static void runInfo(const std::string& prefix, const std::string& type) {
  std::string rel = typeToPath(type);
  if (rel.empty()) {
    std::cerr << "Invalid type name (expected e.g. automsgs.msgs.geometry_msgs.Pose): " << type << "\n";
    return;
  }
  std::string path = prefix + "/share/automsgs/proto/" + rel;
  std::ifstream f(path);
  if (!f) {
    std::cerr << "Message type not found: " << type << " (file: " << path << ")\n";
    return;
  }
  std::cout << "Name: " << type << "\n";
  std::cout << "File: " << rel << "\n\n";
  std::string line;
  while (std::getline(f, line))
    std::cout << line << "\n";
}

int main(int argc, char** argv) {
  bool list = false;
  bool version = false;
  bool help = false;
  std::vector<std::string> infoTypes;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-l" || a == "--list")
      list = true;
    else if (a == "-v" || a == "--version")
      version = true;
    else if (a == "-h" || a == "--help")
      help = true;
    else if (a == "-i" || a == "--info") {
      if (i + 1 < argc)
        infoTypes.push_back(argv[++i]);
    } else if (a.compare(0, 2, "-i") == 0 && a.size() > 2)
      infoTypes.push_back(a.substr(2));
    else if (a.compare(0, 7, "--info=") == 0)
      infoTypes.push_back(a.substr(7));
  }

  if (version) {
    std::cout << AUTOMSGS_MSGS_VERSION_FULL << "\n";
    return 0;
  }
  if (help) {
    std::cout << "Usage: automsgs-msgs [options]\n\n" << kHelp;
    return 0;
  }
  if (list) {
    std::string prefix = automsgs::msgs::getInstallPrefix();
    runList(prefix);
    return 0;
  }
  if (!infoTypes.empty()) {
    std::string prefix = automsgs::msgs::getInstallPrefix();
    for (const auto& t : infoTypes)
      runInfo(prefix, t);
    return 0;
  }

  std::cout << "Usage: automsgs-msgs [options]\n\n" << kHelp;
  return 0;
}
