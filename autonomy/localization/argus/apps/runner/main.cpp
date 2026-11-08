/*
 * Copyright 2026 The Openbot Authors
 */
/**
 * @file main.cpp
 * @brief Unified Argus offline runner (config-driven entry).
 *
 * Usage:
 *   argus_runner --config <apps/runner/configs/kitti.lua> [--data <path>]
 *
 * Dataset-specific binaries remain available (offline_kitti, offline_spires_livo, …).
 * This entry prints resolved options and points at the matching binary until a
 * single playback path is fully wired for all datasets.
 */

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <unordered_map>

namespace {

std::string ExtractString(const std::string& text, const std::string& key) {
  const std::regex re(key + R"rx(\s*=\s*"([^"]*)")rx");
  std::smatch m;
  if (std::regex_search(text, m, re) && m.size() >= 2) {
    return m[1].str();
  }
  return {};
}

bool ExtractBool(const std::string& text, const std::string& key, bool def) {
  const std::regex re(key + R"(\s*=\s*(true|false))");
  std::smatch m;
  if (std::regex_search(text, m, re) && m.size() >= 2) {
    return m[1].str() == "true";
  }
  return def;
}

int ExtractInt(const std::string& text, const std::string& key, int def) {
  const std::regex re(key + R"(\s*=\s*(-?\d+))");
  std::smatch m;
  if (std::regex_search(text, m, re) && m.size() >= 2) {
    return std::atoi(m[1].str().c_str());
  }
  return def;
}

void PrintHelp() {
  std::cerr
      << "Usage: argus_runner --config <file.lua> [--data <path>]\n"
      << "Configs:\n"
      << "  apps/runner/configs/kitti.lua\n"
      << "  apps/runner/configs/newer_college.lua\n"
      << "  apps/runner/configs/tum_rgbd.lua\n"
      << "Primary binary: offline_voxel_slam\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  std::string data_override;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    }
    if ((a == "--config" || a == "-c") && i + 1 < argc) {
      config_path = argv[++i];
      continue;
    }
    if ((a == "--data" || a == "-d") && i + 1 < argc) {
      data_override = argv[++i];
      continue;
    }
  }
  if (config_path.empty()) {
    PrintHelp();
    return 2;
  }

  std::ifstream in(config_path);
  if (!in) {
    std::cerr << "failed to open config: " << config_path << "\n";
    return 1;
  }
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());

  const std::string dataset = ExtractString(text, "dataset");
  std::string data_path = ExtractString(text, "data_path");
  if (!data_override.empty()) {
    data_path = data_override;
  }
  const std::string config_yaml = ExtractString(text, "config_yaml");
  const int max_frames = ExtractInt(text, "max_frames", 0);
  const bool filter_only = ExtractBool(text, "filter_only", false);

  std::cout << "[argus_runner] config=" << config_path << "\n"
            << "  dataset=" << dataset << "\n"
            << "  data_path=" << data_path << "\n"
            << "  config_yaml=" << config_yaml << "\n"
            << "  max_frames=" << max_frames << "\n"
            << "  filter_only=" << (filter_only ? "true" : "false") << "\n";

  const std::unordered_map<std::string, std::string> hint = {
      {"kitti", "offline_kitti --kitti <data> [--config ...]"},
      {"newer_college",
       "verify_oxford_datasets --ncd <data> | offline_spires_livo"},
      {"tum_rgbd", "argus_benchmark <estimate.tum> <gt.tum>"},
  };
  const auto it = hint.find(dataset);
  if (it != hint.end()) {
    std::cout << "  next: " << it->second << "\n";
  } else if (!dataset.empty()) {
    std::cout << "  next: unknown dataset; see --help\n";
  }

  if (!data_path.empty() && !std::filesystem::exists(data_path)) {
    std::cerr << "warning: data_path does not exist: " << data_path << "\n";
    return 1;
  }
  return 0;
}
