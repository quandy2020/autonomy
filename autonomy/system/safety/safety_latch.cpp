/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/safety/safety_latch.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace autonomy {
namespace system {
namespace safety {
namespace {

namespace fs = std::filesystem;

std::string RuntimeRoot() {
    if (const char* xdg = std::getenv("XDG_RUNTIME_DIR");
        xdg != nullptr && xdg[0] != '\0') {
        return std::string(xdg) + "/autonomy";
    }
    if (const char* home = std::getenv("HOME");
        home != nullptr && home[0] != '\0') {
        return std::string(home) + "/.autonomy/runtime";
    }
    return "/tmp/autonomy";
}

}  // namespace

std::string SafetyLatch::DefaultPath() {
    if (const char* override_path = std::getenv("AUTONOMY_SAFETY_LATCH_PATH");
        override_path != nullptr && override_path[0] != '\0') {
        return override_path;
    }
    return RuntimeRoot() + "/safety_latch";
}

SafetyLatch::SafetyLatch(std::string path) : path_(std::move(path)) {}

bool SafetyLatch::IsLatched() const {
    std::ifstream in(path_);
    if (!in) {
        return false;
    }
    std::string line;
    if (!std::getline(in, line)) {
        return false;
    }
    return !line.empty() && line[0] == '1';
}

std::string SafetyLatch::Reason() const {
    std::ifstream in(path_);
    if (!in) {
        return {};
    }
    std::string line;
    std::getline(in, line);  // latched flag
    std::string reason;
    std::getline(in, reason);
    return reason;
}

bool SafetyLatch::SetLatched(bool latched, const std::string& reason) {
    try {
        fs::create_directories(fs::path(path_).parent_path());
    } catch (...) {
        return false;
    }
    std::ofstream out(path_, std::ios::trunc);
    if (!out) {
        return false;
    }
    out << (latched ? '1' : '0') << '\n' << reason << '\n';
    return static_cast<bool>(out);
}

}  // namespace safety
}  // namespace system
}  // namespace autonomy
