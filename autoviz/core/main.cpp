#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "autonomy/autoviz/core/app.hpp"

namespace {

bool g_shutdown = false;

void SignalHandler(int) {
  g_shutdown = true;
}

std::string DefaultConfigPath() {
  namespace fs = std::filesystem;
  const fs::path cwd = fs::current_path();
  const fs::path rel = fs::path("src") / "autonomy" / "autoviz" / "config" / "autoviz.yaml";
  fs::path base = cwd;
  for (int depth = 0; depth < 10; ++depth) {
    const fs::path candidate = base / rel;
    std::error_code ec;
    if (fs::exists(candidate, ec) && !ec) {
      return candidate.string();
    }
    if (!base.has_parent_path()) {
      break;
    }
    base = base.parent_path();
  }
  return rel.string();
}

}  // namespace

int main(int argc, char** argv) {
  (void)argc;

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  const std::string config_path = DefaultConfigPath();

  autoviz::App app;
  if (!app.Init(config_path)) {
    std::cerr << "Failed to initialize autoviz" << std::endl;
    return 1;
  }

  // Simple cooperative shutdown: when a signal is caught, ask the app to stop.
  int exit_code = 0;
  while (!g_shutdown) {
    exit_code = app.Run();
    break;
  }
  app.RequestShutdown();
  return exit_code;
}

