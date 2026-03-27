#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"

#include "autonomy/autoviz/core/app.hpp"

namespace {

autoviz::App* g_app = nullptr;

std::string GuessAutolinkWorkRoot() {
  namespace fs = std::filesystem;
  const fs::path conf_rel = fs::path("conf") / "autolink.pb.conf";

  // Try both:
  // 1) current working directory
  // 2) directory of argv[0] (more robust when launched from elsewhere)
  std::vector<fs::path> start_points;
  start_points.push_back(fs::current_path());
  // argv[0] will be handled by a wrapper to avoid capturing argv here.
  // (This function is kept for backward compatibility.)

  // Look up several directory levels and find:
  //   <repo_root>/src/autonomy/autolink/autolink/conf/autolink.pb.conf
  for (const auto& start : start_points) {
    fs::path base = start;
    for (int depth = 0; depth < 10; ++depth) {
      const fs::path conf =
          base / "src" / "autonomy" / "autolink" / "autolink" / conf_rel;
      std::error_code ec;
      if (fs::exists(conf, ec) && !ec) {
        return conf.parent_path().parent_path().string();  // .../autolink/autolink
      }
      if (!base.has_parent_path()) {
        break;
      }
      base = base.parent_path();
    }
  }
  return "";
}

std::string GuessAutolinkWorkRootFromArgv0(const char* argv0) {
  namespace fs = std::filesystem;
  const fs::path conf_rel = fs::path("conf") / "autolink.pb.conf";
  if (argv0 == nullptr) {
    return "";
  }

  std::error_code ec;
  const fs::path exe = fs::absolute(fs::path(argv0), ec);
  fs::path base = exe.parent_path();
  for (int depth = 0; depth < 15; ++depth) {
    const fs::path conf =
        base / "src" / "autonomy" / "autolink" / "autolink" / conf_rel;
    if (fs::exists(conf, ec) && !ec) {
      return conf.parent_path().parent_path().string();  // .../autolink/autolink
    }
    if (!base.has_parent_path()) {
      break;
    }
    base = base.parent_path();
  }
  return "";
}

void SignalHandler(int) {
  if (g_app != nullptr) {
    g_app->Shutdown();
  }
}

std::string DefaultConfigPath() {
  namespace fs = std::filesystem;
  const fs::path cwd = fs::current_path();
  const fs::path rel = fs::path("src") / "autonomy" / "autoviz" / "config" / "autoviz.pb.conf";
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

  if (std::getenv("AUTOLINK_PATH") == nullptr) {
    const auto guessed_from_argv0 = GuessAutolinkWorkRootFromArgv0(argv[0]);
    const auto guessed = !guessed_from_argv0.empty() ? guessed_from_argv0 : GuessAutolinkWorkRoot();
    if (!guessed.empty()) {
      ::setenv("AUTOLINK_PATH", guessed.c_str(), 0);
    }
  }

  autolink::Init(argv[0]);

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  const std::string config_path = DefaultConfigPath();

  autoviz::App app;
  if (!app.Initialize(config_path)) {
    AERROR << "Failed to initialize autoviz";
    return 1;
  }

  g_app = &app;

  // Blocking run: exits after Shutdown() (e.g. SIGINT/SIGTERM).
  const int exit_code = app.Run();
  g_app = nullptr;
  autolink::WaitForShutdown();
  return exit_code;
}

