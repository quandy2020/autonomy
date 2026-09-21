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

#include "autonomy/system/monitor/restart_request_watcher.hpp"

#include <dirent.h>
#include <signal.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace system {
namespace monitor {
namespace {

namespace fs = std::filesystem;

bool IsNumericPid(const char* name) {
    if (name == nullptr || *name == '\0') {
        return false;
    }
    for (const char* p = name; *p != '\0'; ++p) {
        if (!std::isdigit(static_cast<unsigned char>(*p))) {
            return false;
        }
    }
    return true;
}

std::string ReadCmdline(const std::string& pid) {
    std::ifstream in("/proc/" + pid + "/cmdline", std::ios::binary);
    if (!in) {
        return {};
    }
    std::string raw((std::istreambuf_iterator<char>(in)),
                    std::istreambuf_iterator<char>());
    for (char& c : raw) {
        if (c == '\0') {
            c = ' ';
        }
    }
    return raw;
}

std::vector<pid_t> FindPidsMatching(const std::string& needle) {
    std::vector<pid_t> out;
    if (needle.empty()) {
        return out;
    }
    DIR* dir = opendir("/proc");
    if (dir == nullptr) {
        return out;
    }
    const pid_t self = getpid();
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (!IsNumericPid(ent->d_name)) {
            continue;
        }
        const pid_t pid = static_cast<pid_t>(std::atoi(ent->d_name));
        if (pid <= 1 || pid == self) {
            continue;
        }
        const std::string cmd = ReadCmdline(ent->d_name);
        if (cmd.find(needle) != std::string::npos) {
            out.push_back(pid);
        }
    }
    closedir(dir);
    return out;
}

}  // namespace

std::string RestartModuleRequestPath() {
    const char* xdg = std::getenv("XDG_RUNTIME_DIR");
    fs::path dir =
        xdg && xdg[0] ? fs::path(xdg) / "autonomy" : fs::path("/tmp/autonomy");
    return (dir / "restart_module.request").string();
}

bool PollRestartModuleRequest() {
    const fs::path path = RestartModuleRequestPath();
    if (!fs::exists(path)) {
        return false;
    }

    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::string module;
    std::string reason;
    std::getline(in, module);
    std::getline(in, reason);
    in.close();

    // Consume request even if module is empty / kill fails — avoid loops.
    std::error_code ec;
    fs::remove(path, ec);

    while (!module.empty() &&
           (module.back() == '\r' || module.back() == ' ')) {
        module.pop_back();
    }
    if (module.empty()) {
        AWARN << "RestartModule: empty module_name in " << path.string();
        return true;
    }

    // Prefer autonomy.<module> binary name; also try raw token.
    std::vector<std::string> needles;
    needles.push_back("autonomy." + module);
    needles.push_back(module);

    int killed = 0;
    for (const auto& needle : needles) {
        for (const pid_t pid : FindPidsMatching(needle)) {
            if (kill(pid, SIGTERM) == 0) {
                ++killed;
                AWARN << "RestartModule: SIGTERM pid=" << pid
                      << " module=" << module << " reason=" << reason;
            } else {
                AWARN << "RestartModule: kill(" << pid
                      << ") failed errno=" << errno;
            }
        }
        if (killed > 0) {
            break;
        }
    }

    if (killed == 0) {
        AWARN << "RestartModule: no process matched module=" << module;
    }
    return true;
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
