/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <getopt.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "autolink/init.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/specific_manager/service_manager.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autolink/state.hpp"

namespace {

const std::string kSendGoalSuffix = "/send_goal";

void PrintUsage() {
    std::cout
        << "autolink_action is a command-line tool for introspecting and "
           "interacting with Autolink actions (ROS2-style).\n\n"
        << "Commands:\n"
        << "\tautolink_action list\t\tlist active action names\n"
        << "\tautolink_action info <name>\tprint action server info\n"
        << "\tautolink_action send_goal <name> [goal]\tsend an action goal "
           "(goal "
           "format depends on action type; use C++/Python client for custom "
           "types)\n\n"
        << "Type autolink_action <command> -h for more detailed usage.\n";
}

// Get action names: services whose name ends with "/send_goal", strip suffix
std::vector<std::string> GetActionNames(uint8_t sleep_s = 2) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<autolink::proto::RoleAttributes> servers;
    topology->service_manager()->GetServers(&servers);
    std::unordered_set<std::string> names_set;
    for (const auto& s : servers) {
        const std::string& svc = s.service_name();
        if (svc.size() > kSendGoalSuffix.size()) {
            size_t pos = svc.size() - kSendGoalSuffix.size();
            if (svc.compare(pos, kSendGoalSuffix.size(), kSendGoalSuffix) ==
                0) {
                names_set.insert(svc.substr(0, pos));
            }
        }
    }
    std::vector<std::string> names(names_set.begin(), names_set.end());
    std::sort(names.begin(), names.end());
    return names;
}

void PrintServerRole(const autolink::proto::RoleAttributes& attr) {
    std::cout << "\tprocessid\t" << attr.process_id() << std::endl;
    std::cout << "\tnodename\t" << attr.node_name() << std::endl;
    std::cout << "\thostname\t" << attr.host_name() << std::endl;
}

void CmdList() {
    std::vector<std::string> actions = GetActionNames(2);
    std::cout << "The number of actions is: " << actions.size() << std::endl;
    for (const auto& name : actions) {
        std::cout << name << std::endl;
    }
}

void CmdInfo(const std::string& action_name) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(2);
    std::string send_goal_service = action_name + kSendGoalSuffix;
    if (!topology->service_manager()->HasService(send_goal_service)) {
        std::cerr << "Action '" << action_name
                  << "' has no server (no service '" << send_goal_service
                  << "')." << std::endl;
        return;
    }
    std::vector<autolink::proto::RoleAttributes> servers;
    topology->service_manager()->GetServers(&servers);
    std::cout << "Action: " << action_name << std::endl;
    bool found = false;
    for (const auto& attr : servers) {
        if (attr.service_name() == send_goal_service) {
            found = true;
            PrintServerRole(attr);
        }
    }
    if (!found) {
        std::cout << "\t(no server role attributes)" << std::endl;
    }
    std::cout << std::endl;
}

void CmdSendGoalUsage() {
    std::cout
        << "usage: autolink_action send_goal <action_name> [goal_payload]\n\n"
        << "  Send a goal to an action server. Goal payload format depends on "
           "the action type.\n"
        << "  For programmatic use with custom Goal types, use "
           "autolink::action::CreateClient<ActionT> in C++ or the Python "
           "action "
           "client.\n";
}

void CmdSendGoal(const std::string& action_name,
                 const std::string& goal_payload) {
    (void)goal_payload;
    std::cout << "Action: " << action_name << std::endl;
    std::cout << "send_goal from CLI is limited: goal type is required at "
                 "compile time. Use C++ action client or Python client to send "
                 "goals for your action type."
              << std::endl;
    CmdSendGoalUsage();
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage();
        return 64;
    }

    const std::string command(argv[1]);

    std::signal(SIGINT, [](int sig) { autolink::OnShutdown(sig); });
    std::signal(SIGTERM, [](int sig) { autolink::OnShutdown(sig); });

    autolink::Init(argv[0]);

    int ret = 0;

    if (command == "list") {
        if (argc > 2) {
            std::cerr << "usage: autolink_action list" << std::endl;
            ret = 64;
        } else {
            CmdList();
        }
    } else if (command == "info") {
        std::string action_name;
        int info_argc = argc - 2;
        char** info_argv = argv + 2;
        static const struct option info_opts[] = {
            {"help", no_argument, nullptr, 'h'}, {nullptr, 0, nullptr, 0}};
        optind = 0;
        int c;
        while (info_argc > 0 && (c = getopt_long(info_argc, info_argv, "h",
                                                 info_opts, nullptr)) != -1) {
            if (c == 'h') {
                std::cout << "usage: autolink_action info <action_name>\n";
                return 0;
            }
        }
        if (optind < info_argc) {
            action_name = info_argv[optind];
            if (optind + 1 < info_argc) {
                std::cerr << "you may only specify one action name"
                          << std::endl;
                ret = 64;
                action_name.clear();
            }
        }
        if (action_name.empty() && ret == 0) {
            std::cerr << "action_name must be specified" << std::endl;
            ret = 64;
        } else if (ret == 0) {
            CmdInfo(action_name);
        }
    } else if (command == "send_goal") {
        std::string action_name;
        std::string goal_payload;
        int sg_argc = argc - 2;
        char** sg_argv = argv + 2;
        optind = 0;
        if (sg_argc > 0 && sg_argv[0][0] != '-') {
            action_name = sg_argv[0];
            if (sg_argc > 1 && sg_argv[1][0] != '-') {
                goal_payload = sg_argv[1];
            }
        }
        if (action_name.empty()) {
            std::cerr << "usage: autolink_action send_goal <action_name> "
                         "[goal_payload]"
                      << std::endl;
            CmdSendGoalUsage();
            ret = 64;
        } else {
            CmdSendGoal(action_name, goal_payload);
        }
    } else {
        PrintUsage();
        ret = 64;
    }

    autolink::Clear();
    return ret;
}
