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
#include <vector>

#include "autolink/init.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/specific_manager/channel_manager.hpp"
#include "autolink/service_discovery/specific_manager/node_manager.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autolink/state.hpp"

namespace {

void PrintUsage() {
    std::cout
        << "autolink_node is a command-line tool to show information about "
           "CyberRT Nodes.\n\n"
        << "Commands:\n"
        << "\tautolink_node list \tList active nodes.\n"
        << "\tautolink_node info \tPrint node info.\n\n"
        << "Type autolink_node <command> -h for more detailed usage, e.g. "
           "'autolink_node info -h'\n";
}

std::vector<std::string> GetNodes(uint8_t sleep_s = 2) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<autolink::proto::RoleAttributes> nodes;
    topology->node_manager()->GetNodes(&nodes);
    std::vector<std::string> node_names;
    for (const auto& n : nodes) {
        node_names.push_back(n.node_name());
    }
    std::sort(node_names.begin(), node_names.end());
    return node_names;
}

bool GetNodeAttr(const std::string& node_name, uint8_t sleep_s,
                 autolink::proto::RoleAttributes* out) {
    if (!out)
        return false;
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    if (!topology->node_manager()->HasNode(node_name)) {
        std::cerr << "no node named: " << node_name << std::endl;
        return false;
    }
    std::vector<autolink::proto::RoleAttributes> nodes;
    topology->node_manager()->GetNodes(&nodes);
    for (const auto& attr : nodes) {
        if (attr.node_name() == node_name) {
            out->CopyFrom(attr);
            return true;
        }
    }
    return false;
}

std::vector<std::string> GetReadersOfNode(const std::string& node_name,
                                          uint8_t sleep_s = 0) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<std::string> channels;
    if (!topology->node_manager()->HasNode(node_name)) {
        std::cerr << "no node named: " << node_name << std::endl;
        return channels;
    }
    std::vector<autolink::proto::RoleAttributes> readers;
    topology->channel_manager()->GetReadersOfNode(node_name, &readers);
    for (const auto& r : readers) {
        if (r.channel_name() != "param_event") {
            channels.push_back(r.channel_name());
        }
    }
    std::sort(channels.begin(), channels.end());
    return channels;
}

std::vector<std::string> GetWritersOfNode(const std::string& node_name,
                                          uint8_t sleep_s = 0) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<std::string> channels;
    if (!topology->node_manager()->HasNode(node_name)) {
        std::cerr << "no node named: " << node_name << std::endl;
        return channels;
    }
    std::vector<autolink::proto::RoleAttributes> writers;
    topology->channel_manager()->GetWritersOfNode(node_name, &writers);
    for (const auto& w : writers) {
        if (w.channel_name() != "param_event") {
            channels.push_back(w.channel_name());
        }
    }
    std::sort(channels.begin(), channels.end());
    return channels;
}

void PrintNodeInfo(const std::string& node_name, uint8_t sleep_s = 2) {
    autolink::proto::RoleAttributes attr;
    if (!GetNodeAttr(node_name, sleep_s, &attr)) {
        return;
    }
    if (attr.node_name() != node_name) {
        std::cerr << "RoleAttributes node_name mismatch" << std::endl;
        return;
    }
    std::cout << "Node:    \t" << attr.node_name() << std::endl;
    std::cout << "ProcessId: \t" << attr.process_id() << std::endl;
    std::cout << "Hostname:\t" << attr.host_name() << std::endl;
    std::cout << std::endl;

    std::cout << "[Reading Channels]:" << std::endl;
    std::vector<std::string> reading = GetReadersOfNode(node_name, 0);
    for (const auto& ch : reading) {
        std::cout << ch << std::endl;
    }
    std::cout << std::endl;

    std::cout << "[Writing Channels]:" << std::endl;
    std::vector<std::string> writing = GetWritersOfNode(node_name, 0);
    for (const auto& ch : writing) {
        std::cout << ch << std::endl;
    }
    std::cout << std::endl;
}

void CmdList() {
    std::vector<std::string> nodes = GetNodes(2);
    std::cout << "Number of active nodes: " << nodes.size() << std::endl;
    for (const auto& name : nodes) {
        std::cout << name << std::endl;
    }
}

void CmdInfo(bool all_nodes, const std::vector<std::string>& node_names) {
    if (all_nodes) {
        std::vector<std::string> nodes = GetNodes(
            2);  // match Python: get_nodes() then print_node_info(_, 0)
        for (const auto& nodename : nodes) {
            PrintNodeInfo(nodename, 0);
        }
    } else {
        for (const auto& name : node_names) {
            PrintNodeInfo(name, 2);
        }
    }
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
            std::cerr << "usage: autolink_node list" << std::endl;
            ret = 64;
        } else {
            CmdList();
        }
    } else if (command == "info") {
        bool all_nodes = false;
        std::vector<std::string> node_names;
        int info_argc = argc - 2;
        char** info_argv = argv + 2;
        static const struct option info_opts[] = {
            {"all", no_argument, nullptr, 'a'},
            {"help", no_argument, nullptr, 'h'},
            {nullptr, 0, nullptr, 0}};
        optind = 0;
        int c;
        while (info_argc > 0 && (c = getopt_long(info_argc, info_argv, "ah",
                                                 info_opts, nullptr)) != -1) {
            if (c == 'a')
                all_nodes = true;
            if (c == 'h') {
                std::cout << "usage: autolink_node info [OPTION...] [NODE...]\n"
                          << "  -a, --all  display all nodes' info\n";
                return 0;
            }
        }
        while (optind < info_argc) {
            node_names.push_back(info_argv[optind]);
            optind++;
        }
        if (all_nodes && !node_names.empty()) {
            std::cerr
                << "\"-a/--all\" option is expected to run w/o node name(s)"
                << std::endl;
            ret = 64;
        } else if (!all_nodes && node_names.empty()) {
            std::cerr << "No node name provided." << std::endl;
            ret = 64;
        } else if (ret == 0) {
            CmdInfo(all_nodes, node_names);
        }
    } else {
        PrintUsage();
        ret = 64;
    }

    autolink::Clear();
    return ret;
}
