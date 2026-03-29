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
#include "autolink/service_discovery/specific_manager/service_manager.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autolink/state.hpp"

namespace {

void PrintUsage() {
    std::cout
        << "autolink_service is a command-line tool for printing information "
           "about CyberRT Services.\n\n"
        << "Commands:\n"
        << "\tautolink_service list\tlist active services\n"
        << "\tautolink_service info\tprint information about active service\n\n"
        << "Type autolink_service <command> -h for more detailed usage, e.g. "
           "'autolink_service info -h'\n";
}

std::vector<std::string> GetServices(uint8_t sleep_s = 2) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    std::vector<autolink::proto::RoleAttributes> servers;
    topology->service_manager()->GetServers(&servers);
    std::vector<std::string> names;
    for (const auto& s : servers) {
        names.push_back(s.service_name());
    }
    std::sort(names.begin(), names.end());
    return names;
}

bool GetServiceAttr(const std::string& service_name, uint8_t sleep_s,
                    autolink::proto::RoleAttributes* out) {
    if (!out)
        return false;
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(sleep_s);
    if (!topology->service_manager()->HasService(service_name)) {
        std::cerr << "no service: " << service_name << std::endl;
        return false;
    }
    std::vector<autolink::proto::RoleAttributes> servers;
    topology->service_manager()->GetServers(&servers);
    for (const auto& attr : servers) {
        if (attr.service_name() == service_name) {
            out->CopyFrom(attr);
            return true;
        }
    }
    return false;
}

void PrintServiceInfo(const std::string& service_name, uint8_t sleep_s = 2) {
    autolink::proto::RoleAttributes attr;
    if (!GetServiceAttr(service_name, sleep_s, &attr)) {
        return;
    }
    if (attr.service_name() != service_name) {
        std::cerr << "RoleAttributes service_name mismatch" << std::endl;
        return;
    }
    std::cout << attr.service_name() << std::endl;
    std::cout << "\tprocessid\t" << attr.process_id() << std::endl;
    std::cout << "\tnodename\t" << attr.node_name() << std::endl;
    std::cout << "\thostname\t" << attr.host_name() << std::endl;
    std::cout << std::endl;
}

void CmdList() {
    std::vector<std::string> services = GetServices(2);
    std::cout << "The number of services is: " << services.size() << std::endl;
    for (const auto& name : services) {
        std::cout << name << std::endl;
    }
}

void CmdInfo(bool all_services, const std::vector<std::string>& service_names) {
    if (all_services) {
        std::vector<std::string> services = GetServices(2);
        for (const auto& name : services) {
            PrintServiceInfo(name, 0);
        }
    } else {
        for (const auto& name : service_names) {
            PrintServiceInfo(name, 2);
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
            std::cerr << "usage: autolink_service list" << std::endl;
            ret = 64;
        } else {
            CmdList();
        }
    } else if (command == "info") {
        bool all_services = false;
        std::vector<std::string> service_names;
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
                all_services = true;
            if (c == 'h') {
                std::cout << "usage: autolink_service info servicename\n"
                          << "       autolink_service info -a|--all\n";
                return 0;
            }
        }
        while (optind < info_argc) {
            service_names.push_back(info_argv[optind]);
            optind++;
        }
        if (all_services && !service_names.empty()) {
            std::cerr
                << "\"-a/--all\" option is expected to run w/o service name(s)"
                << std::endl;
            ret = 64;
        } else if (!all_services && service_names.empty()) {
            std::cerr << "servicename must be specified" << std::endl;
            ret = 64;
        } else if (service_names.size() > 1) {
            std::cerr << "you may only specify one service name" << std::endl;
            ret = 64;
        } else if (ret == 0) {
            CmdInfo(all_services, service_names);
        }
    } else {
        PrintUsage();
        ret = 64;
    }

    autolink::Clear();
    return ret;
}
