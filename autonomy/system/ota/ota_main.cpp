/*
 * Copyright 2026 The Openbot Authors
 */

#include <cstdlib>
#include <string>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/system/ota/ota_agent.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/system.pb.h>

int main(int argc, char** argv) {
    CLI::App app{"autonomy.ota"};
    std::string package;
    std::string type;
    bool status = false;
    bool abort = false;
    app.add_option("--package", package, "Local OTA package directory or archive");
    app.add_option("--type", type, "full|delta|empty=auto");
    app.add_flag("--status", status, "Print OTA status and exit");
    app.add_flag("--abort", abort, "Abort in-flight OTA if allowed");
    autonomy::common::ParseOrExit(app, argc, argv);

    autolink::InitLogging(argv[0]);

    auto& agent = autonomy::system::ota::OtaAgent::Shared();
    if (status) {
        const auto st = agent.Status();
        LOG(INFO) << "ota state=" << st.state() << " current=" << st.current_version()
                  << " target=" << st.target_version() << " detail=" << st.detail();
        return EXIT_SUCCESS;
    }
    if (abort) {
        const auto r = agent.Abort("cli");
        LOG(INFO) << r.status().message();
        return r.status().code() == ::automsgs::msgs::status_msgs::OK
                   ? EXIT_SUCCESS
                   : EXIT_FAILURE;
    }
    if (package.empty()) {
        LOG(ERROR) << "need --package= or --status/--abort";
        return EXIT_FAILURE;
    }
    ::automsgs::rpcs::system::StartOtaRequest req;
    req.set_package_uri(package);
    req.set_package_type(type);
    const auto resp = agent.Start(req);
    LOG(INFO) << resp.status().message() << " state=" << resp.state()
              << " detail=" << resp.detail();
    return resp.status().code() == ::automsgs::msgs::status_msgs::OK
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
