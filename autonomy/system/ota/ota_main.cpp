/*
 * Copyright 2026 The Openbot Authors
 */

#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/system/ota/ota_agent.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/system.pb.h>

DEFINE_string(package, "", "Local OTA package directory or archive");
DEFINE_string(type, "", "full|delta|empty=auto");
DEFINE_bool(status, false, "Print OTA status and exit");
DEFINE_bool(abort, false, "Abort in-flight OTA if allowed");

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    autolink::InitLogging(argv[0]);

    auto& agent = autonomy::system::ota::OtaAgent::Shared();
    if (FLAGS_status) {
        const auto st = agent.Status();
        LOG(INFO) << "ota state=" << st.state() << " current=" << st.current_version()
                  << " target=" << st.target_version() << " detail=" << st.detail();
        return EXIT_SUCCESS;
    }
    if (FLAGS_abort) {
        const auto r = agent.Abort("cli");
        LOG(INFO) << r.status().message();
        return r.status().code() == ::automsgs::msgs::status_msgs::OK
                   ? EXIT_SUCCESS
                   : EXIT_FAILURE;
    }
    if (FLAGS_package.empty()) {
        LOG(ERROR) << "need --package= or --status/--abort";
        return EXIT_FAILURE;
    }
    ::automsgs::rpcs::system::StartOtaRequest req;
    req.set_package_uri(FLAGS_package);
    req.set_package_type(FLAGS_type);
    const auto resp = agent.Start(req);
    LOG(INFO) << resp.status().message() << " state=" << resp.state()
              << " detail=" << resp.detail();
    return resp.status().code() == ::automsgs::msgs::status_msgs::OK
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
