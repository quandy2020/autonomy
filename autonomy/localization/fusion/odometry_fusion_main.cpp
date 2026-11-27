// Copyright 2026 The Openbot Authors
#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autonomy/common/cli_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/fusion/odometry_fusion_node.hpp"

int main(int argc, char** argv) {
    CLI::App app{"Wheel odometry + IMU planar EKF"};
    autonomy::common::BindCommonOptions(app);
    std::string config = "autonomy/localization/conf/cartographer/wheel_imu_ekf.yaml";
    app.add_option("--config", config, "EKF sensor/noise YAML")->capture_default_str();
    CLI11_PARSE(app, argc, argv);
    using namespace autonomy::localization;
    try {
        const auto options = fusion::LoadOdometryFusionConfig(cartographer::node::ResolveWorkspacePath(config));
        if (!autolink::Init(argv[0])) return 1;
        cartographer::node::RegisterAutolinkShutdownHandlers();
        auto* buffer = autonomy::transform::Buffer::Instance();
        buffer->Init();
        auto node = autolink::CreateNode("wheel_imu_ekf");
        auto fusion = std::make_shared<fusion::OdometryFusionNode>(options, buffer);
        if (!fusion->Start(node)) return 1;
        autolink::WaitForShutdown();
        fusion->Stop(); fusion.reset(); node.reset();
        autolink::Clear();
        return 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << "EKF startup failed: " << e.what();
        return 1;
    }
}
