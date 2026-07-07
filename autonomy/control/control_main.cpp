/*
 * Copyright 2026 The Openbot Authors
 *
 * Standalone control process: local costmap + controller plugins.
 */

#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/system/proto/autonomy_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/transform_server.hpp"

namespace autonomy {
namespace control {
namespace {

using AutonomyOptions = ::autonomy::system::proto::AutonomyOptions;

void InitTransformStack(const AutonomyOptions& options)
{
    auto* tf_buffer = transform::Buffer::Instance();
    if (tf_buffer->Init() != 0) {
        AWARN << "control_main: transform::Buffer::Init returned non-zero";
    }

    if (!options.has_transform_options()) {
        return;
    }

    auto transform_server = std::make_unique<transform::TransformServer>(
        options.transform_options());
    const auto& static_transforms =
        transform_server->GetTransformStampedsData();
    for (const auto& trans : static_transforms.transforms) {
        geometry_msgs::TransformStamped geo_msg;
        geo_msg.header.stamp = static_cast<uint64_t>(trans.header.stamp.sec) *
                                   1000000000ULL +
                               static_cast<uint64_t>(trans.header.stamp.nanosec);
        geo_msg.header.frame_id = trans.header.frame_id;
        geo_msg.child_frame_id = trans.child_frame_id;
        geo_msg.transform.translation.x = trans.transform.translation.x;
        geo_msg.transform.translation.y = trans.transform.translation.y;
        geo_msg.transform.translation.z = trans.transform.translation.z;
        geo_msg.transform.rotation.x = trans.transform.rotation.x;
        geo_msg.transform.rotation.y = trans.transform.rotation.y;
        geo_msg.transform.rotation.z = trans.transform.rotation.z;
        geo_msg.transform.rotation.w = trans.transform.rotation.w;
        tf_buffer->setTransform(geo_msg, "control_main", true);
    }
}

}  // namespace
}  // namespace control
}  // namespace autonomy

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    const auto options = autonomy::system::CreateOptions(
        autonomy::common::FLAGS_configuration_directory,
        autonomy::common::FLAGS_configuration_basename);
    if (!options.has_controller_options()) {
        LOG(ERROR) << "control_main: missing controller options in config";
        return EXIT_FAILURE;
    }

    autonomy::control::InitTransformStack(options);
    auto controller = std::make_shared<autonomy::control::ControllerServer>(
        options.controller_options());
    controller->Start();

    LOG(INFO) << "control_main running (ControllerServer)";
    autolink::WaitForShutdown();

    controller->Shutdown();
    return EXIT_SUCCESS;
}
