/*
 * Copyright 2026 The Openbot Authors
 *
 * Standalone control process: local costmap + controller plugins.
 * Loads module-local ControllerOptions (does not require AUTONOMY_BUILD_SYSTEM).
 */

#include <cstdlib>
#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/transform/autolink_tf_listener.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/common/transform_interface.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/proto/transform_options.pb.h"
#include "autonomy/transform/transform_server.hpp"

namespace autonomy {
namespace control {
namespace {

using ControllerOptions = ::autonomy::control::proto::ControllerOptions;
using TransformOptions = ::autonomy::transform::proto::TransformOptions;

std::unique_ptr<transform::TransformServer> InitTransformStack(
    const TransformOptions& options)
{
    auto* tf_buffer = transform::Buffer::Instance();
    if (tf_buffer->Init() != 0) {
        AWARN << "control_main: transform::Buffer::Init returned non-zero";
    }

    if (options.extrinsic_file().empty()) {
        return nullptr;
    }

    auto transform_server = std::make_unique<transform::TransformServer>(options);
    const auto& static_transforms =
        transform_server->GetTransformStampedsData();
    for (const auto& trans : static_transforms.transforms()) {
        geometry_msgs::TransformStamped geo_msg;
        geo_msg.header.stamp = static_cast<uint64_t>(trans.header().stamp().sec()) *
                                   1000000000ULL +
                               static_cast<uint64_t>(trans.header().stamp().nanosec());
        geo_msg.header.frame_id = trans.header().frame_id();
        geo_msg.child_frame_id = trans.child_frame_id();
        geo_msg.transform.translation.x = trans.transform().translation().x();
        geo_msg.transform.translation.y = trans.transform().translation().y();
        geo_msg.transform.translation.z = trans.transform().translation().z();
        geo_msg.transform.rotation.x = trans.transform().rotation().x();
        geo_msg.transform.rotation.y = trans.transform().rotation().y();
        geo_msg.transform.rotation.z = trans.transform().rotation().z();
        geo_msg.transform.rotation.w = trans.transform().rotation().w();
        tf_buffer->setTransform(geo_msg, "control_main", true);
    }
    return transform_server;
}

std::string ResolveControllerConfFile()
{
    const std::string& conf = ::autonomy::common::FLAGS_conf;
    if (conf.empty() || conf == "autonomy.pb.txt" ||
        conf == "exploration.pb.txt") {
        return "controller.pb.txt";
    }
    return conf;
}

}  // namespace
}  // namespace control
}  // namespace autonomy

int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    const std::string conf_file =
        autonomy::control::ResolveControllerConfFile();
    autonomy::control::proto::ControllerOptions controller_options;
    if (!autonomy::common::LoadModuleConf("control", conf_file,
                                          &controller_options)) {
        LOG(ERROR) << "control_main: failed to load control conf: " << conf_file;
        return EXIT_FAILURE;
    }

    const auto transform_options =
        autonomy::transform::common::CreateOptions();
    auto static_tf =
        autonomy::control::InitTransformStack(transform_options);

    auto tf_node = autolink::CreateNode("control_tf");
    auto tf_listener =
        std::make_shared<autonomy::transform::AutolinkTfListener>();
    if (!tf_listener->Start(tf_node)) {
        LOG(WARNING) << "control_main: AutolinkTfListener start failed "
                        "(map←base_link may be unavailable)";
        tf_listener.reset();
        tf_node.reset();
    }

    auto controller = std::make_shared<autonomy::control::ControllerServer>(
        controller_options);
    controller->Start();

    LOG(INFO) << "control_main running (ControllerServer)";
    autolink::WaitForShutdown();

    controller->Shutdown();
    if (tf_listener) {
        tf_listener->Stop();
    }
    static_tf.reset();
    return EXIT_SUCCESS;
}
