/*
 * Copyright 2026 The Openbot Authors
 *
 * Standalone perception process: RGB-D exploration via PerceptionServer.
 */

#include <cstdlib>
#include <memory>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/perception/perception_server.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/system/proto/autonomy_options.pb.h"
#include "autonomy/transform/autolink_tf_listener.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/transform_server.hpp"

namespace autonomy {
namespace perception {
namespace {

using AutonomyOptions = ::autonomy::system::proto::AutonomyOptions;

std::unique_ptr<transform::TransformServer> InitTransformStack(
    const AutonomyOptions& options)
{
    auto* tf_buffer = transform::Buffer::Instance();
    if (tf_buffer->Init() != 0) {
        AWARN << "perception_main: transform::Buffer::Init returned non-zero";
    }

    if (!options.has_transform_options() ||
        options.transform_options().extrinsic_file().empty()) {
        return nullptr;
    }

    auto transform_server = std::make_unique<transform::TransformServer>(
        options.transform_options());
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
        tf_buffer->setTransform(geo_msg, "perception_main", true);
    }
    return transform_server;
}

}  // namespace
}  // namespace perception
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
    if (!options.has_perception_options()) {
        LOG(ERROR) << "perception_main: missing perception options in config";
        return EXIT_FAILURE;
    }

    auto static_tf = autonomy::perception::InitTransformStack(options);

    auto tf_node = autolink::CreateNode("perception_tf");
    auto tf_listener =
        std::make_shared<autonomy::transform::AutolinkTfListener>();
    if (!tf_listener->Start(tf_node)) {
        LOG(WARNING) << "perception_main: AutolinkTfListener start failed "
                        "(map<-camera TF may be unavailable)";
        tf_listener.reset();
        tf_node.reset();
    }

    auto tf_buffer = std::shared_ptr<autonomy::transform::Buffer>(
        autonomy::transform::Buffer::Instance(),
        [](autonomy::transform::Buffer*) {});

    auto server = std::make_shared<autonomy::perception::PerceptionServer>(
        options.perception_options());
    server->SetConfigDirectory(
        autonomy::common::FLAGS_configuration_directory);
    server->SetTransformBuffer(tf_buffer);
    server->Start();

    if (!options.perception_options().enabled()) {
        LOG(WARNING) << "perception_main: perception disabled in config "
                        "(set enabled=true or use exploration_autonomy.lua)";
    } else if (!options.perception_options().enable_rgbd_exploration()) {
        LOG(WARNING) << "perception_main: RGB-D exploration disabled in config";
    } else {
        LOG(INFO) << "perception_main running (PerceptionServer + exploration)";
    }

    autolink::WaitForShutdown();

    server->Shutdown();
    if (tf_listener) {
        tf_listener->Stop();
    }
    static_tf.reset();
    return EXIT_SUCCESS;
}
