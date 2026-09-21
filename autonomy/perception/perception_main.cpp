/*
 * Copyright 2026 The Openbot Authors
 *
 * Standalone perception process: RGB-D exploration via PerceptionServer.
 * Loads module-local PerceptionOptions (does not require AUTONOMY_BUILD_SYSTEM).
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
#include "autonomy/perception/perception_server.hpp"
#include "autonomy/perception/proto/perception_options.pb.h"
#include "autonomy/transform/autolink_tf_listener.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/common/transform_interface.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/proto/transform_options.pb.h"
#include "autonomy/transform/transform_server.hpp"

namespace autonomy {
namespace perception {
namespace {

using PerceptionOptions = ::autonomy::perception::proto::PerceptionOptions;
using TransformOptions = ::autonomy::transform::proto::TransformOptions;

std::unique_ptr<transform::TransformServer> InitTransformStack(
    const TransformOptions& options)
{
    auto* tf_buffer = transform::Buffer::Instance();
    if (tf_buffer->Init() != 0) {
        AWARN << "perception_main: transform::Buffer::Init returned non-zero";
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
        tf_buffer->setTransform(geo_msg, "perception_main", true);
    }
    return transform_server;
}

/** Resolve --conf to a perception/*.pb.txt basename. */
std::string ResolvePerceptionConfFile()
{
    const std::string& conf = ::autonomy::common::FLAGS_conf;
    if (conf.empty() || conf == "autonomy.pb.txt") {
        return "perception.pb.txt";
    }
    // Legacy system umbrella used by launch; map to module-local preset.
    if (conf == "exploration.pb.txt") {
        return "perception_exploration.pb.txt";
    }
    return conf;
}

}  // namespace
}  // namespace perception
}  // namespace autonomy

int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    const std::string conf_file =
        autonomy::perception::ResolvePerceptionConfFile();
    autonomy::perception::proto::PerceptionOptions perception_options;
    if (!autonomy::common::LoadModuleConf("perception", conf_file,
                                          &perception_options)) {
        LOG(ERROR) << "perception_main: failed to load perception conf: "
                   << conf_file;
        return EXIT_FAILURE;
    }

    const auto transform_options =
        autonomy::transform::common::CreateOptions();
    auto static_tf =
        autonomy::perception::InitTransformStack(transform_options);

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
        perception_options);
    server->SetConfigDirectory(
        autonomy::common::FLAGS_configuration_directory);
    server->SetTransformBuffer(tf_buffer);
    server->Start();

    if (!perception_options.enabled()) {
        LOG(WARNING) << "perception_main: perception disabled in config "
                        "(set enabled=true or --conf=perception_exploration.pb.txt)";
    } else if (!perception_options.enable_rgbd_exploration()) {
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
