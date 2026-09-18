//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.hpp"

#include <filesystem>
#include <fstream>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "common/options.hpp"
#include "core/g2p5/g2p5.hpp"
#include "core/lio/laser_mapping.hpp"
#include "core/loop_closing/loop_closing.hpp"
#include "core/maps/tiled_map.hpp"

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>(true);
    options_.with_2dvisualization_ = false;
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>(true);
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>(false);

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;
        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);
        if (options_.with_loop_closing_) {
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }
    }

    return true;
}

SlamSystem::~SlamSystem() = default;

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = std::move(map_name);
    running_ = true;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;
    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);
    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);

    if (options_.with_gridmap_ && g2p5_) {
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = static_cast<int>(map.info().width());
        const int height = static_cast<int>(map.info().height());
        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                const int8_t data = map.data(index);
                if (data == 0) {
                    nav_image.at<uchar>(height - 1 - y, x) = 255;
                } else if (data == 100) {
                    nav_image.at<uchar>(height - 1 - y, x) = 0;
                } else {
                    nav_image.at<uchar>(height - 1 - y, x) = 128;
                }
            }
        }
        cv::imwrite(save_path + "/map.pgm", nav_image);

        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;
        }
        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << width;
            emitter << YAML::Key << "height" << YAML::Value << height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info().origin().position().x(),
                                     map.info().origin().position().y(), 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;
            emitter << YAML::EndMap;
            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (!running_) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const CloudPtr& cloud) {
    ProcessLidarFrontend(cloud);
    FlushPendingKeyframe();
}

void SlamSystem::ProcessLidarFrontend(const CloudPtr& cloud) {
    if (!running_) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();
}

void SlamSystem::FlushPendingKeyframe() {
    if (!running_ || lio_ == nullptr) {
        return;
    }

    auto kf = lio_->GetKeyframe();
    if (kf == cur_kf_ || kf == nullptr) {
        return;
    }
    cur_kf_ = kf;

    if (options_.with_loop_closing_ && lc_) {
        lc_->AddKF(cur_kf_);
    }
    if (options_.with_gridmap_ && g2p5_) {
        g2p5_->PushKeyframe(cur_kf_);
    }
}

}  // namespace lightning
