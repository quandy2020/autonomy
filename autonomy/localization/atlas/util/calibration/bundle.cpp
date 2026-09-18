/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/localization/atlas/util/calibration/bundle.hpp"

#include "autonomy/localization/atlas/frontend/eskf/eskf.hpp"
#include "autonomy/localization/atlas/util/yaml.hpp"

#include <fstream>
#include <stdexcept>

#include "glog/logging.h"

namespace autonomy::localization::atlas {
namespace calibration {
namespace {

std::vector<double> AsDoubleVec(const YAML::Node& n) {
    std::vector<double> out;
    if (!n || !n.IsSequence()) {
        return out;
    }
    out.reserve(n.size());
    for (const auto& x : n) {
        out.push_back(x.as<double>(0.0));
    }
    return out;
}

void LoadExtrinsic(const YAML::Node& n, ExtrinsicSE3* e) {
    if (!n || !e) {
        return;
    }
    Mat33_t R = Mat33_t::Identity();
    Vec3_t t = Vec3_t::Zero();
    if (n["T"] && n["T"].IsSequence() && n["T"].size() == 16) {
        const auto v = AsDoubleVec(n["T"]);
        Mat44_t T = Mat44_t::Identity();
        for (int i = 0; i < 16; ++i) {
            T(i / 4, i % 4) = v[static_cast<std::size_t>(i)];
        }
        e->SetT(T);
        return;
    }
    if (n["R"]) {
        ParseMat33(AsDoubleVec(n["R"]), &R);
    }
    if (n["t"]) {
        ParseVec3(AsDoubleVec(n["t"]), &t);
    }
    e->R = R;
    e->t = t;
}

void LoadCamera(const YAML::Node& n, CameraIntrinsics* c) {
    if (!n || !c) {
        return;
    }
    c->model = n["model"].as<std::string>(c->model);
    c->width = n["width"].as<int>(c->width);
    c->height = n["height"].as<int>(c->height);
    c->fx = n["fx"].as<double>(c->fx);
    c->fy = n["fy"].as<double>(c->fy);
    c->cx = n["cx"].as<double>(c->cx);
    c->cy = n["cy"].as<double>(c->cy);
    c->fps = n["fps"].as<double>(c->fps);
    if (n["distortion"] || n["dist_coeffs"] || n["D"]) {
        const auto key = n["distortion"] ? "distortion"
                         : n["dist_coeffs"] ? "dist_coeffs"
                                            : "D";
        c->dist_coeffs = AsDoubleVec(n[key]);
    }
}

void LoadImu(const YAML::Node& n, ImuIntrinsics* imu) {
    if (!n || !imu) {
        return;
    }
    imu->gyro_noise = n["gyro_noise"].as<double>(imu->gyro_noise);
    imu->acc_noise = n["acc_noise"].as<double>(imu->acc_noise);
    imu->gyro_bias_noise =
        n["gyro_bias_noise"].as<double>(imu->gyro_bias_noise);
    imu->acc_bias_noise = n["acc_bias_noise"].as<double>(imu->acc_bias_noise);
    if (n["gravity"]) {
        ParseVec3(AsDoubleVec(n["gravity"]), &imu->gravity);
    }
    if (n["ba0"]) {
        ParseVec3(AsDoubleVec(n["ba0"]), &imu->ba0);
    }
    if (n["bg0"]) {
        ParseVec3(AsDoubleVec(n["bg0"]), &imu->bg0);
    }
}

void LoadLidar(const YAML::Node& n, LidarIntrinsics* lidar) {
    if (!n || !lidar) {
        return;
    }
    lidar->model = n["model"].as<std::string>(lidar->model);
    lidar->min_range = n["min_range"].as<double>(lidar->min_range);
    lidar->max_range = n["max_range"].as<double>(lidar->max_range);
    lidar->scan_period = n["scan_period"].as<double>(lidar->scan_period);
    lidar->range_scale = n["range_scale"].as<double>(lidar->range_scale);
    lidar->range_bias = n["range_bias"].as<double>(lidar->range_bias);
}

YAML::Node ExtrinsicToYaml(const ExtrinsicSE3& e) {
    YAML::Node n;
    n["R"] = std::vector<double>{e.R(0, 0), e.R(0, 1), e.R(0, 2),
                                 e.R(1, 0), e.R(1, 1), e.R(1, 2),
                                 e.R(2, 0), e.R(2, 1), e.R(2, 2)};
    n["t"] = std::vector<double>{e.t.x(), e.t.y(), e.t.z()};
    return n;
}

YAML::Node CameraToYaml(const CameraIntrinsics& c) {
    YAML::Node n;
    n["model"] = c.model;
    n["width"] = c.width;
    n["height"] = c.height;
    n["fx"] = c.fx;
    n["fy"] = c.fy;
    n["cx"] = c.cx;
    n["cy"] = c.cy;
    n["fps"] = c.fps;
    n["distortion"] = c.dist_coeffs;
    return n;
}

}  // namespace

common::Extrinsics CalibrationBundle::ToExtrinsics() const {
    common::Extrinsics e;
    const Mat44_t T_cl = T_cam_lidar_composed();
    e.R_c_l = T_cl.block<3, 3>(0, 0);
    e.t_c_l = T_cl.block<3, 1>(0, 3);
    e.R_b_w = T_base_wheel.R;
    e.t_b_w = T_base_wheel.t;
    e.time_offset_imu = time_offset_imu;
    e.time_offset_lidar = time_offset_lidar;
    e.time_offset_cam = time_offset_cam;
    e.time_offset_wheel = time_offset_wheel;
    return e;
}

void CalibrationBundle::ApplyTo(frontend::LocalEstimator* estimator) const {
    if (!estimator) {
        return;
    }
    estimator->SetT_imu_lidar(T_imu_lidar.T());
    frontend::Eskf::Options opts = estimator->eskf_options();
    opts.gyro_noise = imu.gyro_noise;
    opts.acc_noise = imu.acc_noise;
    opts.gyro_bias_noise = imu.gyro_bias_noise;
    opts.acc_bias_noise = imu.acc_bias_noise;
    estimator->set_eskf_options(opts);
}

CalibrationBundle LoadCalibrationBundle(const YAML::Node& root_in) {
    CalibrationBundle b;
    YAML::Node root = root_in;
    if (root["calibration"]) {
        root = root["calibration"];
    }

    LoadCamera(util::yaml_optional_ref(root, "camera"), &b.camera);
    LoadCamera(util::yaml_optional_ref(root, "camera_right"), &b.camera_right);
    LoadImu(util::yaml_optional_ref(root, "imu"), &b.imu);
    LoadLidar(util::yaml_optional_ref(root, "lidar"), &b.lidar);

    const auto ext = util::yaml_optional_ref(root, "extrinsics");
    if (ext) {
        LoadExtrinsic(ext["T_imu_lidar"], &b.T_imu_lidar);
        // Aliases
        if (ext["T_il"]) {
            LoadExtrinsic(ext["T_il"], &b.T_imu_lidar);
        }
        LoadExtrinsic(ext["T_cam_imu"], &b.T_cam_imu);
        if (ext["T_ci"]) {
            LoadExtrinsic(ext["T_ci"], &b.T_cam_imu);
        }
        if (ext["T_cam_lidar"] || ext["T_cl"]) {
            LoadExtrinsic(ext["T_cam_lidar"] ? ext["T_cam_lidar"] : ext["T_cl"],
                          &b.T_cam_lidar);
            b.has_T_cam_lidar = true;
        }
        // Legacy fusion_default keys
        if (ext["R_c_l"] || ext["t_c_l"]) {
            ExtrinsicSE3 cl;
            if (ext["R_c_l"]) {
                ParseMat33(AsDoubleVec(ext["R_c_l"]), &cl.R);
            }
            if (ext["t_c_l"]) {
                ParseVec3(AsDoubleVec(ext["t_c_l"]), &cl.t);
            }
            b.T_cam_lidar = cl;
            b.has_T_cam_lidar = true;
        }
        if (ext["R_b_w"] || ext["t_b_w"]) {
            if (ext["R_b_w"]) {
                ParseMat33(AsDoubleVec(ext["R_b_w"]), &b.T_base_wheel.R);
            }
            if (ext["t_b_w"]) {
                ParseVec3(AsDoubleVec(ext["t_b_w"]), &b.T_base_wheel.t);
            }
        }
        LoadExtrinsic(ext["T_base_wheel"], &b.T_base_wheel);

        b.time_offset_imu =
            ext["time_offset_imu"].as<double>(b.time_offset_imu);
        b.time_offset_lidar =
            ext["time_offset_lidar"].as<double>(b.time_offset_lidar);
        b.time_offset_cam =
            ext["time_offset_cam"].as<double>(b.time_offset_cam);
        b.time_offset_wheel =
            ext["time_offset_wheel"].as<double>(b.time_offset_wheel);
    }

    return b;
}

CalibrationBundle LoadCalibrationBundle(const std::string& yaml_path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        LOG(ERROR) << "LoadCalibrationBundle: " << yaml_path << ": " << e.what();
        throw;
    }
    LOG(INFO) << "LoadCalibrationBundle: " << yaml_path;
    return LoadCalibrationBundle(root);
}

bool SaveCalibrationBundle(const CalibrationBundle& bundle,
                           const std::string& yaml_path) {
    YAML::Node root;
    root["camera"] = CameraToYaml(bundle.camera);
    if (bundle.camera_right.width > 0) {
        root["camera_right"] = CameraToYaml(bundle.camera_right);
    }
    YAML::Node imu;
    imu["gyro_noise"] = bundle.imu.gyro_noise;
    imu["acc_noise"] = bundle.imu.acc_noise;
    imu["gyro_bias_noise"] = bundle.imu.gyro_bias_noise;
    imu["acc_bias_noise"] = bundle.imu.acc_bias_noise;
    imu["gravity"] = std::vector<double>{bundle.imu.gravity.x(),
                                         bundle.imu.gravity.y(),
                                         bundle.imu.gravity.z()};
    root["imu"] = imu;

    YAML::Node lidar;
    lidar["model"] = bundle.lidar.model;
    lidar["min_range"] = bundle.lidar.min_range;
    lidar["max_range"] = bundle.lidar.max_range;
    lidar["scan_period"] = bundle.lidar.scan_period;
    lidar["range_scale"] = bundle.lidar.range_scale;
    lidar["range_bias"] = bundle.lidar.range_bias;
    root["lidar"] = lidar;

    YAML::Node ext;
    ext["T_imu_lidar"] = ExtrinsicToYaml(bundle.T_imu_lidar);
    ext["T_cam_imu"] = ExtrinsicToYaml(bundle.T_cam_imu);
    if (bundle.has_T_cam_lidar) {
        ext["T_cam_lidar"] = ExtrinsicToYaml(bundle.T_cam_lidar);
    }
    ext["T_base_wheel"] = ExtrinsicToYaml(bundle.T_base_wheel);
    ext["time_offset_imu"] = bundle.time_offset_imu;
    ext["time_offset_lidar"] = bundle.time_offset_lidar;
    ext["time_offset_cam"] = bundle.time_offset_cam;
    ext["time_offset_wheel"] = bundle.time_offset_wheel;
    root["extrinsics"] = ext;

    try {
        std::ofstream ofs(yaml_path);
        if (!ofs) {
            return false;
        }
        ofs << root;
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace calibration
}  // namespace autonomy::localization::atlas
