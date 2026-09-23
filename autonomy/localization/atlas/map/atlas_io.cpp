/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file atlas_io.cpp
 * @brief Atlas JSON serialize / deserialize (`SaveAtlasJson` / `LoadAtlasJson`).
 *
 * Role aligned with ORB-SLAM3 Atlas file I/O; custom JSON format (v1–v5).
 */

#include "autonomy/localization/atlas/map/atlas_io.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>

#include "autolink/common/log.hpp"

#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/sensor/camera.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

constexpr char kB64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string Base64Encode(const unsigned char* data, size_t len) {
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        const unsigned int n =
            (static_cast<unsigned int>(data[i]) << 16) |
            ((i + 1 < len) ? (static_cast<unsigned int>(data[i + 1]) << 8)
                           : 0u) |
            ((i + 2 < len) ? static_cast<unsigned int>(data[i + 2]) : 0u);
        out.push_back(kB64[(n >> 18) & 63]);
        out.push_back(kB64[(n >> 12) & 63]);
        out.push_back((i + 1 < len) ? kB64[(n >> 6) & 63] : '=');
        out.push_back((i + 2 < len) ? kB64[n & 63] : '=');
    }
    return out;
}

int B64Index(char c) {
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    return -1;
}

std::vector<unsigned char> Base64Decode(const std::string& in) {
    std::vector<unsigned char> out;
    out.reserve(in.size() * 3 / 4);
    unsigned int buf = 0;
    int bits = 0;
    for (char c : in) {
        if (c == '=' || c == '\n' || c == '\r') {
            break;
        }
        const int v = B64Index(c);
        if (v < 0) {
            continue;
        }
        buf = (buf << 6) | static_cast<unsigned int>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<unsigned char>((buf >> bits) & 0xFF));
        }
    }
    return out;
}

nlohmann::json Se3ToJson(const SE3& T) {
    nlohmann::json j;
    const Mat33 R = T.linear();
    const Vec3 t = T.translation();
    j["R"] = {R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2),
              R(2, 0), R(2, 1), R(2, 2)};
    j["t"] = {t.x(), t.y(), t.z()};
    return j;
}

SE3 Se3FromJson(const nlohmann::json& j) {
    SE3 T = SE3Identity();
    const auto& R = j.at("R");
    const auto& t = j.at("t");
    Mat33 Rm;
    Rm << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
    T.linear() = Rm;
    T.translation() = Vec3(t[0], t[1], t[2]);
    return T;
}

nlohmann::json DescriptorToJson(const cv::Mat& desc) {
    nlohmann::json j;
    if (desc.empty()) {
        j["rows"] = 0;
        j["cols"] = 0;
        j["b64"] = "";
        return j;
    }
    CV_Assert(desc.isContinuous());
    j["rows"] = desc.rows;
    j["cols"] = desc.cols;
    j["type"] = desc.type();
    j["b64"] = Base64Encode(desc.data,
                            static_cast<size_t>(desc.rows) *
                                static_cast<size_t>(desc.step));
    return j;
}

cv::Mat DescriptorFromJson(const nlohmann::json& j) {
    const int rows = j.value("rows", 0);
    const int cols = j.value("cols", 0);
    if (rows <= 0 || cols <= 0 || !j.contains("b64")) {
        return {};
    }
    const int type = j.value("type", CV_8UC1);
    auto bytes = Base64Decode(j["b64"].get<std::string>());
    cv::Mat desc(rows, cols, type);
    const size_t need =
        static_cast<size_t>(desc.rows) * static_cast<size_t>(desc.step);
    if (bytes.size() < need) {
        return {};
    }
    std::memcpy(desc.data, bytes.data(), need);
    return desc;
}

nlohmann::json KeyPointsToJson(const std::vector<cv::KeyPoint>& kps) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& kp : kps) {
        arr.push_back({{"x", kp.pt.x},
                       {"y", kp.pt.y},
                       {"octave", kp.octave},
                       {"angle", kp.angle},
                       {"size", kp.size}});
    }
    return arr;
}

std::vector<cv::KeyPoint> KeyPointsFromJson(const nlohmann::json& arr) {
    std::vector<cv::KeyPoint> kps;
    if (!arr.is_array()) {
        return kps;
    }
    kps.reserve(arr.size());
    for (const auto& kj : arr) {
        cv::KeyPoint kp;
        kp.pt.x = kj.value("x", 0.f);
        kp.pt.y = kj.value("y", 0.f);
        kp.octave = kj.value("octave", 0);
        kp.angle = kj.value("angle", -1.f);
        kp.size = kj.value("size", 1.f);
        kps.push_back(kp);
    }
    return kps;
}

nlohmann::json FloatVecToJson(const std::vector<float>& v) {
    return nlohmann::json(v);
}

std::vector<float> FloatVecFromJson(const nlohmann::json& j) {
    if (!j.is_array()) {
        return {};
    }
    return j.get<std::vector<float>>();
}

std::vector<int> IntVecFromJson(const nlohmann::json& j) {
    if (!j.is_array()) {
        return {};
    }
    return j.get<std::vector<int>>();
}

nlohmann::json BowToJson(const fbow::BoWVector& bow) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& kv : bow) {
        nlohmann::json item;
        item["id"] = kv.first;
        item["w"] = static_cast<float>(kv.second);
        arr.push_back(std::move(item));
    }
    return arr;
}

fbow::BoWVector BowFromJson(const nlohmann::json& j) {
    fbow::BoWVector bow;
    if (!j.is_array()) {
        return bow;
    }
    for (const auto& e : j) {
        // fbow::_float::operator= takes a non-const float&.
        float weight = static_cast<float>(e.value("w", 0.0));
        bow[e.value("id", 0u)] = weight;
    }
    return bow;
}

nlohmann::json FeatToJson(const fbow::BoWFeatVector& feat) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& kv : feat) {
        arr.push_back({{"node", kv.first}, {"idx", kv.second}});
    }
    return arr;
}

fbow::BoWFeatVector FeatFromJson(const nlohmann::json& j) {
    fbow::BoWFeatVector feat;
    if (!j.is_array()) {
        return feat;
    }
    for (const auto& e : j) {
        feat[e.value("node", 0u)] =
            e.value("idx", std::vector<uint32_t>{});
    }
    return feat;
}

nlohmann::json CameraToJson(const std::shared_ptr<sensor::GeometricCamera>& cam) {
    nlohmann::json j;
    if (!cam) {
        return j;
    }
    j["model"] = cam->type_name();
    j["fx"] = cam->fx();
    j["fy"] = cam->fy();
    j["cx"] = cam->cx();
    j["cy"] = cam->cy();
    j["width"] = cam->width();
    j["height"] = cam->height();
    if (const auto* kb =
            dynamic_cast<const sensor::camera::KannalaBrandt*>(cam.get())) {
        j["params"] = kb->params();
    } else if (const auto* rt =
                   dynamic_cast<const sensor::camera::RadTan*>(cam.get())) {
        j["k1"] = rt->k1();
        j["k2"] = rt->k2();
        j["p1"] = rt->p1();
        j["p2"] = rt->p2();
        j["k3"] = rt->k3();
    }
    return j;
}

std::shared_ptr<sensor::GeometricCamera> CameraFromJson(
    const nlohmann::json& j) {
    if (!j.is_object() || !j.contains("model")) {
        return nullptr;
    }
    sensor::camera::CameraFactory::Intrinsics intr;
    intr.model = j.value("model", "pinhole");
    intr.fx = j.value("fx", 1.0);
    intr.fy = j.value("fy", 1.0);
    intr.cx = j.value("cx", 0.0);
    intr.cy = j.value("cy", 0.0);
    intr.width = j.value("width", 0);
    intr.height = j.value("height", 0);
    if (j.contains("params") && j["params"].is_array()) {
        const auto& p = j["params"];
        if (p.size() >= 8) {
            intr.k1 = p[4];
            intr.k2 = p[5];
            intr.k3 = p[6];
            intr.k4 = p[7];
        }
    }
    intr.k1 = j.value("k1", intr.k1);
    intr.k2 = j.value("k2", intr.k2);
    intr.k3 = j.value("k3", intr.k3);
    intr.k4 = j.value("k4", intr.k4);
    intr.p1 = j.value("p1", 0.0);
    intr.p2 = j.value("p2", 0.0);
    auto cam = sensor::camera::CameraFactory::Create(intr);
    return std::shared_ptr<sensor::GeometricCamera>(std::move(cam));
}

nlohmann::json Vec3ToJson(const Vec3& v) {
    return nlohmann::json::array({v.x(), v.y(), v.z()});
}

Vec3 Vec3FromJson(const nlohmann::json& j) {
    if (!j.is_array() || j.size() < 3) {
        return Vec3::Zero();
    }
    return Vec3(j[0], j[1], j[2]);
}

nlohmann::json Mat33ToJson(const Mat33& M) {
    nlohmann::json a = nlohmann::json::array();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            a.push_back(M(r, c));
        }
    }
    return a;
}

Mat33 Mat33FromJson(const nlohmann::json& j) {
    Mat33 M = Mat33::Identity();
    if (!j.is_array() || j.size() < 9) {
        return M;
    }
    for (int i = 0; i < 9; ++i) {
        M(i / 3, i % 3) = j[i];
    }
    return M;
}

template <typename Derived>
nlohmann::json EigenToJson(const Eigen::MatrixBase<Derived>& M) {
    nlohmann::json a = nlohmann::json::array();
    for (int r = 0; r < M.rows(); ++r) {
        for (int c = 0; c < M.cols(); ++c) {
            a.push_back(M(r, c));
        }
    }
    return a;
}

template <typename Derived>
void EigenFromJson(const nlohmann::json& j, Eigen::MatrixBase<Derived>* M) {
    if (M == nullptr || !j.is_array()) {
        return;
    }
    const int need = static_cast<int>(M->rows() * M->cols());
    if (static_cast<int>(j.size()) < need) {
        return;
    }
    int k = 0;
    for (int r = 0; r < M->rows(); ++r) {
        for (int c = 0; c < M->cols(); ++c) {
            (*M)(r, c) = j[k++];
        }
    }
}

nlohmann::json BiasToJson(const sensor::imu::Bias& b) {
    return {{"ba", Vec3ToJson(b.accelerometer)},
            {"bg", Vec3ToJson(b.gyroscope)}};
}

sensor::imu::Bias BiasFromJson(const nlohmann::json& j) {
    sensor::imu::Bias b;
    if (j.contains("ba")) {
        b.accelerometer = Vec3FromJson(j["ba"]);
    }
    if (j.contains("bg")) {
        b.gyroscope = Vec3FromJson(j["bg"]);
    }
    return b;
}

nlohmann::json CalibToJson(const sensor::imu::Calib& c) {
    nlohmann::json j;
    j["Tbc"] = Se3ToJson(c.T_body_camera);
    j["cov"] = EigenToJson(c.covariance.diagonal());
    j["cov_walk"] = EigenToJson(c.covariance_walk.diagonal());
    j["is_set"] = c.is_set;
    return j;
}

sensor::imu::Calib CalibFromJson(const nlohmann::json& j) {
    sensor::imu::Calib c;
    if (j.contains("Tbc")) {
        c.T_body_camera = Se3FromJson(j["Tbc"]);
        c.T_camera_body = c.T_body_camera.inverse();
    }
    if (j.contains("cov") && j["cov"].is_array() && j["cov"].size() >= 6) {
        Eigen::Matrix<double, 6, 1> d;
        EigenFromJson(j["cov"], &d);
        c.covariance.diagonal() = d;
    }
    if (j.contains("cov_walk") && j["cov_walk"].is_array() &&
        j["cov_walk"].size() >= 6) {
        Eigen::Matrix<double, 6, 1> d;
        EigenFromJson(j["cov_walk"], &d);
        c.covariance_walk.diagonal() = d;
    }
    c.is_set = j.value("is_set", false);
    return c;
}

nlohmann::json PreintToJson(const sensor::imu::Preintegrator& pre) {
    const auto s = pre.ExportState();
    nlohmann::json j;
    j["dt"] = s.delta_t;
    j["C"] = EigenToJson(s.covariance);
    j["Info"] = EigenToJson(s.information);
    j["Nga"] = EigenToJson(s.nga_diag);
    j["NgaWalk"] = EigenToJson(s.nga_walk_diag);
    j["b"] = BiasToJson(s.bias);
    j["bu"] = BiasToJson(s.bias_updated);
    j["dR"] = Mat33ToJson(s.dR);
    j["dV"] = Vec3ToJson(s.dV);
    j["dP"] = Vec3ToJson(s.dP);
    j["JRg"] = Mat33ToJson(s.JRg);
    j["JVg"] = Mat33ToJson(s.JVg);
    j["JVa"] = Mat33ToJson(s.JVa);
    j["JPg"] = Mat33ToJson(s.JPg);
    j["JPa"] = Mat33ToJson(s.JPa);
    j["avgA"] = Vec3ToJson(s.avg_A);
    j["avgW"] = Vec3ToJson(s.avg_W);
    j["db"] = EigenToJson(s.db);
    nlohmann::json meas = nlohmann::json::array();
    for (const auto& row : s.measurements) {
        meas.push_back(EigenToJson(row));
    }
    j["measurements"] = meas;
    return j;
}

std::shared_ptr<sensor::imu::Preintegrator> PreintFromJson(
    const nlohmann::json& j) {
    sensor::imu::Preintegrator::State s;
    s.delta_t = j.value("dt", 0.0);
    if (j.contains("C")) {
        EigenFromJson(j["C"], &s.covariance);
    }
    if (j.contains("Info")) {
        EigenFromJson(j["Info"], &s.information);
    }
    if (j.contains("Nga")) {
        EigenFromJson(j["Nga"], &s.nga_diag);
    }
    if (j.contains("NgaWalk")) {
        EigenFromJson(j["NgaWalk"], &s.nga_walk_diag);
    }
    if (j.contains("b")) {
        s.bias = BiasFromJson(j["b"]);
    }
    if (j.contains("bu")) {
        s.bias_updated = BiasFromJson(j["bu"]);
    }
    if (j.contains("dR")) {
        s.dR = Mat33FromJson(j["dR"]);
    }
    if (j.contains("dV")) {
        s.dV = Vec3FromJson(j["dV"]);
    }
    if (j.contains("dP")) {
        s.dP = Vec3FromJson(j["dP"]);
    }
    if (j.contains("JRg")) {
        s.JRg = Mat33FromJson(j["JRg"]);
    }
    if (j.contains("JVg")) {
        s.JVg = Mat33FromJson(j["JVg"]);
    }
    if (j.contains("JVa")) {
        s.JVa = Mat33FromJson(j["JVa"]);
    }
    if (j.contains("JPg")) {
        s.JPg = Mat33FromJson(j["JPg"]);
    }
    if (j.contains("JPa")) {
        s.JPa = Mat33FromJson(j["JPa"]);
    }
    if (j.contains("avgA")) {
        s.avg_A = Vec3FromJson(j["avgA"]);
    }
    if (j.contains("avgW")) {
        s.avg_W = Vec3FromJson(j["avgW"]);
    }
    if (j.contains("db")) {
        EigenFromJson(j["db"], &s.db);
    }
    if (j.contains("measurements") && j["measurements"].is_array()) {
        for (const auto& row_j : j["measurements"]) {
            Eigen::Matrix<double, 7, 1> row =
                Eigen::Matrix<double, 7, 1>::Zero();
            EigenFromJson(row_j, &row);
            s.measurements.push_back(row);
        }
    }
    auto pre = std::make_shared<sensor::imu::Preintegrator>();
    pre->ImportState(s);
    return pre;
}

}  // namespace

bool SaveAtlasJson(const MultiMap& multi_map, const std::string& path) {
    nlohmann::json root;
    // v5: + IMU bias/calib/velocity + Preintegrator + prev/next KF links.
    root["version"] = 5;
    root["format"] = "atlas_json_v5";

    nlohmann::json kfs = nlohmann::json::array();
    auto keyframes = multi_map.GetAllKeyFrames();
    std::sort(keyframes.begin(), keyframes.end(),
              [](const auto& a, const auto& b) {
                  return a->timestamp < b->timestamp;
              });
    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        nlohmann::json j;
        j["id"] = kf->id;
        j["timestamp"] = kf->timestamp;
        j["Tcw"] = Se3ToJson(kf->GetPose());
        j["fx"] = kf->fx;
        j["fy"] = kf->fy;
        j["cx"] = kf->cx;
        j["cy"] = kf->cy;
        j["bf"] = kf->baseline_times_fx;
        j["b"] = kf->baseline_meters;
        j["th_depth"] = kf->depth_threshold;
        j["scale_factor"] = kf->scale_factor;
        j["scale_levels"] = kf->scale_levels;
        j["scale_factors"] = FloatVecToJson(kf->scale_factors);

        j["keypoints"] = KeyPointsToJson(kf->GetKeyPoints());
        j["descriptors"] = DescriptorToJson(kf->GetDescriptors());
        j["depths"] = FloatVecToJson(kf->GetDepths());
        j["right_u"] = FloatVecToJson(kf->GetRightCoordinates());

        j["num_left"] = kf->num_left;
        j["num_right"] = kf->num_right;
        if (kf->HasDualCameraIndex()) {
            j["keypoints_right"] = KeyPointsToJson(kf->GetKeyPointsRight());
            j["T_c1_c2"] = Se3ToJson(kf->T_c1_c2);
            j["left_to_right"] = kf->left_to_right_match;
        }

        if (kf->camera) {
            j["camera"] = CameraToJson(kf->camera);
        }
        if (kf->camera2) {
            j["camera2"] = CameraToJson(kf->camera2);
        }
        if (kf->HasBoW()) {
            j["bow"] = BowToJson(kf->bow_vector());
            j["feat"] = FeatToJson(kf->feat_vector());
        }

        j["imu_ready"] = kf->imu_ready;
        j["has_velocity"] = kf->has_velocity;
        if (kf->has_velocity) {
            j["Vw"] = Vec3ToJson(kf->velocity_world);
        }
        j["imu_bias"] = BiasToJson(kf->imu_bias);
        if (kf->imu_calib.is_set) {
            j["imu_calib"] = CalibToJson(kf->imu_calib);
        }
        if (kf->imu_preintegrated) {
            j["imu_preint"] = PreintToJson(*kf->imu_preintegrated);
        }
        if (auto prev = kf->previous_keyframe.lock()) {
            j["prev_kf"] = prev->id;
        }
        if (auto next = kf->next_keyframe.lock()) {
            j["next_kf"] = next->id;
        }

        if (auto parent = kf->GetParent()) {
            j["parent"] = parent->id;
        }
        nlohmann::json conns = nlohmann::json::array();
        for (const auto& [weak_kf, weight] : kf->GetConnectedKeyFrames()) {
            auto other = weak_kf.lock();
            if (!other) {
                continue;
            }
            conns.push_back({{"kf", other->id}, {"w", weight}});
        }
        j["connections"] = conns;

        kfs.push_back(j);
    }
    root["keyframes"] = kfs;

    nlohmann::json mps = nlohmann::json::array();
    for (const auto& mp : multi_map.GetAllMapPoints()) {
        if (!mp || mp->isBad()) {
            continue;
        }
        const Vec3 p = mp->GetWorldPos();
        nlohmann::json j;
        j["id"] = mp->id;
        j["xyz"] = {p.x(), p.y(), p.z()};
        nlohmann::json obs = nlohmann::json::array();
        for (const auto& [weak_kf, idx] : mp->GetObservations()) {
            auto kf = weak_kf.lock();
            if (!kf) {
                continue;
            }
            obs.push_back({{"kf", kf->id}, {"idx", idx}});
        }
        j["observations"] = obs;
        const cv::Mat d = mp->GetDescriptor();
        if (!d.empty()) {
            j["descriptor"] = DescriptorToJson(d);
        }
        mps.push_back(j);
    }
    root["map_points"] = mps;

    std::ofstream ofs(path);
    if (!ofs) {
        AERROR << "SaveAtlasJson: cannot write " << path;
        return false;
    }
    ofs << std::setw(2) << root << "\n";
    AINFO << "SaveAtlasJson: " << kfs.size() << " KF, " << mps.size()
          << " MP → " << path;
    return true;
}

bool LoadAtlasJson(MultiMap* multi_map, const std::string& path) {
    if (multi_map == nullptr) {
        return false;
    }
    std::ifstream ifs(path);
    if (!ifs) {
        AERROR << "LoadAtlasJson: cannot open " << path;
        return false;
    }
    nlohmann::json root;
    try {
        ifs >> root;
    } catch (const std::exception& e) {
        AERROR << "LoadAtlasJson: parse error: " << e.what();
        return false;
    }

    multi_map->clearAtlas();
    multi_map->CreateNewMap();
    Map* map = multi_map->mutable_current_map();
    if (map == nullptr) {
        return false;
    }

    struct PendingGraph {
        long unsigned int parent_id = 0;
        bool has_parent = false;
        long unsigned int prev_id = 0;
        bool has_prev = false;
        long unsigned int next_id = 0;
        bool has_next = false;
        std::vector<std::pair<long unsigned int, int>> connections;
    };
    std::unordered_map<long unsigned int, PendingGraph> pending_graph;

    std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> id_to_kf;
    std::shared_ptr<KeyFrame> first_kf;
    int n_kf = 0;
    for (const auto& j : root.value("keyframes", nlohmann::json::array())) {
        tracking::Frame frame;
        frame.timestamp = j.value("timestamp", 0.0);
        frame.SetPose(Se3FromJson(j.at("Tcw")));
        frame.fx = static_cast<float>(j.value("fx", 0.0));
        frame.fy = static_cast<float>(j.value("fy", 0.0));
        frame.cx = static_cast<float>(j.value("cx", 0.0));
        frame.cy = static_cast<float>(j.value("cy", 0.0));
        if (frame.fx > 1e-6f) {
            frame.inv_fx = 1.f / frame.fx;
        }
        if (frame.fy > 1e-6f) {
            frame.inv_fy = 1.f / frame.fy;
        }
        frame.baseline_times_fx =
            static_cast<float>(j.value("bf", j.value("baseline_times_fx", 0.0)));
        frame.baseline_meters =
            static_cast<float>(j.value("b", j.value("baseline_meters", 0.0)));
        frame.depth_threshold =
            static_cast<float>(j.value("th_depth", 40.0));
        frame.scale_factor =
            static_cast<float>(j.value("scale_factor", 1.0));
        frame.scale_levels = j.value("scale_levels", 0);
        frame.scale_factors = FloatVecFromJson(j.value("scale_factors",
                                                       nlohmann::json::array()));

        if (j.contains("keypoints")) {
            frame.keypoints = KeyPointsFromJson(j["keypoints"]);
            frame.keypoints_undistorted = frame.keypoints;
            frame.num_keypoints = static_cast<int>(frame.keypoints.size());
        }
        frame.depths = FloatVecFromJson(j.value("depths", nlohmann::json::array()));
        frame.right_coordinate =
            FloatVecFromJson(j.value("right_u", nlohmann::json::array()));
        if (frame.depths.size() < frame.keypoints.size()) {
            frame.depths.resize(frame.keypoints.size(), -1.f);
        }
        if (frame.right_coordinate.size() < frame.keypoints.size()) {
            frame.right_coordinate.resize(frame.keypoints.size(), -1.f);
        }
        frame.map_points.assign(frame.keypoints.size(), nullptr);

        if (j.contains("descriptors")) {
            frame.descriptors = DescriptorFromJson(j["descriptors"]);
        }

        frame.num_left = j.value("num_left", -1);
        frame.num_right = j.value("num_right", 0);
        if (frame.num_left >= 0) {
            if (j.contains("keypoints_right")) {
                frame.keypoints_right = KeyPointsFromJson(j["keypoints_right"]);
            }
            if (j.contains("T_c1_c2")) {
                frame.T_c1_c2 = Se3FromJson(j["T_c1_c2"]);
            }
            frame.left_to_right_match =
                IntVecFromJson(j.value("left_to_right", nlohmann::json::array()));
            const size_t n_total =
                static_cast<size_t>(frame.num_left + frame.num_right);
            if (frame.map_points.size() < n_total) {
                frame.map_points.resize(n_total, nullptr);
            }
            if (frame.depths.size() < n_total) {
                frame.depths.resize(n_total, -1.f);
            }
            if (frame.right_coordinate.size() < n_total) {
                frame.right_coordinate.resize(n_total, -1.f);
            }
        }

        if (j.contains("camera")) {
            frame.camera = CameraFromJson(j["camera"]);
        }
        if (j.contains("camera2")) {
            frame.camera2 = CameraFromJson(j["camera2"]);
        }

        auto kf = std::make_shared<KeyFrame>(frame, map);
        if (j.contains("id")) {
            kf->id = j["id"].get<long unsigned int>();
        }
        if (j.contains("bow") && j.contains("feat")) {
            kf->SetBoW(BowFromJson(j["bow"]), FeatFromJson(j["feat"]));
        }
        kf->imu_ready = j.value("imu_ready", false);
        kf->has_velocity = j.value("has_velocity", false);
        if (j.contains("Vw")) {
            kf->velocity_world = Vec3FromJson(j["Vw"]);
            kf->has_velocity = true;
        }
        if (j.contains("imu_bias")) {
            kf->imu_bias = BiasFromJson(j["imu_bias"]);
        }
        if (j.contains("imu_calib")) {
            kf->imu_calib = CalibFromJson(j["imu_calib"]);
        }
        if (j.contains("imu_preint")) {
            kf->imu_preintegrated = PreintFromJson(j["imu_preint"]);
        }
        map->AddKeyFrame(kf);
        id_to_kf[kf->id] = kf;
        if (!first_kf) {
            first_kf = kf;
        }

        PendingGraph pg;
        if (j.contains("parent") && !j["parent"].is_null()) {
            pg.parent_id = j["parent"].get<long unsigned int>();
            pg.has_parent = true;
        }
        if (j.contains("prev_kf")) {
            pg.prev_id = j["prev_kf"].get<long unsigned int>();
            pg.has_prev = true;
        }
        if (j.contains("next_kf")) {
            pg.next_id = j["next_kf"].get<long unsigned int>();
            pg.has_next = true;
        }
        if (j.contains("connections")) {
            for (const auto& cj : j["connections"]) {
                pg.connections.emplace_back(cj.value("kf", 0ULL),
                                            cj.value("w", 0));
            }
        }
        pending_graph[kf->id] = std::move(pg);
        ++n_kf;
    }

    // Restore covisibility + spanning tree + IMU temporal links.
    for (const auto& [kf_id, pg] : pending_graph) {
        auto it = id_to_kf.find(kf_id);
        if (it == id_to_kf.end()) {
            continue;
        }
        auto& kf = it->second;
        for (const auto& [other_id, weight] : pg.connections) {
            auto jt = id_to_kf.find(other_id);
            if (jt == id_to_kf.end() || weight <= 0) {
                continue;
            }
            kf->AddConnection(jt->second, weight);
        }
        kf->UpdateBestCovisibles();
        if (pg.has_parent) {
            auto jt = id_to_kf.find(pg.parent_id);
            if (jt != id_to_kf.end()) {
                kf->ChangeParent(jt->second);
            }
        }
        if (pg.has_prev) {
            auto jt = id_to_kf.find(pg.prev_id);
            if (jt != id_to_kf.end()) {
                kf->previous_keyframe = jt->second;
            }
        }
        if (pg.has_next) {
            auto jt = id_to_kf.find(pg.next_id);
            if (jt != id_to_kf.end()) {
                kf->next_keyframe = jt->second;
            }
        }
    }

    int n_mp = 0;
    for (const auto& j : root.value("map_points", nlohmann::json::array())) {
        const auto& xyz = j.at("xyz");
        const Vec3 p(xyz[0], xyz[1], xyz[2]);
        auto mp = std::make_shared<MapPoint>(p, first_kf, map);
        if (j.contains("id")) {
            mp->id = j["id"].get<long unsigned int>();
        }
        map->AddMapPoint(mp);

        if (j.contains("observations")) {
            for (const auto& oj : j["observations"]) {
                const auto kf_id = oj.value("kf", 0ULL);
                const int idx = oj.value("idx", -1);
                auto it = id_to_kf.find(kf_id);
                if (it == id_to_kf.end() || idx < 0) {
                    continue;
                }
                mp->AddObservation(it->second, idx);
                it->second->AddMapPoint(mp, idx);
            }
            mp->ComputeDistinctiveDescriptors();
            mp->UpdateNormalAndDepth();
        }
        ++n_mp;
    }
    AINFO << "LoadAtlasJson: " << n_kf << " KF, " << n_mp << " MP ← " << path;
    return n_kf > 0 || n_mp > 0;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
