/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/evaluation/tum_format_writer.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <Eigen/Geometry>

namespace argus {
namespace evaluation {
namespace {

transform::Rigid3d FromTum(double tx, double ty, double tz, double qx,
                           double qy, double qz, double qw) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (q.norm() > 1e-12) {
        q.normalize();
    } else {
        q = Eigen::Quaterniond::Identity();
    }
    return transform::Rigid3d(Eigen::Vector3d(tx, ty, tz), q);
}

}  // namespace

bool LoadTumTrajectory(const std::string& path, std::vector<TimedPose>* out) {
    if (out == nullptr) {
        return false;
    }
    out->clear();
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        double t = 0;
        double tx = 0;
        double ty = 0;
        double tz = 0;
        double qx = 0;
        double qy = 0;
        double qz = 0;
        double qw = 1;
        if (!(ss >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            continue;
        }
        TimedPose p;
        p.stamp_sec = t;
        p.T_w_b = FromTum(tx, ty, tz, qx, qy, qz, qw);
        out->push_back(p);
    }
    std::sort(out->begin(), out->end(),
              [](const TimedPose& a, const TimedPose& b) {
                  return a.stamp_sec < b.stamp_sec;
              });
    return !out->empty();
}

bool WriteTumTrajectory(const std::string& path,
                        const std::vector<TimedPose>& poses) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << std::setprecision(17);
    for (const TimedPose& p : poses) {
        const Eigen::Quaterniond q(p.T_w_b.rotation());
        const Eigen::Vector3d& t = p.T_w_b.translation();
        out << p.stamp_sec << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
            << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
            << '\n';
    }
    return true;
}

}  // namespace evaluation
}  // namespace argus
