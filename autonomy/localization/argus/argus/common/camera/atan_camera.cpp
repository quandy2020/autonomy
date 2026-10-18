/*
 * Copyright 2026 The Openbot Authors
 * Port of rpg_vikit ATANCamera (Apache-2.0 / original vikit license).
 */

#include "argus/common/camera/atan_camera.hpp"

namespace argus {
namespace camera {

ATANCamera::ATANCamera(double width, double height, double scale, double fx,
                       double fy, double cx, double cy, double s)
    : ATANCamera(static_cast<int>(width * scale),
                 static_cast<int>(height * scale), scale, fx * scale,
                 fy * scale, cx * scale, cy * scale, s, true) {}

ATANCamera ATANCamera::FromNormalized(double width, double height, double fx_n,
                                      double fy_n, double cx_n, double cy_n,
                                      double s) {
    // rpg_vikit: fx_(W*fx), cx_(cx*W - 0.5)
    return ATANCamera(static_cast<int>(width), static_cast<int>(height), 1.0,
                      width * fx_n, height * fy_n, cx_n * width - 0.5,
                      cy_n * height - 0.5, s, true);
}

ATANCamera::ATANCamera(int width, int height, double scale, double fx,
                       double fy, double cx, double cy, double s, bool)
    : AbstractCamera(width, height, scale),
      fx_(fx),
      fy_(fy),
      fx_inv_(1.0 / fx),
      fy_inv_(1.0 / fy),
      cx_(cx),
      cy_(cy),
      s_(s) {
    if (s_ != 0.0) {
        tans_ = 2.0 * std::tan(s_ / 2.0);
        tans_inv_ = 1.0 / tans_;
        s_inv_ = 1.0 / s_;
        distortion_ = true;
    } else {
        s_inv_ = 0.0;
        tans_ = 0.0;
        tans_inv_ = 0.0;
        distortion_ = false;
    }
}

Eigen::Vector3d ATANCamera::cam2world(const double& x, const double& y) const {
    Eigen::Vector2d dist_cam((x - cx_) * fx_inv_, (y - cy_) * fy_inv_);
    const double dist_r = dist_cam.norm();
    const double r = invrtrans(dist_r);
    const double d_factor = (dist_r > 0.01) ? (r / dist_r) : 1.0;
    return Unproject2d(d_factor * dist_cam).normalized();
}

Eigen::Vector3d ATANCamera::cam2world(const Eigen::Vector2d& px) const {
    return cam2world(px[0], px[1]);
}

Eigen::Vector2d ATANCamera::world2cam(const Eigen::Vector3d& xyz_c) const {
    return world2cam(Project2d(xyz_c));
}

Eigen::Vector2d ATANCamera::world2cam(const Eigen::Vector2d& uv) const {
    const double r = uv.norm();
    const double factor = rtrans_factor(r);
    const Eigen::Vector2d dist_cam = factor * uv;
    return Eigen::Vector2d(cx_ + fx_ * dist_cam[0], cy_ + fy_ * dist_cam[1]);
}

}  // namespace camera
}  // namespace argus
