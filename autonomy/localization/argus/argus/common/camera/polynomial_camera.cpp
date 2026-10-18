/*
 * Copyright 2026 The Openbot Authors
 * Port of rpg_vikit PolynomialCamera.
 */

#include "argus/common/camera/polynomial_camera.hpp"

#include <cmath>

namespace argus {
namespace camera {

PolynomialCamera::PolynomialCamera(double width, double height, double scale,
                                   double fx, double fy, double cx, double cy,
                                   double skew, double k2, double k3, double k4,
                                   double k5, double k6, double k7)
    : AbstractCamera(static_cast<int>(width * scale),
                     static_cast<int>(height * scale), scale),
      fx_(fx * scale),
      fy_(fy * scale),
      cx_(cx * scale),
      cy_(cy * scale),
      skew_(skew * scale),
      distortion_(std::fabs(k2) > 1e-7),
      k2_(k2),
      k3_(k3),
      k4_(k4),
      k5_(k5),
      k6_(k6),
      k7_(k7) {}

Eigen::Vector3d PolynomialCamera::cam2world(const double& u,
                                            const double& v) const {
    Eigen::Vector3d xyz;
    if (!distortion_) {
        xyz[1] = (v - cy_) / fy_;
        xyz[0] = (u - cx_ - xyz[1] * skew_) / fx_;
        xyz[2] = 1.0;
    } else {
        double y = (v - cy_) / fy_;
        double x = (u - cx_ - y * skew_) / fx_;
        const double thetad = std::sqrt(x * x + y * y);
        double theta = thetad;
        for (int i = 0; i < 7; ++i) {
            const double theta2 = theta * theta;
            const double theta3 = theta2 * theta;
            const double theta4 = theta3 * theta;
            const double theta5 = theta4 * theta;
            const double theta6 = theta5 * theta;
            theta = thetad / (1.0 + k2_ * theta + k3_ * theta2 + k4_ * theta3 +
                              k5_ * theta4 + k6_ * theta5 + k7_ * theta6);
        }
        if (thetad > 1e-8) {
            const double scaling = std::tan(theta) / thetad;
            x *= scaling;
            y *= scaling;
        }
        xyz[0] = x;
        xyz[1] = y;
        xyz[2] = 1.0;
    }
    return xyz.normalized();
}

Eigen::Vector3d PolynomialCamera::cam2world(const Eigen::Vector2d& uv) const {
    return cam2world(uv[0], uv[1]);
}

Eigen::Vector2d PolynomialCamera::world2cam(const Eigen::Vector3d& xyz) const {
    Eigen::Vector2d px;
    if (!distortion_) {
        // Match vikit PolynomialCamera::world2cam(Vector3d) (no Project2d).
        px[0] = fx_ * xyz[0] + cx_;
        px[1] = fy_ * xyz[1] + cy_;
    } else {
        const double r = std::sqrt(xyz(1) * xyz(1) + xyz(0) * xyz(0));
        const double theta = std::acos(xyz(2) / xyz.norm());
        const double thetad = thetad_from_theta(theta);
        const double scaling = (r > 1e-12) ? (thetad / r) : 0.0;
        const double xd = xyz[0] * scaling;
        const double yd = xyz[1] * scaling;
        px[0] = xd * fx_ + yd * skew_ + cx_;
        px[1] = yd * fy_ + cy_;
    }
    return px;
}

Eigen::Vector2d PolynomialCamera::world2cam(const Eigen::Vector2d& uv) const {
    Eigen::Vector2d px;
    if (!distortion_) {
        px[0] = fx_ * uv[0] + cx_;
        px[1] = fy_ * uv[1] + cy_;
    } else {
        const double r = uv.norm();
        if (r < 1e-8) {
            return uv;
        }
        const double theta = std::atan(r);
        const double thetad = thetad_from_theta(theta);
        const double scaling = thetad / r;
        const double xd = uv[0] * scaling;
        const double yd = uv[1] * scaling;
        px[0] = xd * fx_ + yd * skew_ + cx_;
        px[1] = yd * fy_ + cy_;
    }
    return px;
}

Eigen::Matrix2d PolynomialCamera::jacobian_2x2(
    const Eigen::Vector2d& uv) const {
    const double r = uv.norm();
    if (r < 1e-8) {
        return Eigen::Matrix2d::Identity();
    }
    const double inv_r = 1.0 / r;
    const double r2 = r * r;
    const double dr_du = uv(0) * inv_r;
    const double dr_dv = uv(1) * inv_r;
    const double theta = std::atan(r);
    const double dtheta_dr = 1.0 / (1.0 + r * r);
    const double thetad = thetad_from_theta(theta);
    const double dthetad_dtheta = deriv_thetad_from_theta(theta);
    const double dthetad_dr = dthetad_dtheta * dtheta_dr;
    const double scaling = thetad / r;
    const double dscaling_du = (dthetad_dr * dr_du * r - dr_du * thetad) / r2;
    const double dscaling_dv = (dthetad_dr * dr_dv * r - dr_dv * thetad) / r2;
    Eigen::Matrix2d jac;
    jac << dscaling_du * uv(0) + scaling, dscaling_dv * uv(0),
        dscaling_du * uv(1), dscaling_dv * uv(1) + scaling;
    return jac;
}

Eigen::Matrix<double, 2, 3> PolynomialCamera::jacobian_2x3(
    const Eigen::Vector3d& p) const {
    Eigen::Matrix<double, 2, 3> jac;
    const double x = p[0];
    const double y = p[1];
    const double z_inv = 1.0 / p[2];
    const double z_inv_2 = z_inv * z_inv;
    jac(0, 0) = fx_ * z_inv;
    jac(0, 1) = skew_ * z_inv;
    jac(0, 2) = -fx_ * x * z_inv_2;
    jac(1, 0) = 0.0;
    jac(1, 1) = fy_ * z_inv;
    jac(1, 2) = -fy_ * y * z_inv_2;
    return jac;
}

}  // namespace camera
}  // namespace argus
