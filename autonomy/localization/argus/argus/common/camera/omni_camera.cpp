/*
 * Copyright 2026 The Openbot Authors
 * Port of rpg_vikit OmniCamera.
 */

#include "argus/common/camera/omni_camera.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

namespace argus {
namespace camera {

namespace {
constexpr int kCmvMaxBuf = 1024;

bool ReadPolynomial(FILE* f, char* buf, int* length, double* coeffs) {
    if (std::fgets(buf, kCmvMaxBuf, f) == nullptr) {
        return false;
    }
    std::fscanf(f, "\n");
    if (std::fscanf(f, "%d", length) != 1 || *length <= 0 ||
        *length > kOcamMaxPolLength) {
        return false;
    }
    for (int i = 0; i < *length; ++i) {
        if (std::fscanf(f, " %lf", &coeffs[i]) != 1) {
            return false;
        }
    }
    return true;
}

bool ReadInvPolynomial(FILE* f, char* buf, int* length, double* coeffs) {
    std::fscanf(f, "\n");
    return ReadPolynomial(f, buf, length, coeffs);
}

bool ReadCenterAffineSize(FILE* f, char* buf, OcamModel* ocam) {
    // Center.
    std::fscanf(f, "\n");
    if (std::fgets(buf, kCmvMaxBuf, f) == nullptr) {
        return false;
    }
    std::fscanf(f, "\n");
    if (std::fscanf(f, "%lf %lf\n", &ocam->xc, &ocam->yc) != 2) {
        return false;
    }

    // Affine.
    if (std::fgets(buf, kCmvMaxBuf, f) == nullptr) {
        return false;
    }
    std::fscanf(f, "\n");
    if (std::fscanf(f, "%lf %lf %lf\n", &ocam->c, &ocam->d, &ocam->e) != 3) {
        return false;
    }

    // Image size (height width in Scaramuzza file).
    if (std::fgets(buf, kCmvMaxBuf, f) == nullptr) {
        return false;
    }
    std::fscanf(f, "\n");
    if (std::fscanf(f, "%d %d", &ocam->height, &ocam->width) != 2) {
        return false;
    }
    return true;
}

}  // namespace

OmniCamera::OmniCamera(const std::string& calib_file) {
    if (LoadCalibFile(calib_file)) {
        width_ = ocam_.width;
        height_ = ocam_.height;
        scale_ = 1.0;
        error_multiplier_ = computeErrorMultiplier();
        valid_ = true;
    }
}

OmniCamera::OmniCamera(const OcamModel& model) : ocam_(model) {
    width_ = ocam_.width;
    height_ = ocam_.height;
    scale_ = 1.0;
    if (width_ > 0 && height_ > 0 && ocam_.length_pol > 0 &&
        ocam_.length_invpol > 0) {
        error_multiplier_ = computeErrorMultiplier();
        valid_ = true;
    }
}

bool OmniCamera::LoadCalibFile(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (f == nullptr) {
        std::cerr << "[argus::camera] OmniCamera: cannot open " << path
                  << std::endl;
        return false;
    }

    char buf[kCmvMaxBuf];
    // Polynomial.
    if (!ReadPolynomial(f, buf, &ocam_.length_pol, ocam_.pol)) {
        std::fclose(f);
        return false;
    }
    // Inverse polynomial.
    if (!ReadInvPolynomial(f, buf, &ocam_.length_invpol, ocam_.invpol)) {
        std::fclose(f);
        return false;
    }
    if (!ReadCenterAffineSize(f, buf, &ocam_)) {
        std::fclose(f);
        return false;
    }

    std::fclose(f);
    return true;
}

Eigen::Vector3d OmniCamera::cam2world(const double& u, const double& v) const {
    Eigen::Vector3d xyz;
    const double invdet = 1.0 / (ocam_.c - ocam_.d * ocam_.e);
    xyz[0] = invdet * ((v - ocam_.xc) - ocam_.d * (u - ocam_.yc));
    xyz[1] = invdet * (-ocam_.e * (v - ocam_.xc) + ocam_.c * (u - ocam_.yc));

    const double r = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
    xyz[2] = ocam_.pol[0];
    double r_i = 1.0;
    for (int i = 1; i < ocam_.length_pol; ++i) {
        r_i *= r;
        xyz[2] += r_i * ocam_.pol[i];
    }
    xyz.normalize();

    // Davide ↔ pinhole axis convention (vikit).
    const double temp = xyz[0];
    xyz[0] = xyz[1];
    xyz[1] = temp;
    xyz[2] = -xyz[2];
    return xyz;
}

Eigen::Vector3d OmniCamera::cam2world(const Eigen::Vector2d& px) const {
    return cam2world(px[0], px[1]);
}

Eigen::Vector2d OmniCamera::world2cam(const Eigen::Vector3d& xyz_c) const {
    Eigen::Vector2d uv;
    Eigen::Vector3d w;
    w[0] = xyz_c[1];
    w[1] = xyz_c[0];
    w[2] = -xyz_c[2];

    const double norm = std::sqrt(w[0] * w[0] + w[1] * w[1]);
    const double theta = std::atan(w[2] / norm);

    if (norm != 0.0) {
        double rho = ocam_.invpol[0];
        double t_i = 1.0;
        for (int i = 1; i < ocam_.length_invpol; ++i) {
            t_i *= theta;
            rho += t_i * ocam_.invpol[i];
        }
        const double x = w[0] * rho / norm;
        const double y = w[1] * rho / norm;
        uv[1] = x * ocam_.c + y * ocam_.d + ocam_.xc;
        uv[0] = x * ocam_.e + y + ocam_.yc;
    } else {
        uv[1] = ocam_.xc;
        uv[0] = ocam_.yc;
    }
    return uv;
}

Eigen::Vector2d OmniCamera::world2cam(const Eigen::Vector2d& uv) const {
    return world2cam(Unproject2d(uv).normalized());
}

double OmniCamera::computeErrorMultiplier() {
    Eigen::Vector3d v1 = cam2world(0.5 * width_, 0.5 * height_);
    Eigen::Vector3d v2 = cam2world(0.5 * width_ + 0.5, 0.5 * height_);
    v1.normalize();
    v2.normalize();
    const double factor1 = 0.5 / (1.0 - v1.dot(v2));

    v1 = cam2world(static_cast<double>(width_), 0.5 * height_);
    v2 = cam2world(-0.5 + static_cast<double>(width_), 0.5 * height_);
    v1.normalize();
    v2.normalize();
    const double factor2 = 0.5 / (1.0 - v1.dot(v2));

    return (factor2 + factor1) * 0.5;
}

}  // namespace camera
}  // namespace argus
