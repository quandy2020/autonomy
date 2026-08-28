#include "autonomy/localization/atlas/optimize/g2o/line3d.hpp"

#include <g2o/stuff/misc.h>

namespace autonomy::localization::atlas::optimize::plp_g2o {

namespace {

Eigen::Matrix3d skew(const Vec3_t& t) {
    Eigen::Matrix3d S;
    S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
    return S;
}

}  // namespace

PluckerVec6 Line3D::toCartesian() const {
    PluckerVec6 cartesian;
    cartesian.tail<3>() = d() / d().norm();
    Eigen::Matrix3d W = -skew(d());
    const double damping = static_cast<double>(1e-9);
    Eigen::Matrix3d A = W.transpose() * W + (Eigen::Matrix3d::Identity() * damping);
    cartesian.head<3>() = A.ldlt().solve(W.transpose() * w());
    return cartesian;
}

Line3D operator*(const Eigen::Isometry3d& t, const Line3D& line) {
    PluckerMatrix6 A = PluckerMatrix6::Zero();
    A.block<3, 3>(0, 0) = t.linear();
    A.block<3, 3>(0, 3) = skew(t.translation()) * t.linear();
    A.block<3, 3>(3, 3) = t.linear();
    const PluckerVec6 v = static_cast<PluckerVec6>(line);
    return Line3D(A * v);
}

PluckerVec6 transformCartesianLine(const Eigen::Isometry3d& t, const PluckerVec6& line) {
    PluckerVec6 l;
    l.head<3>() = t * line.head<3>();
    l.tail<3>() = t.linear() * line.tail<3>();
    return normalizeCartesianLine(l);
}

PluckerVec6 normalizeCartesianLine(const PluckerVec6& line) {
    Vec3_t p0 = line.head<3>();
    Vec3_t d0 = line.tail<3>();
    d0.normalize();
    p0 -= d0 * (d0.dot(p0));
    PluckerVec6 nl;
    nl.head<3>() = p0;
    nl.tail<3>() = d0;
    return nl;
}

}  // namespace autonomy::localization::atlas::optimize::plp_g2o
