#pragma once

#include <Eigen/Core>

namespace autonomy::localization::atlas {

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> Skew(const Eigen::Matrix<Scalar, 3, 1>& vector) {
    Eigen::Matrix<Scalar, 3, 3> matrix;
    matrix << Scalar{0}, -vector.z(), vector.y(), vector.z(), Scalar{0},
        -vector.x(), -vector.y(), vector.x(), Scalar{0};
    return matrix;
}

template <typename Scalar>
Scalar Square(Scalar value) {
    return value * value;
}

}  // namespace autonomy::localization::atlas
