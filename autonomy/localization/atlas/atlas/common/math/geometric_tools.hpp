/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_COMMON_MATH_GEOMETRIC_TOOLS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_COMMON_MATH_GEOMETRIC_TOOLS_HPP_

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include "autolink/common/log.hpp"
#include "autonomy/common/math/se3.hpp"

namespace autonomy::localization::atlas {

class KeyFrame;

class GeometricTools
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Compute the Fundamental matrix between KF1 and KF2
    static Eigen::Matrix3f ComputeF12(KeyFrame*& pKF1, KeyFrame*& pKF2);

    // Triangulate point with KF1 and KF2
    static bool Triangulate(Eigen::Vector3f& x_c1, Eigen::Vector3f& x_c2,
                            Eigen::Matrix<float, 3, 4>& Tc1w,
                            Eigen::Matrix<float, 3, 4>& Tc2w,
                            Eigen::Vector3f& x3D);

    template <int rows, int cols>
    static bool CheckMatrices(const cv::Mat& cvMat,
                              const Eigen::Matrix<float, rows, cols>& eigMat) {
        const float epsilon = 1e-3;
        // AINFO << cvMat.cols - cols << cvMat.rows - rows << std::endl;
        if (rows != cvMat.rows || cols != cvMat.cols) {
            AINFO << "wrong cvmat size\n";
            return false;
        }
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                if ((cvMat.at<float>(i, j) > (eigMat(i, j) + epsilon)) ||
                    (cvMat.at<float>(i, j) < (eigMat(i, j) - epsilon))) {
                    AINFO << "cv mat:\n" << cvMat << std::endl;
                    AINFO << "eig mat:\n" << eigMat << std::endl;
                    return false;
                }
        return true;
    }

    template <typename T, int rows, int cols>
    static bool CheckMatrices(const Eigen::Matrix<T, rows, cols>& eigMat1,
                              const Eigen::Matrix<T, rows, cols>& eigMat2) {
        const float epsilon = 1e-3;
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                if ((eigMat1(i, j) > (eigMat2(i, j) + epsilon)) ||
                    (eigMat1(i, j) < (eigMat2(i, j) - epsilon))) {
                    AINFO << "eig mat 1:\n" << eigMat1 << std::endl;
                    AINFO << "eig mat 2:\n" << eigMat2 << std::endl;
                    return false;
                }
        return true;
    }
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_COMMON_MATH_GEOMETRIC_TOOLS_HPP_
