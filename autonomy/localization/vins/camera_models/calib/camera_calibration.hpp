/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

 #ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/core/core.hpp>

#include "autonomy/localization/vins/camera_models/camera_models/camera.hpp"

namespace camodocal
{

class CameraCalibration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraCalibration();

    CameraCalibration(Camera::ModelType modelType,
                      const std::string& cameraName,
                      const cv::Size& imageSize,
                      const cv::Size& boardSize,
                      float squareSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& corners);

    bool calibrate(void);

    int sampleCount(void) const;
    std::vector<std::vector<cv::Point2f> >& imagePoints(void);
    const std::vector<std::vector<cv::Point2f> >& imagePoints(void) const;
    std::vector<std::vector<cv::Point3f> >& scenePoints(void);
    const std::vector<std::vector<cv::Point3f> >& scenePoints(void) const;
    CameraPtr& camera(void);
    const CameraConstPtr camera(void) const;

    Eigen::Matrix2d& measurementCovariance(void);
    const Eigen::Matrix2d& measurementCovariance(void) const;

    cv::Mat& cameraPoses(void);
    const cv::Mat& cameraPoses(void) const;

    void drawResults(std::vector<cv::Mat>& images) const;

    void writeParams(const std::string& filename) const;

    bool writeChessboardData(const std::string& filename) const;
    bool readChessboardData(const std::string& filename);

    void setVerbose(bool verbose);

private:
    bool calibrateHelper(CameraPtr& camera,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    void optimize(CameraPtr& camera,
                  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    cv::Size m_boardSize;
    float m_squareSize;

    CameraPtr m_camera;
    cv::Mat m_cameraPoses;

    std::vector<std::vector<cv::Point2f> > m_imagePoints;
    std::vector<std::vector<cv::Point3f> > m_scenePoints;

    Eigen::Matrix2d m_measurementCovariance;

    bool m_verbose;
};

}

#endif
