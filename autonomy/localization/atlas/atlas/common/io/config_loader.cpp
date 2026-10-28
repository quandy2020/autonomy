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

#include "atlas/common/io/config_loader.hpp"
#include "autolink/common/log.hpp"

#include "atlas/sensor/camera/kannala_brandt_camera.hpp"
#include "atlas/sensor/camera/pinhole_camera.hpp"

#include "atlas/system.hpp"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>

#include <iostream>

namespace autonomy::localization::atlas {

template <>
float ConfigLoader::readParameter<float>(cv::FileStorage& fConfigLoader,
                                         const std::string& name, bool& found,
                                         const bool required) {
    cv::FileNode node = fConfigLoader[name];
    if (node.empty()) {
        if (required) {
            AERROR << name << " required parameter does not exist, aborting..."
                   << std::endl;
            exit(-1);
        } else {
            AERROR << name << " optional parameter does not exist..."
                   << std::endl;
            found = false;
            return 0.0f;
        }
    } else if (!node.isReal()) {
        AERROR << name << " parameter must be a real number, aborting..."
               << std::endl;
        exit(-1);
    } else {
        found = true;
        return node.real();
    }
}

template <>
int ConfigLoader::readParameter<int>(cv::FileStorage& fConfigLoader,
                                     const std::string& name, bool& found,
                                     const bool required) {
    cv::FileNode node = fConfigLoader[name];
    if (node.empty()) {
        if (required) {
            AERROR << name << " required parameter does not exist, aborting..."
                   << std::endl;
            exit(-1);
        } else {
            AERROR << name << " optional parameter does not exist..."
                   << std::endl;
            found = false;
            return 0;
        }
    } else if (!node.isInt()) {
        AERROR << name << " parameter must be an integer number, aborting..."
               << std::endl;
        exit(-1);
    } else {
        found = true;
        return node.operator int();
    }
}

template <>
std::string ConfigLoader::readParameter<std::string>(
    cv::FileStorage& fConfigLoader, const std::string& name, bool& found,
    const bool required) {
    cv::FileNode node = fConfigLoader[name];
    if (node.empty()) {
        if (required) {
            AERROR << name << " required parameter does not exist, aborting..."
                   << std::endl;
            exit(-1);
        } else {
            AERROR << name << " optional parameter does not exist..."
                   << std::endl;
            found = false;
            return std::string();
        }
    } else if (!node.isString()) {
        AERROR << name << " parameter must be a string, aborting..."
               << std::endl;
        exit(-1);
    } else {
        found = true;
        return node.string();
    }
}

template <>
cv::Mat ConfigLoader::readParameter<cv::Mat>(cv::FileStorage& fConfigLoader,
                                             const std::string& name,
                                             bool& found, const bool required) {
    cv::FileNode node = fConfigLoader[name];
    if (node.empty()) {
        if (required) {
            AERROR << name << " required parameter does not exist, aborting..."
                   << std::endl;
            exit(-1);
        } else {
            AERROR << name << " optional parameter does not exist..."
                   << std::endl;
            found = false;
            return cv::Mat();
        }
    } else {
        found = true;
        return node.mat();
    }
}

ConfigLoader::ConfigLoader(const std::string& configFile, const int& sensor)
    : bNeedToUndistort_(false),
      bNeedToRectify_(false),
      bNeedToResize1_(false),
      bNeedToResize2_(false) {
    sensor_ = sensor;

    // Open settings file
    cv::FileStorage fConfigLoader(configFile, cv::FileStorage::READ);
    if (!fConfigLoader.isOpened()) {
        AERROR << "[ERROR]: could not open configuration file at: "
               << configFile << std::endl;
        AERROR << "Aborting..." << std::endl;

        exit(-1);
    } else {
        AINFO << "Loading settings from " << configFile << std::endl;
    }

    // Read first camera
    readCamera1(fConfigLoader);
    AINFO << "\t-Loaded camera 1" << std::endl;

    // Read second camera if stereo (not rectified)
    if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
        readCamera2(fConfigLoader);
        AINFO << "\t-Loaded camera 2" << std::endl;
    }

    // Read image info
    readImageInfo(fConfigLoader);
    AINFO << "\t-Loaded image info" << std::endl;

    if (sensor_ == System::IMU_MONOCULAR || sensor_ == System::IMU_STEREO ||
        sensor_ == System::IMU_RGBD) {
        readIMU(fConfigLoader);
        AINFO << "\t-Loaded IMU calibration" << std::endl;
    }

    if (sensor_ == System::RGBD || sensor_ == System::IMU_RGBD) {
        readRGBD(fConfigLoader);
        AINFO << "\t-Loaded RGB-D calibration" << std::endl;
    }

    readORB(fConfigLoader);
    AINFO << "\t-Loaded ORB settings" << std::endl;
    readViewer(fConfigLoader);
    AINFO << "\t-Loaded viewer settings" << std::endl;
    readLoadAndSave(fConfigLoader);
    AINFO << "\t-Loaded Atlas settings" << std::endl;
    readOtherParameters(fConfigLoader);
    AINFO << "\t-Loaded misc parameters" << std::endl;

    if (bNeedToRectify_) {
        precomputeRectificationMaps();
        AINFO << "\t-Computed rectification maps" << std::endl;
    }

    AINFO << "----------------------------------" << std::endl;
}

void ConfigLoader::readCamera1(cv::FileStorage& fConfigLoader) {
    bool found;

    // Read camera model
    std::string cameraModel =
        readParameter<std::string>(fConfigLoader, "Camera.type", found);

    std::vector<float> vCalibration;
    if (cameraModel == "PinHole") {
        cameraType_ = PinHole;

        // Read intrinsic parameters
        float fx = readParameter<float>(fConfigLoader, "Camera1.fx", found);
        float fy = readParameter<float>(fConfigLoader, "Camera1.fy", found);
        float cx = readParameter<float>(fConfigLoader, "Camera1.cx", found);
        float cy = readParameter<float>(fConfigLoader, "Camera1.cy", found);

        vCalibration = {fx, fy, cx, cy};

        calibration1_ = new PinholeCamera(vCalibration);
        originalCalib1_ = new PinholeCamera(vCalibration);

        // Check if it is a distorted PinHole
        readParameter<float>(fConfigLoader, "Camera1.k1", found, false);
        if (found) {
            readParameter<float>(fConfigLoader, "Camera1.k3", found, false);
            if (found) {
                vPinHoleDistorsion1_.resize(5);
                vPinHoleDistorsion1_[4] =
                    readParameter<float>(fConfigLoader, "Camera1.k3", found);
            } else {
                vPinHoleDistorsion1_.resize(4);
            }
            vPinHoleDistorsion1_[0] =
                readParameter<float>(fConfigLoader, "Camera1.k1", found);
            vPinHoleDistorsion1_[1] =
                readParameter<float>(fConfigLoader, "Camera1.k2", found);
            vPinHoleDistorsion1_[2] =
                readParameter<float>(fConfigLoader, "Camera1.p1", found);
            vPinHoleDistorsion1_[3] =
                readParameter<float>(fConfigLoader, "Camera1.p2", found);
        }

        // Check if we need to correct distortion from the images
        if ((sensor_ == System::MONOCULAR ||
             sensor_ == System::IMU_MONOCULAR) &&
            vPinHoleDistorsion1_.size() != 0) {
            bNeedToUndistort_ = true;
        }
    } else if (cameraModel == "Rectified") {
        cameraType_ = Rectified;

        // Read intrinsic parameters
        float fx = readParameter<float>(fConfigLoader, "Camera1.fx", found);
        float fy = readParameter<float>(fConfigLoader, "Camera1.fy", found);
        float cx = readParameter<float>(fConfigLoader, "Camera1.cx", found);
        float cy = readParameter<float>(fConfigLoader, "Camera1.cy", found);

        vCalibration = {fx, fy, cx, cy};

        calibration1_ = new PinholeCamera(vCalibration);
        originalCalib1_ = new PinholeCamera(vCalibration);

        // Rectified images are assumed to be ideal PinHole images (no
        // distortion)
    } else if (cameraModel == "KannalaBrandt8") {
        cameraType_ = KannalaBrandt;

        // Read intrinsic parameters
        float fx = readParameter<float>(fConfigLoader, "Camera1.fx", found);
        float fy = readParameter<float>(fConfigLoader, "Camera1.fy", found);
        float cx = readParameter<float>(fConfigLoader, "Camera1.cx", found);
        float cy = readParameter<float>(fConfigLoader, "Camera1.cy", found);

        float k0 = readParameter<float>(fConfigLoader, "Camera1.k1", found);
        float k1 = readParameter<float>(fConfigLoader, "Camera1.k2", found);
        float k2 = readParameter<float>(fConfigLoader, "Camera1.k3", found);
        float k3 = readParameter<float>(fConfigLoader, "Camera1.k4", found);

        vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

        calibration1_ = new KannalaBrandtCamera(vCalibration);
        originalCalib1_ = new KannalaBrandtCamera(vCalibration);

        if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
            int colBegin = readParameter<int>(
                fConfigLoader, "Camera1.overlappingBegin", found);
            int colEnd = readParameter<int>(fConfigLoader,
                                            "Camera1.overlappingEnd", found);
            std::vector<int> vOverlapping = {colBegin, colEnd};

            static_cast<KannalaBrandtCamera*>(calibration1_)->mvLappingArea =
                vOverlapping;
        }
    } else {
        AERROR << "Error: " << cameraModel << " not known" << std::endl;
        exit(-1);
    }
}

void ConfigLoader::readCamera2(cv::FileStorage& fConfigLoader) {
    bool found;
    std::vector<float> vCalibration;
    if (cameraType_ == PinHole) {
        bNeedToRectify_ = true;

        // Read intrinsic parameters
        float fx = readParameter<float>(fConfigLoader, "Camera2.fx", found);
        float fy = readParameter<float>(fConfigLoader, "Camera2.fy", found);
        float cx = readParameter<float>(fConfigLoader, "Camera2.cx", found);
        float cy = readParameter<float>(fConfigLoader, "Camera2.cy", found);

        vCalibration = {fx, fy, cx, cy};

        calibration2_ = new PinholeCamera(vCalibration);
        originalCalib2_ = new PinholeCamera(vCalibration);

        // Check if it is a distorted PinHole
        readParameter<float>(fConfigLoader, "Camera2.k1", found, false);
        if (found) {
            readParameter<float>(fConfigLoader, "Camera2.k3", found, false);
            if (found) {
                vPinHoleDistorsion2_.resize(5);
                vPinHoleDistorsion2_[4] =
                    readParameter<float>(fConfigLoader, "Camera2.k3", found);
            } else {
                vPinHoleDistorsion2_.resize(4);
            }
            vPinHoleDistorsion2_[0] =
                readParameter<float>(fConfigLoader, "Camera2.k1", found);
            vPinHoleDistorsion2_[1] =
                readParameter<float>(fConfigLoader, "Camera2.k2", found);
            vPinHoleDistorsion2_[2] =
                readParameter<float>(fConfigLoader, "Camera2.p1", found);
            vPinHoleDistorsion2_[3] =
                readParameter<float>(fConfigLoader, "Camera2.p2", found);
        }
    } else if (cameraType_ == KannalaBrandt) {
        // Read intrinsic parameters
        float fx = readParameter<float>(fConfigLoader, "Camera2.fx", found);
        float fy = readParameter<float>(fConfigLoader, "Camera2.fy", found);
        float cx = readParameter<float>(fConfigLoader, "Camera2.cx", found);
        float cy = readParameter<float>(fConfigLoader, "Camera2.cy", found);

        float k0 = readParameter<float>(fConfigLoader, "Camera1.k1", found);
        float k1 = readParameter<float>(fConfigLoader, "Camera1.k2", found);
        float k2 = readParameter<float>(fConfigLoader, "Camera1.k3", found);
        float k3 = readParameter<float>(fConfigLoader, "Camera1.k4", found);

        vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

        calibration2_ = new KannalaBrandtCamera(vCalibration);
        originalCalib2_ = new KannalaBrandtCamera(vCalibration);

        int colBegin = readParameter<int>(fConfigLoader,
                                          "Camera2.overlappingBegin", found);
        int colEnd =
            readParameter<int>(fConfigLoader, "Camera2.overlappingEnd", found);
        std::vector<int> vOverlapping = {colBegin, colEnd};

        static_cast<KannalaBrandtCamera*>(calibration2_)->mvLappingArea =
            vOverlapping;
    }

    // Load stereo extrinsic calibration
    if (cameraType_ == Rectified) {
        b_ = readParameter<float>(fConfigLoader, "Stereo.b", found);
        bf_ = b_ * calibration1_->getParameter(0);
    } else {
        cv::Mat cvTlr =
            readParameter<cv::Mat>(fConfigLoader, "Stereo.T_c1_c2", found);
        Tlr_ = Converter::toLieGroup(cvTlr);

        // TODO: also search for Trl and invert if necessary

        b_ = Tlr_.translation().norm();
        bf_ = b_ * calibration1_->getParameter(0);
    }

    thDepth_ = readParameter<float>(fConfigLoader, "Stereo.ThDepth", found);
}

void ConfigLoader::readImageInfo(cv::FileStorage& fConfigLoader) {
    bool found;
    // Read original and desired image dimensions
    int originalRows =
        readParameter<int>(fConfigLoader, "Camera.height", found);
    int originalCols = readParameter<int>(fConfigLoader, "Camera.width", found);
    originalImSize_.width = originalCols;
    originalImSize_.height = originalRows;

    newImSize_ = originalImSize_;
    int newHeigh =
        readParameter<int>(fConfigLoader, "Camera.newHeight", found, false);
    if (found) {
        bNeedToResize1_ = true;
        newImSize_.height = newHeigh;

        if (!bNeedToRectify_) {
            // Update calibration
            float scaleRowFactor = static_cast<float>(newImSize_.height) /
                                   static_cast<float>(originalImSize_.height);
            calibration1_->setParameter(
                calibration1_->getParameter(1) * scaleRowFactor, 1);
            calibration1_->setParameter(
                calibration1_->getParameter(3) * scaleRowFactor, 3);

            if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) &&
                cameraType_ != Rectified) {
                calibration2_->setParameter(
                    calibration2_->getParameter(1) * scaleRowFactor, 1);
                calibration2_->setParameter(
                    calibration2_->getParameter(3) * scaleRowFactor, 3);
            }
        }
    }

    int newWidth =
        readParameter<int>(fConfigLoader, "Camera.newWidth", found, false);
    if (found) {
        bNeedToResize1_ = true;
        newImSize_.width = newWidth;

        if (!bNeedToRectify_) {
            // Update calibration
            float scaleColFactor = static_cast<float>(newImSize_.width) /
                                   static_cast<float>(originalImSize_.width);
            calibration1_->setParameter(
                calibration1_->getParameter(0) * scaleColFactor, 0);
            calibration1_->setParameter(
                calibration1_->getParameter(2) * scaleColFactor, 2);

            if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) &&
                cameraType_ != Rectified) {
                calibration2_->setParameter(
                    calibration2_->getParameter(0) * scaleColFactor, 0);
                calibration2_->setParameter(
                    calibration2_->getParameter(2) * scaleColFactor, 2);

                if (cameraType_ == KannalaBrandt) {
                    static_cast<KannalaBrandtCamera*>(calibration1_)
                        ->mvLappingArea[0] *= scaleColFactor;
                    static_cast<KannalaBrandtCamera*>(calibration1_)
                        ->mvLappingArea[1] *= scaleColFactor;

                    static_cast<KannalaBrandtCamera*>(calibration2_)
                        ->mvLappingArea[0] *= scaleColFactor;
                    static_cast<KannalaBrandtCamera*>(calibration2_)
                        ->mvLappingArea[1] *= scaleColFactor;
                }
            }
        }
    }

    fps_ = readParameter<int>(fConfigLoader, "Camera.fps", found);
    bRGB_ = static_cast<bool>(
        readParameter<int>(fConfigLoader, "Camera.RGB", found));
}

void ConfigLoader::readIMU(cv::FileStorage& fConfigLoader) {
    bool found;
    noiseGyro_ = readParameter<float>(fConfigLoader, "IMU.NoiseGyro", found);
    noiseAcc_ = readParameter<float>(fConfigLoader, "IMU.NoiseAcc", found);
    gyroWalk_ = readParameter<float>(fConfigLoader, "IMU.GyroWalk", found);
    accWalk_ = readParameter<float>(fConfigLoader, "IMU.AccWalk", found);
    imuFrequency_ = readParameter<float>(fConfigLoader, "IMU.Frequency", found);

    cv::Mat cvTbc = readParameter<cv::Mat>(fConfigLoader, "IMU.T_b_c1", found);
    Tbc_ = Converter::toLieGroup(cvTbc);

    readParameter<int>(fConfigLoader, "IMU.InsertKFsWhenLost", found, false);
    if (found) {
        insertKFsWhenLost_ = static_cast<bool>(readParameter<int>(
            fConfigLoader, "IMU.InsertKFsWhenLost", found, false));
    } else {
        insertKFsWhenLost_ = true;
    }
}

void ConfigLoader::readRGBD(cv::FileStorage& fConfigLoader) {
    bool found;

    depthMapFactor_ =
        readParameter<float>(fConfigLoader, "RGBD.DepthMapFactor", found);
    thDepth_ = readParameter<float>(fConfigLoader, "Stereo.ThDepth", found);
    b_ = readParameter<float>(fConfigLoader, "Stereo.b", found);
    bf_ = b_ * calibration1_->getParameter(0);
}

void ConfigLoader::readORB(cv::FileStorage& fConfigLoader) {
    bool found;

    nFeatures_ =
        readParameter<int>(fConfigLoader, "ORBextractor.nFeatures", found);
    scaleFactor_ =
        readParameter<float>(fConfigLoader, "ORBextractor.scaleFactor", found);
    nLevels_ = readParameter<int>(fConfigLoader, "ORBextractor.nLevels", found);
    initThFAST_ =
        readParameter<int>(fConfigLoader, "ORBextractor.iniThFAST", found);
    minThFAST_ =
        readParameter<int>(fConfigLoader, "ORBextractor.minThFAST", found);
}

void ConfigLoader::readViewer(cv::FileStorage& fConfigLoader) {
    bool found;

    keyFrameSize_ =
        readParameter<float>(fConfigLoader, "Viewer.KeyFrameSize", found);
    keyFrameLineWidth_ =
        readParameter<float>(fConfigLoader, "Viewer.KeyFrameLineWidth", found);
    graphLineWidth_ =
        readParameter<float>(fConfigLoader, "Viewer.GraphLineWidth", found);
    pointSize_ = readParameter<float>(fConfigLoader, "Viewer.PointSize", found);
    cameraSize_ =
        readParameter<float>(fConfigLoader, "Viewer.CameraSize", found);
    cameraLineWidth_ =
        readParameter<float>(fConfigLoader, "Viewer.CameraLineWidth", found);
    viewPointX_ =
        readParameter<float>(fConfigLoader, "Viewer.ViewpointX", found);
    viewPointY_ =
        readParameter<float>(fConfigLoader, "Viewer.ViewpointY", found);
    viewPointZ_ =
        readParameter<float>(fConfigLoader, "Viewer.ViewpointZ", found);
    viewPointF_ =
        readParameter<float>(fConfigLoader, "Viewer.ViewpointF", found);
    imageViewerScale_ = readParameter<float>(
        fConfigLoader, "Viewer.imageViewScale", found, false);

    if (!found)
        imageViewerScale_ = 1.0f;
}

void ConfigLoader::readLoadAndSave(cv::FileStorage& fConfigLoader) {
    bool found;

    sLoadFrom_ = readParameter<std::string>(
        fConfigLoader, "System.LoadAtlasFromFile", found, false);
    sSaveto_ = readParameter<std::string>(
        fConfigLoader, "System.SaveAtlasToFile", found, false);
}

void ConfigLoader::readOtherParameters(cv::FileStorage& fConfigLoader) {
    bool found;

    thFarPoints_ =
        readParameter<float>(fConfigLoader, "System.thFarPoints", found, false);
}

void ConfigLoader::precomputeRectificationMaps() {
    // Precompute rectification maps, new calibrations, ...
    cv::Mat K1 = static_cast<PinholeCamera*>(calibration1_)->toK();
    K1.convertTo(K1, CV_64F);
    cv::Mat K2 = static_cast<PinholeCamera*>(calibration2_)->toK();
    K2.convertTo(K2, CV_64F);

    cv::Mat cvTlr;
    cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
    cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
    R12.convertTo(R12, CV_64F);
    cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
    t12.convertTo(t12, CV_64F);

    cv::Mat R_r1_u1, R_r2_u2;
    cv::Mat P1, P2, Q;

    cv::stereoRectify(K1, camera1DistortionCoef(), K2, camera2DistortionCoef(),
                      newImSize_, R12, t12, R_r1_u1, R_r2_u2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, -1, newImSize_);
    cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1,
                                P1.rowRange(0, 3).colRange(0, 3), newImSize_,
                                CV_32F, M1l_, M2l_);
    cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2,
                                P2.rowRange(0, 3).colRange(0, 3), newImSize_,
                                CV_32F, M1r_, M2r_);

    // Update calibration
    calibration1_->setParameter(P1.at<double>(0, 0), 0);
    calibration1_->setParameter(P1.at<double>(1, 1), 1);
    calibration1_->setParameter(P1.at<double>(0, 2), 2);
    calibration1_->setParameter(P1.at<double>(1, 2), 3);

    // Update bf
    bf_ = b_ * P1.at<double>(0, 0);

    // Update relative pose between camera 1 and IMU if necessary
    if (sensor_ == System::IMU_STEREO) {
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
        autonomy::common::math::SE3f T_r1_u1(eigenR_r1_u1,
                                             Eigen::Vector3f::Zero());
        Tbc_ = Tbc_ * T_r1_u1.inverse();
    }
}

std::ostream& operator<<(std::ostream& output, const ConfigLoader& settings) {
    output << "SLAM settings: " << std::endl;

    output << "\t-Camera 1 parameters (";
    if (settings.cameraType_ == ConfigLoader::PinHole ||
        settings.cameraType_ == ConfigLoader::Rectified) {
        output << "Pinhole";
    } else {
        output << "Kannala-Brandt";
    }
    output << ")"
           << ": [";
    for (std::size_t i = 0; i < settings.originalCalib1_->size(); i++) {
        output << " " << settings.originalCalib1_->getParameter(i);
    }
    output << " ]" << std::endl;

    if (!settings.vPinHoleDistorsion1_.empty()) {
        output << "\t-Camera 1 distortion parameters: [ ";
        for (float d : settings.vPinHoleDistorsion1_) {
            output << " " << d;
        }
        output << " ]" << std::endl;
    }

    if (settings.sensor_ == System::STEREO ||
        settings.sensor_ == System::IMU_STEREO) {
        output << "\t-Camera 2 parameters (";
        if (settings.cameraType_ == ConfigLoader::PinHole ||
            settings.cameraType_ == ConfigLoader::Rectified) {
            output << "Pinhole";
        } else {
            output << "Kannala-Brandt";
        }
        output << ""
               << ": [";
        for (std::size_t i = 0; i < settings.originalCalib2_->size(); i++) {
            output << " " << settings.originalCalib2_->getParameter(i);
        }
        output << " ]" << std::endl;

        if (!settings.vPinHoleDistorsion2_.empty()) {
            output << "\t-Camera 1 distortion parameters: [ ";
            for (float d : settings.vPinHoleDistorsion2_) {
                output << " " << d;
            }
            output << " ]" << std::endl;
        }
    }

    output << "\t-Original image size: [ " << settings.originalImSize_.width
           << " , " << settings.originalImSize_.height << " ]" << std::endl;
    output << "\t-Current image size: [ " << settings.newImSize_.width << " , "
           << settings.newImSize_.height << " ]" << std::endl;

    if (settings.bNeedToRectify_) {
        output << "\t-Camera 1 parameters after rectification: [ ";
        for (std::size_t i = 0; i < settings.calibration1_->size(); i++) {
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << std::endl;
    } else if (settings.bNeedToResize1_) {
        output << "\t-Camera 1 parameters after resize: [ ";
        for (std::size_t i = 0; i < settings.calibration1_->size(); i++) {
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << std::endl;

        if ((settings.sensor_ == System::STEREO ||
             settings.sensor_ == System::IMU_STEREO) &&
            settings.cameraType_ == ConfigLoader::KannalaBrandt) {
            output << "\t-Camera 2 parameters after resize: [ ";
            for (std::size_t i = 0; i < settings.calibration2_->size(); i++) {
                output << " " << settings.calibration2_->getParameter(i);
            }
            output << " ]" << std::endl;
        }
    }

    output << "\t-Sequence FPS: " << settings.fps_ << std::endl;

    // Stereo stuff
    if (settings.sensor_ == System::STEREO ||
        settings.sensor_ == System::IMU_STEREO) {
        output << "\t-Stereo baseline: " << settings.b_ << std::endl;
        output << "\t-Stereo depth threshold : " << settings.thDepth_
               << std::endl;

        if (settings.cameraType_ == ConfigLoader::KannalaBrandt) {
            auto vOverlapping1 =
                static_cast<KannalaBrandtCamera*>(settings.calibration1_)
                    ->mvLappingArea;
            auto vOverlapping2 =
                static_cast<KannalaBrandtCamera*>(settings.calibration2_)
                    ->mvLappingArea;
            output << "\t-Camera 1 overlapping area: [ " << vOverlapping1[0]
                   << " , " << vOverlapping1[1] << " ]" << std::endl;
            output << "\t-Camera 2 overlapping area: [ " << vOverlapping2[0]
                   << " , " << vOverlapping2[1] << " ]" << std::endl;
        }
    }

    if (settings.sensor_ == System::IMU_MONOCULAR ||
        settings.sensor_ == System::IMU_STEREO ||
        settings.sensor_ == System::IMU_RGBD) {
        output << "\t-Gyro noise: " << settings.noiseGyro_ << std::endl;
        output << "\t-Accelerometer noise: " << settings.noiseAcc_ << std::endl;
        output << "\t-Gyro walk: " << settings.gyroWalk_ << std::endl;
        output << "\t-Accelerometer walk: " << settings.accWalk_ << std::endl;
        output << "\t-IMU frequency: " << settings.imuFrequency_ << std::endl;
    }

    if (settings.sensor_ == System::RGBD ||
        settings.sensor_ == System::IMU_RGBD) {
        output << "\t-RGB-D depth map factor: " << settings.depthMapFactor_
               << std::endl;
    }

    output << "\t-Features per image: " << settings.nFeatures_ << std::endl;
    output << "\t-ORB scale factor: " << settings.scaleFactor_ << std::endl;
    output << "\t-ORB number of scales: " << settings.nLevels_ << std::endl;
    output << "\t-Initial FAST threshold: " << settings.initThFAST_
           << std::endl;
    output << "\t-Min FAST threshold: " << settings.minThFAST_ << std::endl;

    return output;
}
};  // namespace autonomy::localization::atlas
