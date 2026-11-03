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

#include "atlas/tracking/tracker.hpp"
#include "autolink/common/log.hpp"

#include "atlas/common/math/converter.hpp"
#include "atlas/common/math/geometric_tools.hpp"
#include "atlas/common/optimization/g2o_types.hpp"
#include "atlas/common/optimization/optimizer.hpp"
#include "atlas/frame_drawer.hpp"
#include "atlas/match/ml_pnp_solver.hpp"
#include "atlas/match/orb_matcher.hpp"
#include "atlas/sensor/camera/kannala_brandt_camera.hpp"
#include "atlas/sensor/camera/pinhole_camera.hpp"
#include "atlas/tracking/keyframe_admission_policy.hpp"
#include "atlas/tracking/local_map_tracking_transaction.hpp"
#include "atlas/tracking/recovery_policy.hpp"
#include "atlas/tracking/rgbd_map_reanchor_matcher.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace autonomy::localization::atlas {
namespace {

const char* TrackingStateName(int state) {
    switch (state) {
        case Tracker::SYSTEM_NOT_READY:
            return "SYSTEM_NOT_READY";
        case Tracker::NO_IMAGES_YET:
            return "NO_IMAGES_YET";
        case Tracker::NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case Tracker::OK:
            return "OK";
        case Tracker::RECENTLY_LOST:
            return "RECENTLY_LOST";
        case Tracker::LOST:
            return "LOST";
        case Tracker::OK_KLT:
            return "OK_KLT";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

void Tracker::LogVisualSlamEvent(const char* event) {
    const Eigen::Vector3f t = mCurrentFrame.isSet()
                                  ? mCurrentFrame.GetCameraCenter()
                                  : Eigen::Vector3f::Zero();
    AINFO << "ATLAS_TRACK event=" << event
          << " state=" << TrackingStateName(mState)
          << " inliers=" << mnMatchesInliers
          << " frame=" << mCurrentFrame.mnId << " last_kf="
          << (mpLastKeyFrame ? static_cast<int>(mpLastKeyFrame->mnId) : -1)
          << " kfs=" << (mpAtlas ? mpAtlas->KeyFramesInMap() : 0) << " pose=["
          << t.x() << "," << t.y() << "," << t.z() << "]";
}

Tracker::Tracker(System* pSys, Vocabulary* pVoc, FrameDrawer* pFrameDrawer,
                 MapDrawer* pMapDrawer, Atlas* pAtlas, PlaceRecognizer* pKFDB,
                 const std::string& strSettingPath, const int sensor,
                 ConfigLoader* settings, const std::string& _nameSeq)
    : mState(NO_IMAGES_YET),
      mSensor(sensor),
      mTrackedFr(0),
      mbStep(false),
      mbOnlyTracking(false),
      imu_rgbd_visual_recovery_active_(false),
      mbMapUpdated(false),
      mbVO(false),
      mpVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      mbReadyToInitializate(false),
      mpSystem(pSys),
      mpViewer(nullptr),
      bStepByStep(false),
      mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer),
      mpAtlas(pAtlas),
      mnLastRelocFrameId(0),
      time_recently_lost(5.0),
      mnInitialFrameId(0),
      mbCreatedMap(false),
      mnFirstFrameId(0),
      mpCamera2(nullptr),
      mpLastKeyFrame(static_cast<KeyFrame*>(nullptr)) {
    // Load camera parameters from settings file
    if (settings) {
        newParameterLoader(settings);
    } else {
        cv::FileStorage fConfigLoader(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fConfigLoader);
        if (!b_parse_cam) {
            AINFO << "*Error with the camera parameters in the config file*"
                  << std::endl;
        }

        // Load ORB parameters
        bool b_parse_orb = ParseORBParamFile(fConfigLoader);
        if (!b_parse_orb) {
            AINFO << "*Error with the ORB parameters in the config file*"
                  << std::endl;
        }

        bool b_parse_imu = true;
        if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
            sensor == System::IMU_RGBD) {
            b_parse_imu = ParseIMUParamFile(fConfigLoader);
            if (!b_parse_imu) {
                AINFO << "*Error with the IMU parameters in the config file*"
                      << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        if (!b_parse_cam || !b_parse_orb || !b_parse_imu) {
            AERROR << "**ERROR in the config file, the format is not correct**"
                   << std::endl;
            try {
                throw -1;
            } catch (const std::exception& e) {
            }
        }
    }

    initID = 0;
    lastID = 0;
    mbInitWith3KFs = false;
    mnNumDataset = 0;

    std::vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    AINFO << "There are " << vpCams.size() << " cameras in the atlas"
          << std::endl;
    for (GeometricCamera* pCam : vpCams) {
        AINFO << "Camera " << pCam->GetId();
        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
            AINFO << " is pinhole" << std::endl;
        } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
            AINFO << " is fisheye" << std::endl;
        } else {
            AINFO << " is unknown" << std::endl;
        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif
}

#ifdef REGISTER_TIMES
double calcAverage(std::vector<double> v_times) {
    double accum = 0;
    for (double value : v_times) {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(std::vector<double> v_times, double average) {
    double accum = 0;
    for (double value : v_times) {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(std::vector<int> v_values) {
    double accum = 0;
    int total = 0;
    for (double value : v_values) {
        if (value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(std::vector<int> v_values, double average) {
    double accum = 0;
    int total = 0;
    for (double value : v_values) {
        if (value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracker::LocalMapStats2File() {
    std::ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << std::fixed << std::setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF "
         "culling[ms], Total[ms]"
      << std::endl;
    for (int i = 0; i < mpLocalMapper->vdLMTotal_ms.size(); ++i) {
        f << mpLocalMapper->vdKFInsert_ms[i] << ","
          << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << ","
          << mpLocalMapper->vdLBASync_ms[i] << ","
          << mpLocalMapper->vdKFCullingSync_ms[i] << ","
          << mpLocalMapper->vdLMTotal_ms[i] << std::endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << std::fixed << std::setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << std::endl;
    for (int i = 0; i < mpLocalMapper->vdLBASync_ms.size(); ++i) {
        f << mpLocalMapper->vdLBASync_ms[i] << ","
          << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << ","
          << mpLocalMapper->vnLBA_MPs[i] << "," << mpLocalMapper->vnLBA_edges[i]
          << std::endl;
    }

    f.close();
}

void Tracker::TrackStats2File() {
    std::ofstream f;
    f.open("SessionInfo.txt");
    f << std::fixed;
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << std::endl;

    f << "OpenCV version: " << CV_VERSION << std::endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << std::fixed << std::setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], "
         "IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
      << std::endl;

    for (int i = 0; i < vdTrackTotal_ms.size(); ++i) {
        double stereo_rect = 0.0;
        if (!vdRectStereo_ms.empty()) {
            stereo_rect = vdRectStereo_ms[i];
        }

        double resize_image = 0.0;
        if (!vdResizeImage_ms.empty()) {
            resize_image = vdResizeImage_ms[i];
        }

        double stereo_match = 0.0;
        if (!vdStereoMatch_ms.empty()) {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if (!vdIMUInteg_ms.empty()) {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i]
          << "," << stereo_match << "," << imu_preint << "," << vdPosePred_ms[i]
          << "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << ","
          << vdTrackTotal_ms[i] << std::endl;
    }

    f.close();
}

void Tracker::PrintTimeStats() {
    // Save data in files
    TrackStats2File();
    LocalMapStats2File();

    std::ofstream f;
    f.open("ExecMean.txt");
    f << std::fixed;
    // Report the mean and std of each one
    AINFO << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    AINFO << "OpenCV version: " << CV_VERSION << std::endl;
    f << "OpenCV version: " << CV_VERSION << std::endl;
    AINFO << "---------------------------" << std::endl;
    AINFO << "Tracker" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracker" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if (!vdRectStereo_ms.empty()) {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        AINFO << "Stereo Rectification: " << average << "$\\pm$" << deviation
              << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!vdResizeImage_ms.empty()) {
        average = calcAverage(vdResizeImage_ms);
        deviation = calcDeviation(vdResizeImage_ms, average);
        AINFO << "Image Resize: " << average << "$\\pm$" << deviation
              << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    AINFO << "ORB Extraction: " << average << "$\\pm$" << deviation
          << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if (!vdStereoMatch_ms.empty()) {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        AINFO << "Stereo Matching: " << average << "$\\pm$" << deviation
              << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!vdIMUInteg_ms.empty()) {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        AINFO << "IMU Preintegration: " << average << "$\\pm$" << deviation
              << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    AINFO << "Pose Prediction: " << average << "$\\pm$" << deviation
          << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    AINFO << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    AINFO << "New KF decision: " << average << "$\\pm$" << deviation
          << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    AINFO << "Total Tracker: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Tracker: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    AINFO << std::endl << std::endl << std::endl;
    AINFO << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    AINFO << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    AINFO << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    AINFO << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLBA_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
    AINFO << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdKFCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
    AINFO << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    AINFO << "Total Local Mapping: " << average << "$\\pm$" << deviation
          << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation
      << std::endl;

    // Local Mapping LBA complexity
    AINFO << "---------------------------" << std::endl;
    AINFO << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    AINFO << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    AINFO << "LBA KF optimized: " << average << "$\\pm$" << deviation
          << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    AINFO << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    AINFO << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
          << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;

    AINFO << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    AINFO << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // Map complexity
    AINFO << "---------------------------" << std::endl;
    AINFO << std::endl << "Map complexity" << std::endl;
    AINFO << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
    AINFO << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBestMap = vpMaps[0];
    for (int i = 1; i < vpMaps.size(); ++i) {
        if (pBestMap->GetAllKeyFrames().size() <
            vpMaps[i]->GetAllKeyFrames().size()) {
            pBestMap = vpMaps[i];
        }
    }

    f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    AINFO << "---------------------------" << std::endl;
    AINFO << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopCloser->vdDataQuery_ms);
    deviation = calcDeviation(mpLoopCloser->vdDataQuery_ms, average);
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Database Query: " << average << "$\\pm$" << deviation
          << std::endl;
    average = calcAverage(mpLoopCloser->vdEstSim3_ms);
    deviation = calcDeviation(mpLoopCloser->vdEstSim3_ms, average);
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "SE3 estimation: " << average << "$\\pm$" << deviation
          << std::endl;
    average = calcAverage(mpLoopCloser->vdPRTotal_ms);
    deviation = calcDeviation(mpLoopCloser->vdPRTotal_ms, average);
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation
      << std::endl
      << std::endl;
    AINFO << "Total Place Recognition: " << average << "$\\pm$" << deviation
          << std::endl
          << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    AINFO << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopCloser->vdLoopFusion_ms);
    deviation = calcDeviation(mpLoopCloser->vdLoopFusion_ms, average);
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopCloser->vdLoopOptEss_ms);
    deviation = calcDeviation(mpLoopCloser->vdLoopOptEss_ms, average);
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Essential Graph: " << average << "$\\pm$" << deviation
          << std::endl;
    average = calcAverage(mpLoopCloser->vdLoopTotal_ms);
    deviation = calcDeviation(mpLoopCloser->vdLoopTotal_ms, average);
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    AINFO << "Total Loop Closing: " << average << "$\\pm$" << deviation
          << std::endl
          << std::endl;

    f << "Numb exec: " << mpLoopCloser->nLoop << std::endl;
    AINFO << "Num exec: " << mpLoopCloser->nLoop << std::endl;
    average = calcAverage(mpLoopCloser->vnLoopKFs);
    deviation = calcDeviation(mpLoopCloser->vnLoopKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    AINFO << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopCloser->vdMergeMaps_ms);
    deviation = calcDeviation(mpLoopCloser->vdMergeMaps_ms, average);
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopCloser->vdWeldingBA_ms);
    deviation = calcDeviation(mpLoopCloser->vdWeldingBA_ms, average);
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopCloser->vdMergeOptEss_ms);
    deviation = calcDeviation(mpLoopCloser->vdMergeOptEss_ms, average);
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Optimization Ess.: " << average << "$\\pm$" << deviation
          << std::endl;
    average = calcAverage(mpLoopCloser->vdMergeTotal_ms);
    deviation = calcDeviation(mpLoopCloser->vdMergeTotal_ms, average);
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    AINFO << "Total Map Merging: " << average << "$\\pm$" << deviation
          << std::endl
          << std::endl;

    f << "Numb exec: " << mpLoopCloser->nMerges << std::endl;
    AINFO << "Num exec: " << mpLoopCloser->nMerges << std::endl;
    average = calcAverage(mpLoopCloser->vnMergeKFs);
    deviation = calcDeviation(mpLoopCloser->vnMergeKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopCloser->vnMergeMPs);
    deviation = calcDeviation(mpLoopCloser->vnMergeMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    AINFO << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopCloser->vdGBA_ms);
    deviation = calcDeviation(mpLoopCloser->vdGBA_ms, average);
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopCloser->vdUpdateMap_ms);
    deviation = calcDeviation(mpLoopCloser->vdUpdateMap_ms, average);
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopCloser->vdFGBATotal_ms);
    deviation = calcDeviation(mpLoopCloser->vdFGBATotal_ms, average);
    f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    AINFO << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
          << std::endl;

    f << "Numb exec: " << mpLoopCloser->nFGBA_exec << std::endl;
    AINFO << "Num exec: " << mpLoopCloser->nFGBA_exec << std::endl;
    f << "Numb abort: " << mpLoopCloser->nFGBA_abort << std::endl;
    AINFO << "Num abort: " << mpLoopCloser->nFGBA_abort << std::endl;
    average = calcAverage(mpLoopCloser->vnGBAKFs);
    deviation = calcDeviation(mpLoopCloser->vnGBAKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopCloser->vnGBAMPs);
    deviation = calcDeviation(mpLoopCloser->vnGBAMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    AINFO << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f.close();
}

#endif

Tracker::~Tracker() {
    // f_track_stats.close();
}

void Tracker::newParameterLoader(ConfigLoader* settings) {
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if (settings->needToUndistort()) {
        mDistCoef = settings->camera1DistortionCoef();
    } else {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = cv::Mat::eye(3, 3, CV_32F);
    mK.at<float>(0, 0) = mpCamera->getParameter(0);
    mK.at<float>(1, 1) = mpCamera->getParameter(1);
    mK.at<float>(0, 2) = mpCamera->getParameter(2);
    mK.at<float>(1, 2) = mpCamera->getParameter(3);

    mK_.setIdentity();
    mK_(0, 0) = mpCamera->getParameter(0);
    mK_(1, 1) = mpCamera->getParameter(1);
    mK_(0, 2) = mpCamera->getParameter(2);
    mK_(1, 2) = mpCamera->getParameter(3);

    if ((mSensor == System::STEREO || mSensor == System::IMU_STEREO) &&
        settings->cameraType() == ConfigLoader::KannalaBrandt) {
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }

    if (mSensor == System::STEREO || mSensor == System::RGBD ||
        mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
        mDepthMapFactor = settings->depthMapFactor();
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    // ORB parameters
    int nFeatures = settings->nFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpOrbExtractorLeft = new OrbExtractor(nFeatures, fScaleFactor, nLevels,
                                          fIniThFAST, fMinThFAST);

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
        mpOrbExtractorRight = new OrbExtractor(nFeatures, fScaleFactor, nLevels,
                                               fIniThFAST, fMinThFAST);

    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
        mpIniOrbExtractor = new OrbExtractor(5 * nFeatures, fScaleFactor,
                                             nLevels, fIniThFAST, fMinThFAST);

    // IMU parameters
    autonomy::common::math::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuInitKeyFrameMinInterval = settings->imuInitKeyFrameMinInterval();
    mImuPer = 0.001;  // 1.0 / static_cast<double>(mImuFreq);     //TODO: ESTO
                      // ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF =
        new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
}

bool Tracker::ParseCamParamFile(cv::FileStorage& fConfigLoader) {
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    AINFO << std::endl << "Camera Parameters: " << std::endl;
    bool b_miss_params = false;

    std::string sCameraName = fConfigLoader["Camera.type"];
    if (sCameraName == "PinHole") {
        float fx = 0.F, fy = 0.F, cx = 0.F, cy = 0.F;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fConfigLoader["Camera.fx"];
        if (!node.empty() && node.isReal()) {
            fx = node.real();
        } else {
            AERROR
                << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.fy"];
        if (!node.empty() && node.isReal()) {
            fy = node.real();
        } else {
            AERROR
                << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.cx"];
        if (!node.empty() && node.isReal()) {
            cx = node.real();
        } else {
            AERROR
                << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.cy"];
        if (!node.empty() && node.isReal()) {
            cy = node.real();
        } else {
            AERROR
                << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fConfigLoader["Camera.k1"];
        if (!node.empty() && node.isReal()) {
            mDistCoef.at<float>(0) = node.real();
        } else {
            AERROR
                << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.k2"];
        if (!node.empty() && node.isReal()) {
            mDistCoef.at<float>(1) = node.real();
        } else {
            AERROR
                << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.p1"];
        if (!node.empty() && node.isReal()) {
            mDistCoef.at<float>(2) = node.real();
        } else {
            AERROR
                << "*Camera.p1 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.p2"];
        if (!node.empty() && node.isReal()) {
            mDistCoef.at<float>(3) = node.real();
        } else {
            AERROR
                << "*Camera.p2 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.k3"];
        if (!node.empty() && node.isReal()) {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        node = fConfigLoader["Camera.imageScale"];
        if (!node.empty() && node.isReal()) {
            mImageScale = node.real();
        }

        if (b_miss_params) {
            return false;
        }

        if (mImageScale != 1.f) {
            // K matrix parameters must be scaled.
            fx = fx * mImageScale;
            fy = fy * mImageScale;
            cx = cx * mImageScale;
            cy = cy * mImageScale;
        }

        std::vector<float> vCamCalib{fx, fy, cx, cy};

        mpCamera = new PinholeCamera(vCamCalib);

        mpCamera = mpAtlas->AddCamera(mpCamera);

        AINFO << "- Camera: PinholeCamera" << std::endl;
        AINFO << "- Image scale: " << mImageScale << std::endl;
        AINFO << "- fx: " << fx << std::endl;
        AINFO << "- fy: " << fy << std::endl;
        AINFO << "- cx: " << cx << std::endl;
        AINFO << "- cy: " << cy << std::endl;
        AINFO << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        AINFO << "- k2: " << mDistCoef.at<float>(1) << std::endl;

        AINFO << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        AINFO << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if (mDistCoef.rows == 5)
            AINFO << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3, 3, CV_32F);
        mK.at<float>(0, 0) = fx;
        mK.at<float>(1, 1) = fy;
        mK.at<float>(0, 2) = cx;
        mK.at<float>(1, 2) = cy;

        mK_.setIdentity();
        mK_(0, 0) = fx;
        mK_(1, 1) = fy;
        mK_(0, 2) = cx;
        mK_(1, 2) = cy;
    } else if (sCameraName == "KannalaBrandt8") {
        float fx = 0.F, fy = 0.F, cx = 0.F, cy = 0.F;
        float k1 = 0.F, k2 = 0.F, k3 = 0.F, k4 = 0.F;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fConfigLoader["Camera.fx"];
        if (!node.empty() && node.isReal()) {
            fx = node.real();
        } else {
            AERROR
                << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
        node = fConfigLoader["Camera.fy"];
        if (!node.empty() && node.isReal()) {
            fy = node.real();
        } else {
            AERROR
                << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.cx"];
        if (!node.empty() && node.isReal()) {
            cx = node.real();
        } else {
            AERROR
                << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.cy"];
        if (!node.empty() && node.isReal()) {
            cy = node.real();
        } else {
            AERROR
                << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fConfigLoader["Camera.k1"];
        if (!node.empty() && node.isReal()) {
            k1 = node.real();
        } else {
            AERROR
                << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
        node = fConfigLoader["Camera.k2"];
        if (!node.empty() && node.isReal()) {
            k2 = node.real();
        } else {
            AERROR
                << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.k3"];
        if (!node.empty() && node.isReal()) {
            k3 = node.real();
        } else {
            AERROR
                << "*Camera.k3 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.k4"];
        if (!node.empty() && node.isReal()) {
            k4 = node.real();
        } else {
            AERROR
                << "*Camera.k4 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fConfigLoader["Camera.imageScale"];
        if (!node.empty() && node.isReal()) {
            mImageScale = node.real();
        }

        if (!b_miss_params) {
            if (mImageScale != 1.f) {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            std::vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
            mpCamera = new KannalaBrandtCamera(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);
            AINFO << "- Camera: Fisheye" << std::endl;
            AINFO << "- Image scale: " << mImageScale << std::endl;
            AINFO << "- fx: " << fx << std::endl;
            AINFO << "- fy: " << fy << std::endl;
            AINFO << "- cx: " << cx << std::endl;
            AINFO << "- cy: " << cy << std::endl;
            AINFO << "- k1: " << k1 << std::endl;
            AINFO << "- k2: " << k2 << std::endl;
            AINFO << "- k3: " << k3 << std::endl;
            AINFO << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3, 3, CV_32F);
            mK.at<float>(0, 0) = fx;
            mK.at<float>(1, 1) = fy;
            mK.at<float>(0, 2) = cx;
            mK.at<float>(1, 2) = cy;

            mK_.setIdentity();
            mK_(0, 0) = fx;
            mK_(1, 1) = fy;
            mK_(0, 2) = cx;
            mK_(1, 2) = cy;
        }

        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD) {
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fConfigLoader["Camera2.fx"];
            if (!node.empty() && node.isReal()) {
                fx = node.real();
            } else {
                AERROR << "*Camera2.fx parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }
            node = fConfigLoader["Camera2.fy"];
            if (!node.empty() && node.isReal()) {
                fy = node.real();
            } else {
                AERROR << "*Camera2.fy parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }

            node = fConfigLoader["Camera2.cx"];
            if (!node.empty() && node.isReal()) {
                cx = node.real();
            } else {
                AERROR << "*Camera2.cx parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }

            node = fConfigLoader["Camera2.cy"];
            if (!node.empty() && node.isReal()) {
                cy = node.real();
            } else {
                AERROR << "*Camera2.cy parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fConfigLoader["Camera2.k1"];
            if (!node.empty() && node.isReal()) {
                k1 = node.real();
            } else {
                AERROR << "*Camera2.k1 parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }
            node = fConfigLoader["Camera2.k2"];
            if (!node.empty() && node.isReal()) {
                k2 = node.real();
            } else {
                AERROR << "*Camera2.k2 parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }

            node = fConfigLoader["Camera2.k3"];
            if (!node.empty() && node.isReal()) {
                k3 = node.real();
            } else {
                AERROR << "*Camera2.k3 parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }

            node = fConfigLoader["Camera2.k4"];
            if (!node.empty() && node.isReal()) {
                k4 = node.real();
            } else {
                AERROR << "*Camera2.k4 parameter doesn't exist or is not a "
                          "real number*"
                       << std::endl;
                b_miss_params = true;
            }

            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fConfigLoader["Camera.lappingBegin"];
            if (!node.empty() && node.isInt()) {
                leftLappingBegin = node.operator int();
            } else {
                AINFO << "WARNING: Camera.lappingBegin not correctly defined"
                      << std::endl;
            }
            node = fConfigLoader["Camera.lappingEnd"];
            if (!node.empty() && node.isInt()) {
                leftLappingEnd = node.operator int();
            } else {
                AINFO << "WARNING: Camera.lappingEnd not correctly defined"
                      << std::endl;
            }
            node = fConfigLoader["Camera2.lappingBegin"];
            if (!node.empty() && node.isInt()) {
                rightLappingBegin = node.operator int();
            } else {
                AINFO << "WARNING: Camera2.lappingBegin not correctly defined"
                      << std::endl;
            }
            node = fConfigLoader["Camera2.lappingEnd"];
            if (!node.empty() && node.isInt()) {
                rightLappingEnd = node.operator int();
            } else {
                AINFO << "WARNING: Camera2.lappingEnd not correctly defined"
                      << std::endl;
            }

            node = fConfigLoader["Tlr"];
            cv::Mat cvTlr;
            if (!node.empty()) {
                cvTlr = node.mat();
                if (cvTlr.rows != 3 || cvTlr.cols != 4) {
                    AERROR
                        << "*Tlr matrix have to be a 3x4 transformation matrix*"
                        << std::endl;
                    b_miss_params = true;
                }
            } else {
                AERROR << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if (!b_miss_params) {
                if (mImageScale != 1.f) {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;

                    leftLappingBegin = leftLappingBegin * mImageScale;
                    leftLappingEnd = leftLappingEnd * mImageScale;
                    rightLappingBegin = rightLappingBegin * mImageScale;
                    rightLappingEnd = rightLappingEnd * mImageScale;
                }

                static_cast<KannalaBrandtCamera*>(mpCamera)->mvLappingArea[0] =
                    leftLappingBegin;
                static_cast<KannalaBrandtCamera*>(mpCamera)->mvLappingArea[1] =
                    leftLappingEnd;

                mpFrameDrawer->both = true;

                std::vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
                mpCamera2 = new KannalaBrandtCamera(vCamCalib2);
                mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                mTlr = Converter::toLieGroup(cvTlr);

                static_cast<KannalaBrandtCamera*>(mpCamera2)->mvLappingArea[0] =
                    rightLappingBegin;
                static_cast<KannalaBrandtCamera*>(mpCamera2)->mvLappingArea[1] =
                    rightLappingEnd;

                AINFO << "- Camera1 Lapping: " << leftLappingBegin << ", "
                      << leftLappingEnd << std::endl;

                AINFO << std::endl << "Camera2 Parameters:" << std::endl;
                AINFO << "- Camera: Fisheye" << std::endl;
                AINFO << "- Image scale: " << mImageScale << std::endl;
                AINFO << "- fx: " << fx << std::endl;
                AINFO << "- fy: " << fy << std::endl;
                AINFO << "- cx: " << cx << std::endl;
                AINFO << "- cy: " << cy << std::endl;
                AINFO << "- k1: " << k1 << std::endl;
                AINFO << "- k2: " << k2 << std::endl;
                AINFO << "- k3: " << k3 << std::endl;
                AINFO << "- k4: " << k4 << std::endl;

                AINFO << "- mTlr: \n" << cvTlr << std::endl;

                AINFO << "- Camera2 Lapping: " << rightLappingBegin << ", "
                      << rightLappingEnd << std::endl;
            }
        }

        if (b_miss_params) {
            return false;
        }

    } else {
        AERROR << "*Not Supported Camera Sensor*" << std::endl;
        AERROR << "Check an example configuration file with the desired sensor"
               << std::endl;
    }

    if (mSensor == System::STEREO || mSensor == System::RGBD ||
        mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        cv::FileNode node = fConfigLoader["Camera.bf"];
        if (!node.empty() && node.isReal()) {
            mbf = node.real();
            if (mImageScale != 1.f) {
                mbf *= mImageScale;
            }
        } else {
            AERROR
                << "*Camera.bf parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
    }

    float fps = fConfigLoader["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    AINFO << "- fps: " << fps << std::endl;

    int nRGB = fConfigLoader["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
        AINFO << "- color order: RGB (ignored if grayscale)" << std::endl;
    else
        AINFO << "- color order: BGR (ignored if grayscale)" << std::endl;

    if (mSensor == System::STEREO || mSensor == System::RGBD ||
        mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fConfigLoader["ThDepth"];
        if (!node.empty() && node.isReal()) {
            mThDepth = node.real();
            mThDepth = mbf * mThDepth / fx;
            AINFO << std::endl
                  << "Depth Threshold (Close/Far Points): " << mThDepth
                  << std::endl;
        } else {
            AERROR
                << "*ThDepth parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
    }

    if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
        cv::FileNode node = fConfigLoader["DepthMapFactor"];
        if (!node.empty() && node.isReal()) {
            mDepthMapFactor = node.real();
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        } else {
            AERROR << "*DepthMapFactor parameter doesn't exist or is not a "
                      "real number*"
                   << std::endl;
            b_miss_params = true;
        }
    }

    if (b_miss_params) {
        return false;
    }

    return true;
}

bool Tracker::ParseORBParamFile(cv::FileStorage& fConfigLoader) {
    bool b_miss_params = false;
    int nFeatures = 0, nLevels = 0, fIniThFAST = 0, fMinThFAST = 0;
    float fScaleFactor = 0.F;

    cv::FileNode node = fConfigLoader["ORBextractor.nFeatures"];
    if (!node.empty() && node.isInt()) {
        nFeatures = node.operator int();
    } else {
        AERROR << "*OrbExtractor.nFeatures parameter doesn't exist or is not "
                  "an integer*"
               << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["ORBextractor.scaleFactor"];
    if (!node.empty() && node.isReal()) {
        fScaleFactor = node.real();
    } else {
        AERROR << "*OrbExtractor.scaleFactor parameter doesn't exist or is not "
                  "a real number*"
               << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["ORBextractor.nLevels"];
    if (!node.empty() && node.isInt()) {
        nLevels = node.operator int();
    } else {
        AERROR << "*OrbExtractor.nLevels parameter doesn't exist or is not an "
                  "integer*"
               << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["ORBextractor.iniThFAST"];
    if (!node.empty() && node.isInt()) {
        fIniThFAST = node.operator int();
    } else {
        AERROR << "*OrbExtractor.iniThFAST parameter doesn't exist or is not "
                  "an integer*"
               << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["ORBextractor.minThFAST"];
    if (!node.empty() && node.isInt()) {
        fMinThFAST = node.operator int();
    } else {
        AERROR << "*OrbExtractor.minThFAST parameter doesn't exist or is not "
                  "an integer*"
               << std::endl;
        b_miss_params = true;
    }

    if (b_miss_params) {
        return false;
    }

    mpOrbExtractorLeft = new OrbExtractor(nFeatures, fScaleFactor, nLevels,
                                          fIniThFAST, fMinThFAST);

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
        mpOrbExtractorRight = new OrbExtractor(nFeatures, fScaleFactor, nLevels,
                                               fIniThFAST, fMinThFAST);

    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
        mpIniOrbExtractor = new OrbExtractor(5 * nFeatures, fScaleFactor,
                                             nLevels, fIniThFAST, fMinThFAST);

    AINFO << std::endl << "ORB Extractor Parameters: " << std::endl;
    AINFO << "- Number of Features: " << nFeatures << std::endl;
    AINFO << "- Scale Levels: " << nLevels << std::endl;
    AINFO << "- Scale Factor: " << fScaleFactor << std::endl;
    AINFO << "- Initial Fast Threshold: " << fIniThFAST << std::endl;
    AINFO << "- Minimum Fast Threshold: " << fMinThFAST << std::endl;

    return true;
}

bool Tracker::ParseIMUParamFile(cv::FileStorage& fConfigLoader) {
    bool b_miss_params = false;

    cv::Mat cvTbc;
    cv::FileNode node = fConfigLoader["Tbc"];
    if (!node.empty()) {
        cvTbc = node.mat();
        if (cvTbc.rows != 4 || cvTbc.cols != 4) {
            AERROR << "*Tbc matrix have to be a 4x4 transformation matrix*"
                   << std::endl;
            b_miss_params = true;
        }
    } else {
        AERROR << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }
    AINFO << std::endl;
    AINFO << "Left camera to Imu Transform (Tbc): " << std::endl
          << cvTbc << std::endl;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    autonomy::common::math::SE3f Tbc(eigTbc);

    node = fConfigLoader["InsertKFsWhenLost"];
    mInsertKFsLost = true;
    if (!node.empty() && node.isInt()) {
        mInsertKFsLost = static_cast<bool>(node.operator int());
    }

    if (!mInsertKFsLost)
        AINFO << "Do not insert keyframes when lost visual tracking "
              << std::endl;

    float Ng, Na, Ngw, Naw;

    node = fConfigLoader["IMU.Frequency"];
    if (!node.empty() && node.isInt()) {
        mImuFreq = node.operator int();
        mImuPer = 0.001;  // 1.0 / static_cast<double>(mImuFreq);
    } else {
        AERROR << "*IMU.Frequency parameter doesn't exist or is not an integer*"
               << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["IMU.InitKeyFrameMinInterval"];
    if (!node.empty()) {
        if (node.isReal()) {
            const double interval_seconds = node.real();
            if (std::isfinite(interval_seconds) && interval_seconds > 0.0) {
                mImuInitKeyFrameMinInterval = interval_seconds;
            } else {
                AERROR << "*IMU.InitKeyFrameMinInterval must be finite and "
                          "greater than zero*"
                       << std::endl;
                b_miss_params = true;
            }
        } else {
            AERROR << "*IMU.InitKeyFrameMinInterval must be a real number*"
                   << std::endl;
            b_miss_params = true;
        }
    }

    node = fConfigLoader["IMU.NoiseGyro"];
    if (!node.empty() && node.isReal()) {
        Ng = node.real();
    } else {
        AERROR
            << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["IMU.NoiseAcc"];
    if (!node.empty() && node.isReal()) {
        Na = node.real();
    } else {
        AERROR
            << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["IMU.GyroWalk"];
    if (!node.empty() && node.isReal()) {
        Ngw = node.real();
    } else {
        AERROR
            << "*IMU.GyroWalk parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["IMU.AccWalk"];
    if (!node.empty() && node.isReal()) {
        Naw = node.real();
    } else {
        AERROR
            << "*IMU.AccWalk parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
    }

    node = fConfigLoader["IMU.fastInit"];
    mFastInit = false;
    if (!node.empty()) {
        mFastInit = static_cast<int>(fConfigLoader["IMU.fastInit"]) != 0;
    }

    if (mFastInit)
        AINFO << "Fast IMU initialization. Acceleration is not checked \n";

    if (b_miss_params) {
        return false;
    }

    const float sf = sqrt(mImuFreq);
    AINFO << std::endl;
    AINFO << "IMU frequency: " << mImuFreq << " Hz" << std::endl;
    AINFO << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << std::endl;
    AINFO << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << std::endl;
    AINFO << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)"
          << std::endl;
    AINFO << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)"
          << std::endl;

    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF =
        new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);

    return true;
}

void Tracker::SetLocalMapper(LocalMapper* pLocalMapper) {
    mpLocalMapper = pLocalMapper;
}

void Tracker::SetLoopCloser(LoopCloser* pLoopCloser) {
    mpLoopCloser = pLoopCloser;
}

void Tracker::SetViewer(Viewer* pViewer) {
    mpViewer = pViewer;
}

void Tracker::SetStepByStep(bool bSet) {
    bStepByStep = bSet;
}

bool Tracker::GetStepByStep() {
    return bStepByStep;
}

autonomy::common::math::SE3f Tracker::GrabImageStereo(
    const cv::Mat& imRectLeft, const cv::Mat& imRectRight,
    const double& timestamp, std::string filename) {
    // AINFO << "GrabImageStereo" << endl;

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImRight = imRectRight;

    if (mImGray.channels() == 3) {
        // AINFO << "Image with 3 channels" << endl;
        if (mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
        }
    } else if (mImGray.channels() == 4) {
        // AINFO << "Image with 4 channels" << endl;
        if (mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
        }
    }

    // AINFO << "Incoming frame creation" << endl;

    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame =
            Frame(mImGray, imGrayRight, timestamp, mpOrbExtractorLeft,
                  mpOrbExtractorRight, mpVocabulary, mK, mDistCoef, mbf,
                  mThDepth, mpCamera);
    else if (mSensor == System::STEREO && mpCamera2)
        mCurrentFrame =
            Frame(mImGray, imGrayRight, timestamp, mpOrbExtractorLeft,
                  mpOrbExtractorRight, mpVocabulary, mK, mDistCoef, mbf,
                  mThDepth, mpCamera, mpCamera2, mTlr);
    else if (mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame =
            Frame(mImGray, imGrayRight, timestamp, mpOrbExtractorLeft,
                  mpOrbExtractorRight, mpVocabulary, mK, mDistCoef, mbf,
                  mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
    else if (mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(
            mImGray, imGrayRight, timestamp, mpOrbExtractorLeft,
            mpOrbExtractorRight, mpVocabulary, mK, mDistCoef, mbf, mThDepth,
            mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

    // AINFO << "Incoming frame ended" << endl;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

    // AINFO << "Tracker start" << endl;
    Track();
    // AINFO << "Tracker end" << endl;

    return mCurrentFrame.GetPose();
}

autonomy::common::math::SE3f Tracker::GrabImageRGBD(const cv::Mat& imRGB,
                                                    const cv::Mat& imD,
                                                    const double& timestamp,
                                                    std::string filename) {
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    if (mSensor == System::RGBD)
        mCurrentFrame =
            Frame(mImGray, imDepth, timestamp, mpOrbExtractorLeft, mpVocabulary,
                  mK, mDistCoef, mbf, mThDepth, mpCamera);
    else if (mSensor == System::IMU_RGBD)
        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpOrbExtractorLeft,
                              mpVocabulary, mK, mDistCoef, mbf, mThDepth,
                              mpCamera, &mLastFrame, *mpImuCalib);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    Track();

    return mCurrentFrame.GetPose();
}

autonomy::common::math::SE3f Tracker::GrabImageMonocular(
    const cv::Mat& im, const double& timestamp, std::string filename) {
    mImGray = im;
    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR) {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET ||
            (lastID - initID) < mMaxFrames)
            mCurrentFrame =
                Frame(mImGray, timestamp, mpIniOrbExtractor, mpVocabulary,
                      mpCamera, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame =
                Frame(mImGray, timestamp, mpOrbExtractorLeft, mpVocabulary,
                      mpCamera, mDistCoef, mbf, mThDepth);
    } else if (mSensor == System::IMU_MONOCULAR) {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) {
            mCurrentFrame = Frame(mImGray, timestamp, mpIniOrbExtractor,
                                  mpVocabulary, mpCamera, mDistCoef, mbf,
                                  mThDepth, &mLastFrame, *mpImuCalib);
        } else
            mCurrentFrame = Frame(mImGray, timestamp, mpOrbExtractorLeft,
                                  mpVocabulary, mpCamera, mDistCoef, mbf,
                                  mThDepth, &mLastFrame, *mpImuCalib);
    }

    if (mState == NO_IMAGES_YET)
        t0 = timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    Track();

    return mCurrentFrame.GetPose();
}

void Tracker::GrabImuData(const IMU::Point& imuMeasurement) {
    std::unique_lock<std::mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracker::PreintegrateIMU() {
    if (!mCurrentFrame.mpPrevFrame) {
        ADEBUG << "No previous frame";
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if (mlQueueImuData.size() == 0) {
        AINFO << "Not IMU data in mlQueueImuData!!";
        mCurrentFrame.setIntegrated();
        return;
    }

    while (true) {
        bool bSleep = false;
        {
            std::unique_lock<std::mutex> lock(mMutexImuQueue);
            if (!mlQueueImuData.empty()) {
                IMU::Point* m = &mlQueueImuData.front();
                if (m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer) {
                    mlQueueImuData.pop_front();
                } else if (m->t < mCurrentFrame.mTimeStamp - mImuPer) {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                } else {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            } else {
                break;
                bSleep = true;
            }
        }
        if (bSleep)
            usleep(500);
    }

    // size()<2 used to underflow (n=-1) or early-return without setIntegrated(),
    // which left mpImuPreintegratedFrame null and crashed tracking / UpdateFrameIMU.
    if (mvImuFromLastFrame.size() < 2) {
        AWARN << "Insufficient IMU measurements for preintegration: size="
              << mvImuFromLastFrame.size();
        IMU::Preintegrated* pImuPreintegratedFromLastFrame =
            new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);
        if (!mvImuFromLastFrame.empty()) {
            const float tstep = static_cast<float>(
                mCurrentFrame.mTimeStamp -
                mCurrentFrame.mpPrevFrame->mTimeStamp);
            if (std::isfinite(tstep) && tstep > 0.f) {
                if (!mpImuPreintegratedFromLastKF) {
                    AINFO << "mpImuPreintegratedFromLastKF does not exist"
                          << std::endl;
                } else {
                    mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(
                        mvImuFromLastFrame.front().a,
                        mvImuFromLastFrame.front().w, tstep);
                }
                pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(
                    mvImuFromLastFrame.front().a, mvImuFromLastFrame.front().w,
                    tstep);
            }
        }
        mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;
        mCurrentFrame.setIntegrated();
        return;
    }

    const int n = static_cast<int>(mvImuFromLastFrame.size()) - 1;

    IMU::Preintegrated* pImuPreintegratedFromLastFrame =
        new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

    for (int i = 0; i < n; i++) {
        float tstep;
        Eigen::Vector3f acc, angVel;
        if ((i == 0) && (i < (n - 1))) {
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tini =
                mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                       (tini / tab)) *
                  0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                          (tini / tab)) *
                     0.5f;
            tstep = mvImuFromLastFrame[i + 1].t -
                    mCurrentFrame.mpPrevFrame->mTimeStamp;
        } else if (i < (n - 1)) {
            acc =
                (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
            angVel =
                (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
            tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
        } else if ((i > 0) && (i == (n - 1))) {
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                       (tend / tab)) *
                  0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                          (tend / tab)) *
                     0.5f;
            tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
        } else if ((i == 0) && (i == (n - 1))) {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp -
                    mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            AINFO << "mpImuPreintegratedFromLastKF does not exist" << std::endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel,
                                                              tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel,
                                                                tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    // AINFO << "Preintegration is finished!! ";
}

bool Tracker::PredictStateIMU() {
    if (!mCurrentFrame.mpPrevFrame) {
        ADEBUG << "No last frame";
        return false;
    }

    if (mbMapUpdated && mpLastKeyFrame) {
        if (!mpImuPreintegratedFromLastKF) {
            AINFO << "not IMU prediction!!" << std::endl;
            return false;
        }
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(
                       mpLastKeyFrame->GetImuBias()));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(
                       mpLastKeyFrame->GetImuBias());
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(
                       mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    } else if (!mbMapUpdated) {
        if (!mCurrentFrame.mpImuPreintegratedFrame) {
            AINFO << "not IMU prediction!!" << std::endl;
            return false;
        }
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(
                       mLastFrame.mImuBias));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(
                       mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(
                       mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    } else
        AINFO << "not IMU prediction!!" << std::endl;

    return false;
}

void Tracker::ResetFrameIMU() {
    // TODO To implement...
}

void Tracker::SoftRecoverAfterTimestampJump() {
    const double delta_seconds =
        mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp;
    AWARN << "Timestamp jump soft recover: delta_sec=" << delta_seconds
          << " last=" << mLastFrame.mTimeStamp
          << " current=" << mCurrentFrame.mTimeStamp
          << "; keeping active map and falling back to visual recovery";
    mbVelocity = false;
    mState = RECENTLY_LOST;
    mTimeStampLost = mLastFrame.mTimeStamp;
    imu_rgbd_visual_recovery_active_ = (mSensor == System::IMU_RGBD);
    // Keep mpImuPreintegratedFromLastKF and the IMU queue so PreintegrateIMU
    // can still bridge the stall when samples kept arriving during Wait*.
}

RgbdTemporalPoseEstimator::FrameView Tracker::CurrentRgbdTemporalFrameView(
    Map* current_map) const {
    RgbdTemporalPoseEstimator::FrameView view;
    if (!current_map) {
        return view;
    }
    view.descriptors = &mCurrentFrame.mDescriptors;
    view.keypoints = &mCurrentFrame.mvKeysUn;
    view.depths = &mCurrentFrame.mvDepth;
    view.intrinsics = mCurrentFrame.mK_;
    view.minimum_x = Frame::mnMinX;
    view.maximum_x = Frame::mnMaxX;
    view.minimum_y = Frame::mnMinY;
    view.maximum_y = Frame::mnMaxY;
    view.timestamp = mCurrentFrame.mTimeStamp;
    view.frame_id = static_cast<std::uint64_t>(mCurrentFrame.mnId);
    view.map_id = static_cast<std::uint64_t>(current_map->GetId());
    view.coordinate_epoch = current_map->GetCoordinateEpoch();
    view.map_change_index = current_map->GetMapChangeIndex();
    return view;
}

void Tracker::LogRgbdTemporalMapPointShadow(
    const RgbdTemporalPoseEstimator::Result& result, Map* current_map) const {
    if (!current_map || result.pnp_inlier_pairs.empty()) {
        return;
    }

    const RgbdTemporalPoseEstimator::Diagnostics& diagnostics =
        result.diagnostics;
    const std::uint64_t map_id =
        static_cast<std::uint64_t>(current_map->GetId());
    const std::uint64_t coordinate_epoch = current_map->GetCoordinateEpoch();
    const int map_revision = current_map->GetMapChangeIndex();
    const bool identity_valid =
        mLastFrame.isSet() &&
        diagnostics.reference_frame_id ==
            static_cast<std::uint64_t>(mLastFrame.mnId) &&
        diagnostics.current_frame_id ==
            static_cast<std::uint64_t>(mCurrentFrame.mnId) &&
        diagnostics.reference_map_id == map_id &&
        diagnostics.current_map_id == map_id &&
        diagnostics.reference_coordinate_epoch == coordinate_epoch &&
        diagnostics.current_coordinate_epoch == coordinate_epoch &&
        diagnostics.reference_map_revision == map_revision &&
        diagnostics.current_map_revision == map_revision;

    std::size_t source_nonnull = 0;
    std::size_t replaced = 0;
    std::size_t bad = 0;
    std::size_t wrong_map = 0;
    std::size_t zero_observations = 0;
    std::size_t duplicate = 0;
    std::size_t unique_transferable = 0;
    std::unordered_set<std::uint64_t> transferable_ids;
    transferable_ids.reserve(result.pnp_inlier_pairs.size());
    constexpr int kGridColumns = 4;
    constexpr int kGridRows = 4;
    std::array<bool, kGridColumns * kGridRows> occupied_cells{};

    if (identity_valid) {
        for (const RgbdTemporalPoseEstimator::PnpInlierPair& pair :
             result.pnp_inlier_pairs) {
            if (pair.reference_original_index < 0 || pair.current_index < 0 ||
                static_cast<std::size_t>(pair.reference_original_index) >=
                    mLastFrame.mvpMapPoints.size() ||
                static_cast<std::size_t>(pair.current_index) >=
                    mCurrentFrame.mvKeysUn.size()) {
                continue;
            }
            MapPoint* map_point =
                mLastFrame.mvpMapPoints[pair.reference_original_index];
            if (!map_point) {
                continue;
            }
            ++source_nonnull;
            MapPoint* replacement = map_point->GetReplaced();
            if (replacement) {
                ++replaced;
                map_point = replacement;
            }
            if (map_point->isBad()) {
                ++bad;
                continue;
            }
            if (map_point->GetMap() != current_map) {
                ++wrong_map;
                continue;
            }
            if (map_point->Observations() <= 0) {
                ++zero_observations;
                continue;
            }
            if (!transferable_ids.insert(map_point->mnId).second) {
                ++duplicate;
                continue;
            }
            ++unique_transferable;

            const cv::Point2f& point =
                mCurrentFrame.mvKeysUn[pair.current_index].pt;
            const float width = Frame::mnMaxX - Frame::mnMinX;
            const float height = Frame::mnMaxY - Frame::mnMinY;
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                width <= 0.0F || height <= 0.0F || point.x < Frame::mnMinX ||
                point.x > Frame::mnMaxX || point.y < Frame::mnMinY ||
                point.y > Frame::mnMaxY) {
                continue;
            }
            const int column =
                std::clamp(static_cast<int>((point.x - Frame::mnMinX) / width *
                                            kGridColumns),
                           0, kGridColumns - 1);
            const int row =
                std::clamp(static_cast<int>((point.y - Frame::mnMinY) / height *
                                            kGridRows),
                           0, kGridRows - 1);
            occupied_cells[row * kGridColumns + column] = true;
        }
    }

    const int seed_grid = static_cast<int>(
        std::count(occupied_cells.begin(), occupied_cells.end(), true));
    AINFO << "ATLAS_RGBD_TEMPORAL_PNP_MP_SHADOW"
          << " pairs=" << result.pnp_inlier_pairs.size()
          << " source_nonnull=" << source_nonnull << " replaced=" << replaced
          << " bad=" << bad << " wrong_map=" << wrong_map
          << " zero_obs=" << zero_observations << " duplicate=" << duplicate
          << " unique_transferable=" << unique_transferable
          << " seed_grid=" << seed_grid
          << " identity_valid=" << (identity_valid ? 1 : 0) << std::endl;
}

// TODO: Enable System::RGBD only after its no-vote recovery path can seed a
// local keyframe window. The current recovery seeding policy is intentionally
// specific to IMU_RGBD.
bool Tracker::TryRgbdTemporalPose(
    Map* current_map, RgbdTemporalPoseEstimator::Quality* quality,
    RgbdTemporalPoseEstimator::Result* accepted_result,
    internal::LocalMapTrackingTransaction* transaction) {
    if (mSensor != System::IMU_RGBD || !current_map || !accepted_result ||
        !transaction) {
        return false;
    }
    if (current_map != mpAtlas->GetCurrentMap()) {
        rgbd_temporal_odometry_state_.Reset();
        rgbd_temporal_pose_estimator_.Reset();
        return false;
    }
    const RgbdTemporalPoseEstimator::FrameView frame_view =
        CurrentRgbdTemporalFrameView(current_map);
    if (!rgbd_temporal_odometry_state_.active) {
        if (!mLastFrame.isSet()) {
            return false;
        }
        const double start_timestamp =
            mState == RECENTLY_LOST ? mTimeStampLost : frame_view.timestamp;
        rgbd_temporal_odometry_state_.Start(
            start_timestamp, mLastFrame.mTimeStamp, frame_view.map_id,
            frame_view.coordinate_epoch, frame_view.map_change_index);
    }
    if (!rgbd_temporal_odometry_state_.CanEstimate(
            frame_view.timestamp, frame_view.map_id,
            frame_view.coordinate_epoch, frame_view.map_change_index)) {
        rgbd_temporal_odometry_state_.Reset();
        rgbd_temporal_pose_estimator_.Reset();
        return false;
    }
    const RgbdTemporalPoseEstimator::Result result =
        rgbd_temporal_pose_estimator_.Estimate(frame_view);
    const RgbdTemporalPoseEstimator::Diagnostics& diagnostics =
        result.diagnostics;
    AINFO << "ATLAS_RGBD_TEMPORAL_PNP_ATTEMPT"
          << " success=" << (result.success ? 1 : 0) << " stage="
          << RgbdTemporalPoseEstimator::StageName(diagnostics.stage)
          << " rejection="
          << RgbdTemporalPoseEstimator::RejectionReasonName(
                 diagnostics.rejection)
          << " reference_available="
          << (diagnostics.reference_available ? 1 : 0)
          << " ref_features=" << diagnostics.reference_feature_count
          << " current_features=" << diagnostics.current_feature_count
          << " mutual_matches=" << diagnostics.mutual_match_count
          << " guided_matches=" << diagnostics.guided_match_count
          << " final_matches=" << diagnostics.final_match_count
          << " pnp_inliers=" << diagnostics.pnp_inlier_count
          << " accepted_inliers=" << diagnostics.inlier_count
          << " rigid_depth_pairs=" << diagnostics.rigid_depth_pair_count
          << " rigid_inliers=" << diagnostics.rigid_inlier_count
          << " rigid_five_cm_inliers="
          << diagnostics.rigid_five_centimeter_inlier_count
          << " rigid_inlier_ratio=" << diagnostics.rigid_inlier_ratio
          << " rigid_rmse_m=" << diagnostics.rigid_rmse_meters
          << " rigid_median_m=" << diagnostics.rigid_xyz_median_meters
          << " grid_cells=" << diagnostics.covered_grid_cells
          << " depth_checked=" << diagnostics.depth_check_count
          << " depth_consistent=" << diagnostics.depth_consistent_count
          << " xyz_checked=" << diagnostics.xyz_check_count
          << " xyz_consistent=" << diagnostics.xyz_consistent_count
          << " xyz_rmse_m=" << diagnostics.xyz_rmse_meters
          << " used_rigid_pose="
          << (diagnostics.used_rigid_pose ? 1 : 0)
          << " used_pose_guided_matching="
          << (diagnostics.used_pose_guided_matching ? 1 : 0)
          << " reprojection_rmse_px=" << diagnostics.reprojection_rmse_pixels
          << " pose_condition=" << diagnostics.pose_condition_number
          << " rigid_retry_attempted="
          << (diagnostics.rigid_retry.attempted ? 1 : 0)
          << " rigid_retry_stage="
          << RgbdTemporalPoseEstimator::StageName(
                 diagnostics.rigid_retry.stage)
          << " rigid_retry_rejection="
          << RgbdTemporalPoseEstimator::RejectionReasonName(
                 diagnostics.rigid_retry.rejection)
          << " rigid_retry_guided_matches="
          << diagnostics.rigid_retry.guided_match_count
          << " rigid_retry_final_matches="
          << diagnostics.rigid_retry.final_match_count
          << " rigid_retry_depth_pairs="
          << diagnostics.rigid_retry.rigid_depth_pair_count
          << " rigid_retry_inliers="
          << diagnostics.rigid_retry.rigid_inlier_count
          << " rigid_retry_five_cm_inliers="
          << diagnostics.rigid_retry.rigid_five_centimeter_inlier_count
          << " rigid_retry_inlier_ratio="
          << diagnostics.rigid_retry.rigid_inlier_ratio
          << " rigid_retry_rmse_m="
          << diagnostics.rigid_retry.rigid_rmse_meters
          << " rigid_retry_median_m="
          << diagnostics.rigid_retry.rigid_xyz_median_meters
          << " rigid_retry_grid_cells="
          << diagnostics.rigid_retry.covered_grid_cells
          << " rigid_retry_depth_checked="
          << diagnostics.rigid_retry.depth_check_count
          << " rigid_retry_depth_consistent="
          << diagnostics.rigid_retry.depth_consistent_count
          << " rigid_retry_xyz_checked="
          << diagnostics.rigid_retry.xyz_check_count
          << " rigid_retry_xyz_consistent="
          << diagnostics.rigid_retry.xyz_consistent_count
          << " rigid_retry_xyz_rmse_m="
          << diagnostics.rigid_retry.xyz_rmse_meters
          << " rigid_retry_reprojection_rmse_px="
          << diagnostics.rigid_retry.reprojection_rmse_pixels
          << " rigid_retry_pose_condition="
          << diagnostics.rigid_retry.pose_condition_number
          << " ref_map_id=" << diagnostics.reference_map_id
          << " current_map_id=" << diagnostics.current_map_id
          << " ref_epoch=" << diagnostics.reference_coordinate_epoch
          << " current_epoch=" << diagnostics.current_coordinate_epoch
          << " ref_revision=" << diagnostics.reference_map_revision
          << " current_revision=" << diagnostics.current_map_revision
          << " frame_id=" << mCurrentFrame.mnId << std::endl;
    LogRgbdTemporalMapPointShadow(result, current_map);
    if (!result.success) {
        return false;
    }
    const bool low_coverage =
        result.quality ==
        RgbdTemporalPoseEstimator::Quality::kLowCoverageFallback;
    if (!rgbd_temporal_odometry_state_.CanAccept(low_coverage) ||
        diagnostics.reference_frame_id !=
            static_cast<std::uint64_t>(mLastFrame.mnId)) {
        return false;
    }
    if (quality) {
        *quality = result.quality;
    }
    *accepted_result = result;
    transaction->Reserve(
        mvpLocalMapPoints.size() + mCurrentFrame.mvpMapPoints.size(),
        mvpLocalKeyFrames.size());
    const auto prepare_retry_point = [&](MapPoint* map_point) {
        transaction->Capture(map_point);
        if (!map_point) {
            return;
        }
        if (map_point->mnLastFrameSeen == mCurrentFrame.mnId) {
            map_point->mnLastFrameSeen = mLastFrame.mnId;
        }
        if (map_point->mnTrackReferenceForFrame == mCurrentFrame.mnId) {
            map_point->mnTrackReferenceForFrame = mLastFrame.mnId;
        }
        map_point->mbTrackInView = false;
        map_point->mbTrackInViewR = false;
    };
    for (MapPoint* map_point : mvpLocalMapPoints) {
        prepare_retry_point(map_point);
    }
    for (MapPoint* map_point : mCurrentFrame.mvpMapPoints) {
        prepare_retry_point(map_point);
    }
    for (KeyFrame* keyframe : mvpLocalKeyFrames) {
        transaction->Capture(keyframe);
        if (keyframe &&
            keyframe->mnTrackReferenceForFrame == mCurrentFrame.mnId) {
            keyframe->mnTrackReferenceForFrame = mLastFrame.mnId;
        }
    }

    mCurrentFrame.SetPose(result.camera_from_world);
    mCurrentFrame.mvpMapPoints.assign(mCurrentFrame.N, nullptr);
    mCurrentFrame.mvbOutlier.assign(mCurrentFrame.N, false);
    mCurrentFrame.mmProjectPoints.clear();
    mCurrentFrame.mmMatchedInImage.clear();
    return true;
}

bool Tracker::AdvanceRgbdTemporalOdometryReference(
    Map* current_map, RgbdTemporalPoseEstimator::Quality quality) {
    if (!current_map || !mCurrentFrame.isSet() ||
        !rgbd_temporal_odometry_state_.active) {
        return false;
    }
    if (current_map != mpAtlas->GetCurrentMap()) {
        rgbd_temporal_odometry_state_.Reset();
        rgbd_temporal_pose_estimator_.Reset();
        return false;
    }
    const RgbdTemporalPoseEstimator::FrameView view =
        CurrentRgbdTemporalFrameView(current_map);
    if (!rgbd_temporal_odometry_state_.CanEstimate(view.timestamp, view.map_id,
                                                   view.coordinate_epoch,
                                                   view.map_change_index) ||
        !rgbd_temporal_pose_estimator_.UpdateReference(
            view, mCurrentFrame.GetPose())) {
        rgbd_temporal_odometry_state_.Reset();
        rgbd_temporal_pose_estimator_.Reset();
        return false;
    }
    rgbd_temporal_odometry_state_.Advance(
        view.timestamp,
        quality == RgbdTemporalPoseEstimator::Quality::kLowCoverageFallback);
    return true;
}

bool Tracker::RebaseRgbdTemporalReferenceAfterMapUpdate(
    Map* current_map, int previous_map_revision, int current_map_revision) {
    if (mSensor != System::IMU_RGBD) {
        return false;
    }
    if (!rgbd_temporal_pose_estimator_.HasReference()) {
        rgbd_temporal_odometry_state_.Reset();
        return false;
    }

    const auto reject_rebase = [&](const char* reason) {
        AINFO << "ATLAS_RGBD_TEMPORAL_REFERENCE_REBASE"
              << " success=0"
              << " reason=" << reason
              << " previous_revision=" << previous_map_revision
              << " current_revision=" << current_map_revision
              << " frame_id=" << mLastFrame.mnId << std::endl;
        rgbd_temporal_odometry_state_.Reset();
        rgbd_temporal_pose_estimator_.Reset();
        return false;
    };

    if (!current_map || current_map != mpAtlas->GetCurrentMap() ||
        current_map_revision <= previous_map_revision) {
        return reject_rebase("invalid_map_transition");
    }
    if (!mLastFrame.isSet() || mlRelativeFramePoses.empty() ||
        mlpReferences.empty() || mlbLost.empty() || mlbLost.back()) {
        return reject_rebase("invalid_last_frame");
    }
    KeyFrame* const reference_keyframe = mLastFrame.mpReferenceKF;
    if (!reference_keyframe || reference_keyframe != mlpReferences.back() ||
        reference_keyframe->isBad() ||
        reference_keyframe->GetMap() != current_map) {
        return reject_rebase("invalid_reference_keyframe");
    }

    const std::uint64_t map_id =
        static_cast<std::uint64_t>(current_map->GetId());
    const std::uint64_t coordinate_epoch = current_map->GetCoordinateEpoch();
    internal::BoundedRgbdTemporalOdometryState rebased_state =
        rgbd_temporal_odometry_state_;
    if (rebased_state.active &&
        !rebased_state.RebaseMapRevision(map_id, coordinate_epoch,
                                         previous_map_revision,
                                         current_map_revision)) {
        return reject_rebase("odometry_state_identity_mismatch");
    }

    const autonomy::common::math::SE3f corrected_camera_from_world =
        mlRelativeFramePoses.back() * reference_keyframe->GetPose();
    if (!rgbd_temporal_pose_estimator_.RebaseReferencePose(
            static_cast<std::uint64_t>(mLastFrame.mnId), map_id,
            coordinate_epoch, previous_map_revision, current_map_revision,
            corrected_camera_from_world)) {
        return reject_rebase("reference_identity_mismatch");
    }

    mLastFrame.SetPose(corrected_camera_from_world);
    rgbd_temporal_odometry_state_ = rebased_state;
    ADEBUG << "ATLAS_RGBD_TEMPORAL_REFERENCE_REBASE"
          << " success=1"
          << " reason=none"
          << " map_id=" << map_id << " coordinate_epoch=" << coordinate_epoch
          << " previous_revision=" << previous_map_revision
          << " current_revision=" << current_map_revision
          << " state_active=" << (rgbd_temporal_odometry_state_.active ? 1 : 0)
          << " frame_id=" << mLastFrame.mnId << std::endl;
    return true;
}

bool Tracker::ValidateAndCommitRgbdTemporalPose(
    Map* expected_map,
    const RgbdTemporalPoseEstimator::Result& accepted_result,
    internal::LocalMapTrackingTransaction* transaction,
    int* validation_inliers) {
    if (!transaction) {
        return false;
    }
    Map* const current_map = mpAtlas->GetCurrentMap();
    if (!current_map || current_map != expected_map) {
        transaction->Restore();
        rgbd_temporal_odometry_state_.Reset();
        rgbd_temporal_pose_estimator_.Reset();
        return false;
    }
    const bool imu_initialized = current_map->isImuInitialized();
    const int required_inliers =
        internal::RequiredRgbdTemporalReanchorInliers(imu_initialized);
    const autonomy::common::math::SE3f provisional_pose =
        mCurrentFrame.GetPose();
    const std::vector<MapPoint*> frame_points = mCurrentFrame.mvpMapPoints;
    const std::vector<MapPoint*> last_frame_points = mLastFrame.mvpMapPoints;
    const std::vector<bool> frame_outliers = mCurrentFrame.mvbOutlier;
    const auto project_points = mCurrentFrame.mmProjectPoints;
    const auto matched_points = mCurrentFrame.mmMatchedInImage;
    KeyFrame* const tracker_reference = mpReferenceKF;
    KeyFrame* const frame_reference = mCurrentFrame.mpReferenceKF;
    const std::vector<MapPoint*> local_points = mvpLocalMapPoints;
    const std::vector<KeyFrame*> local_keyframes = mvpLocalKeyFrames;
    const std::vector<MapPoint*> atlas_points =
        current_map->GetReferenceMapPoints();
    const int previous_inliers = mnMatchesInliers;
    const int previous_mapper_inliers = mpLocalMapper->mnMatchesInliers;
    const int previous_tracked_frames = mTrackedFr;
    const auto restore_frame_and_tracker = [&]() {
        mvpLocalMapPoints = local_points;
        mvpLocalKeyFrames = local_keyframes;
        mpReferenceKF = tracker_reference;
        mCurrentFrame.mpReferenceKF = frame_reference;
        mCurrentFrame.mvpMapPoints = frame_points;
        mLastFrame.mvpMapPoints = last_frame_points;
        mCurrentFrame.mvbOutlier = frame_outliers;
        mCurrentFrame.mmProjectPoints = project_points;
        mCurrentFrame.mmMatchedInImage = matched_points;
        mCurrentFrame.SetPose(provisional_pose);
        mnMatchesInliers = previous_inliers;
        mpLocalMapper->mnMatchesInliers = previous_mapper_inliers;
        mTrackedFr = previous_tracked_frames;
        current_map->SetReferenceMapPoints(atlas_points);
    };

    const RgbdTemporalPoseEstimator::Diagnostics& diagnostics =
        accepted_result.diagnostics;
    const internal::RgbdTemporalSeedIdentity seed_identity = {
        diagnostics.reference_frame_id,
        diagnostics.current_frame_id,
        diagnostics.reference_map_id,
        diagnostics.current_map_id,
        diagnostics.reference_coordinate_epoch,
        diagnostics.current_coordinate_epoch,
        diagnostics.reference_map_revision,
        diagnostics.current_map_revision,
    };
    const internal::RgbdTemporalSeedContext seed_context = {
        mLastFrame.isSet(),
        static_cast<std::uint64_t>(mLastFrame.mnId),
        static_cast<std::uint64_t>(mCurrentFrame.mnId),
        static_cast<std::uint64_t>(current_map->GetId()),
        current_map->GetCoordinateEpoch(),
        current_map->GetMapChangeIndex(),
    };
    if (!internal::HasMatchingRgbdTemporalSeedIdentity(seed_identity,
                                                       seed_context)) {
        AINFO << "ATLAS_RGBD_TEMPORAL_PNP_MP_SEED"
              << " identity_valid=0"
              << " pairs=" << accepted_result.pnp_inlier_pairs.size()
              << " frame_id=" << mCurrentFrame.mnId << std::endl;
        transaction->Restore();
        restore_frame_and_tracker();
        return false;
    }

    constexpr int kMaximumReplacementHops = 16;
    constexpr int kSeedGridColumns = 4;
    constexpr int kSeedGridRows = 4;
    std::unordered_set<std::uint64_t> seeded_map_point_ids;
    std::unordered_set<int> seeded_feature_indices;
    seeded_map_point_ids.reserve(accepted_result.pnp_inlier_pairs.size());
    seeded_feature_indices.reserve(accepted_result.pnp_inlier_pairs.size());
    std::array<bool, kSeedGridColumns * kSeedGridRows> occupied_seed_cells{};
    std::size_t out_of_bounds = 0;
    std::size_t source_null = 0;
    std::size_t replacement_hops = 0;
    std::size_t invalid_replacement_chain = 0;
    std::size_t bad = 0;
    std::size_t wrong_map = 0;
    std::size_t zero_observations = 0;
    std::size_t duplicate_map_point = 0;
    std::size_t duplicate_current_index = 0;
    std::size_t occupied_current_slot = 0;
    std::size_t seeded = 0;
    for (const RgbdTemporalPoseEstimator::PnpInlierPair& pair :
         accepted_result.pnp_inlier_pairs) {
        if (pair.reference_original_index < 0 || pair.current_index < 0 ||
            static_cast<std::size_t>(pair.reference_original_index) >=
                mLastFrame.mvpMapPoints.size() ||
            static_cast<std::size_t>(pair.current_index) >=
                mCurrentFrame.mvpMapPoints.size() ||
            static_cast<std::size_t>(pair.current_index) >=
                mCurrentFrame.mvbOutlier.size() ||
            static_cast<std::size_t>(pair.current_index) >=
                mCurrentFrame.mvKeysUn.size()) {
            ++out_of_bounds;
            continue;
        }

        MapPoint* map_point =
            mLastFrame.mvpMapPoints[pair.reference_original_index];
        if (!map_point) {
            ++source_null;
            continue;
        }

        std::array<MapPoint*, kMaximumReplacementHops + 1>
            replacement_chain{};
        int replacement_chain_size = 1;
        replacement_chain[0] = map_point;
        bool replacement_chain_valid = true;
        while (MapPoint* replacement = map_point->GetReplaced()) {
            if (replacement_chain_size >=
                    static_cast<int>(replacement_chain.size()) ||
                std::find(replacement_chain.begin(),
                          replacement_chain.begin() + replacement_chain_size,
                          replacement) !=
                    replacement_chain.begin() + replacement_chain_size) {
                replacement_chain_valid = false;
                break;
            }
            replacement_chain[replacement_chain_size++] = replacement;
            map_point = replacement;
            ++replacement_hops;
        }
        if (!replacement_chain_valid) {
            ++invalid_replacement_chain;
            continue;
        }
        if (map_point->isBad()) {
            ++bad;
            continue;
        }
        if (map_point->GetMap() != current_map) {
            ++wrong_map;
            continue;
        }
        if (map_point->Observations() <= 0) {
            ++zero_observations;
            continue;
        }
        if (seeded_feature_indices.count(pair.current_index) != 0U) {
            ++duplicate_current_index;
            continue;
        }
        if (mCurrentFrame.mvpMapPoints[pair.current_index]) {
            ++occupied_current_slot;
            continue;
        }
        if (seeded_map_point_ids.count(map_point->mnId) != 0U) {
            ++duplicate_map_point;
            continue;
        }

        transaction->Capture(map_point);
        seeded_feature_indices.insert(pair.current_index);
        seeded_map_point_ids.insert(map_point->mnId);
        mCurrentFrame.mvpMapPoints[pair.current_index] = map_point;
        mCurrentFrame.mvbOutlier[pair.current_index] = false;
        ++seeded;

        const cv::Point2f& point =
            mCurrentFrame.mvKeysUn[pair.current_index].pt;
        const float width = Frame::mnMaxX - Frame::mnMinX;
        const float height = Frame::mnMaxY - Frame::mnMinY;
        if (std::isfinite(point.x) && std::isfinite(point.y) && width > 0.0F &&
            height > 0.0F && point.x >= Frame::mnMinX &&
            point.x <= Frame::mnMaxX && point.y >= Frame::mnMinY &&
            point.y <= Frame::mnMaxY) {
            const int column = std::clamp(
                static_cast<int>((point.x - Frame::mnMinX) / width *
                                 kSeedGridColumns),
                0, kSeedGridColumns - 1);
            const int row = std::clamp(
                static_cast<int>((point.y - Frame::mnMinY) / height *
                                 kSeedGridRows),
                0, kSeedGridRows - 1);
            occupied_seed_cells[row * kSeedGridColumns + column] = true;
        }
    }
    const int seed_grid = static_cast<int>(std::count(
        occupied_seed_cells.begin(), occupied_seed_cells.end(), true));
    AINFO << "ATLAS_RGBD_TEMPORAL_PNP_MP_SEED"
          << " identity_valid=1"
          << " pairs=" << accepted_result.pnp_inlier_pairs.size()
          << " seeded=" << seeded << " seed_grid=" << seed_grid
          << " out_of_bounds=" << out_of_bounds
          << " source_null=" << source_null
          << " replacement_hops=" << replacement_hops
          << " invalid_replacement_chain=" << invalid_replacement_chain
          << " bad=" << bad << " wrong_map=" << wrong_map
          << " zero_obs=" << zero_observations
          << " duplicate_map_point=" << duplicate_map_point
          << " duplicate_current_index=" << duplicate_current_index
          << " occupied_current_slot=" << occupied_current_slot
          << " frame_id=" << mCurrentFrame.mnId << std::endl;

    int committed_inliers = 0;
    const bool committed =
        TrackLocalMap(true, internal::LocalMapCommitMode::kTransactional,
                      &committed_inliers, required_inliers, transaction);
    if (validation_inliers) {
        *validation_inliers = committed_inliers;
    }
    if (!committed || !internal::HasEnoughRgbdTemporalReanchorInliers(
                          imu_initialized, committed_inliers)) {
        transaction->Restore();
        restore_frame_and_tracker();
        return false;
    }
    transaction->Commit();
    return true;
}

void Tracker::UpdateRgbdTemporalReference(Map* current_map) {
    if (mSensor != System::IMU_RGBD || !current_map || !mCurrentFrame.isSet()) {
        return;
    }
    rgbd_temporal_pose_estimator_.UpdateReference(
        CurrentRgbdTemporalFrameView(current_map), mCurrentFrame.GetPose());
}

void Tracker::Track() {
    if (bStepByStep) {
        AINFO << "Tracker: Waiting to the next step" << std::endl;
        while (!mbStep && bStepByStep)
            usleep(500);
        mbStep = false;
    }

    if (mpLocalMapper->mbBadImu) {
        AINFO << "TRACK: Reset map because local mapper set the bad imu flag "
              << std::endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    if (!pCurrentMap) {
        AINFO << "ERROR: There is not an active map in the atlas" << std::endl;
    }

    if (mState != NO_IMAGES_YET) {
        if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
            AERROR << "ERROR: Frame with a timestamp older than previous frame "
                      "detected!"
                   << std::endl;
            std::unique_lock<std::mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        } else if (mpAtlas->isInertial()) {
            const auto jump_action = internal::ClassifyInertialTimestampJump(
                mCurrentFrame.mTimeStamp, mLastFrame.mTimeStamp);
            if (jump_action ==
                internal::InertialTimestampJumpAction::kSoftRecover) {
                // Loop correction blocks Track* long enough that the next
                // synced frame often jumps >1s. Soft-recover keeps the map.
                SoftRecoverAfterTimestampJump();
            } else if (jump_action ==
                       internal::InertialTimestampJumpAction::kHardReset) {
                if (mpAtlas->isImuInitialized()) {
                    AINFO << "Timestamp jump detected. State set to LOST. "
                             "Reseting IMU integration..."
                          << std::endl;
                    if (!pCurrentMap->GetIniertialBA2()) {
                        mpSystem->ResetActiveMap();
                    } else {
                        CreateMapInAtlas();
                    }
                } else {
                    AINFO << "Timestamp jump detected, before IMU "
                             "initialization. Reseting..."
                          << std::endl;
                    mpSystem->ResetActiveMap();
                }
                return;
            }
        }
    }

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD) &&
        mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if (mState == NO_IMAGES_YET) {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;
    bool mark_current_frame_lost = false;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD) &&
        !mbCreatedMap) {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU =
            std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU =
            std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                time_EndPreIMU - time_StartPreIMU)
                                .count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif
    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    std::unique_lock<std::mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if (nCurMapChangeIndex > nMapChangeIndex) {
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
        RebaseRgbdTemporalReferenceAfterMapUpdate(pCurrentMap, nMapChangeIndex,
                                                  nCurMapChangeIndex);
    }

    if (mState == NOT_INITIALIZED) {
        if (mSensor == System::STEREO || mSensor == System::RGBD ||
            mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            StereoInitialization();
        } else {
            MonocularInitialization();
        }

        if (mState != OK)  // If rightly initialized, mState=OK
        {
            mpFrameDrawer->Update(this);
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if (mpAtlas->GetAllMaps().size() == 1) {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    } else {
        // System is initialized. Track Frame.
        bool bOK = false;
        const bool use_imu_rgbd_visual_recovery =
            internal::ShouldUseImuRgbdVisualRecovery(
                mSensor == System::IMU_RGBD, !mbOnlyTracking,
                imu_rgbd_visual_recovery_active_, mState == RECENTLY_LOST,
                pCurrentMap->isImuInitialized(),
                pCurrentMap->GetIniertialBA2());
        if (mState == OK && !use_imu_rgbd_visual_recovery) {
            imu_rgbd_visual_recovery_active_ = false;
        }
        bool is_imu_rgbd_visual_recovery_frame = false;
        bool imu_rgbd_visual_recovery_timed_out = false;
        bool failed_imu_rgbd_visual_recovery = false;
        bool attempted_local_map = false;
        bool used_imu_pose_seed = false;
        bool used_rgbd_temporal_pose_seed = false;
        bool temporal_pose_is_provisional = false;
        int temporal_validation_inliers = 0;
        RgbdTemporalPoseEstimator::Quality temporal_pose_quality =
            RgbdTemporalPoseEstimator::Quality::kRejected;
        RgbdTemporalPoseEstimator::Result temporal_pose_result;
        internal::LocalMapTrackingTransaction temporal_transaction;
        autonomy::common::math::SE3f provisional_temporal_pose;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred =
            std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization
        // (if tracking is lost)
        if (!mbOnlyTracking) {
            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if (mState == OK) {
                // Local Mapping might have changed some MapPoints tracked in
                // last frame
                CheckReplacedInLastFrame();

                if ((!mbVelocity && !pCurrentMap->isImuInitialized()) ||
                    mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                    AINFO << "TRACK: Track with respect to the reference KF ";
                    bOK = TrackReferenceKeyFrame();
                } else {
                    ADEBUG << "TRACK: Track with motion model";
                    bOK = TrackWithMotionModel();
                    if (!bOK) {
                        bOK = TrackReferenceKeyFrame();
                    }
                }

                if (!bOK) {
                    if (use_imu_rgbd_visual_recovery) {
                        imu_rgbd_visual_recovery_active_ = true;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                        is_imu_rgbd_visual_recovery_frame = true;
                    } else if (mCurrentFrame.mnId <=
                                   (mnLastRelocFrameId + mnFramesToResetIMU) &&
                               (mSensor == System::IMU_MONOCULAR ||
                                mSensor == System::IMU_STEREO ||
                                mSensor == System::IMU_RGBD)) {
                        mState = LOST;
                    } else if (pCurrentMap->KeyFramesInMap() > 10) {
                        // AINFO << "KF in map: " <<
                        // pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                    } else {
                        mState = LOST;
                    }
                }
            } else {
                if (mState == RECENTLY_LOST) {
                    ADEBUG << "Lost for a short time";

                    bOK = true;
                    if ((mSensor == System::IMU_MONOCULAR ||
                         mSensor == System::IMU_STEREO ||
                         mSensor == System::IMU_RGBD)) {
                        if (use_imu_rgbd_visual_recovery) {
                            imu_rgbd_visual_recovery_active_ = true;
                            is_imu_rgbd_visual_recovery_frame = true;
                            bOK =
                                TrackImuRgbdVisualRecoveryReferenceKeyFrames();
                            if (internal::
                                    ShouldUseImuPoseSeedAfterReferenceFailure(
                                        mSensor == System::IMU_RGBD,
                                        imu_rgbd_visual_recovery_active_, bOK,
                                        pCurrentMap->isImuInitialized())) {
                                // Recovery success still requires visual
                                // validation by TrackLocalMap below.
                                used_imu_pose_seed = PredictStateIMU();
                                bOK = used_imu_pose_seed;
                            }
                        } else if (pCurrentMap->isImuInitialized()) {
                            const bool imu_prediction_succeeded =
                                PredictStateIMU();
                            if (internal::ShouldUseImuPredictionResult(
                                    mSensor == System::IMU_RGBD)) {
                                bOK = imu_prediction_succeeded;
                            }
                        } else {
                            bOK = false;
                        }

                        imu_rgbd_visual_recovery_timed_out =
                            internal::HasRecentlyLostTimedOut(
                                mCurrentFrame.mTimeStamp - mTimeStampLost,
                                mSensor == System::IMU_RGBD,
                                imu_rgbd_visual_recovery_active_,
                                time_recently_lost);
                    } else {
                        // Relocalization
                        bOK = Relocalization();
                        // AINFO << "mCurrentFrame.mTimeStamp:" <<
                        // to_string(mCurrentFrame.mTimeStamp) << std::endl;
                        // AINFO << "mTimeStampLost:" <<
                        // to_string(mTimeStampLost) << std::endl;
                        if (mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f &&
                            !bOK) {
                            mState = LOST;
                            LogVisualSlamEvent("lost");
                            bOK = false;
                        }
                    }
                } else if (mState == LOST) {
                    AINFO << "A new map is started...";
                    imu_rgbd_visual_recovery_active_ = false;

                    if (pCurrentMap->KeyFramesInMap() < 10) {
                        mpSystem->ResetActiveMap();
                        AINFO << "Reseting current map...";
                    } else
                        CreateMapInAtlas();

                    if (mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(nullptr);

                    AINFO << "done";

                    return;
                }
            }

        } else {
            // Localization Mode: Local Mapping is deactivated (TODO Not
            // available in inertial mode)
            if (mState == LOST) {
                if (mSensor == System::IMU_MONOCULAR ||
                    mSensor == System::IMU_STEREO ||
                    mSensor == System::IMU_RGBD)
                    AINFO << "IMU. State LOST";
                bOK = Relocalization();
            } else {
                if (!mbVO) {
                    // In last frame we tracked enough MapPoints in the map
                    if (mbVelocity) {
                        bOK = TrackWithMotionModel();
                    } else {
                        bOK = TrackReferenceKeyFrame();
                    }
                } else {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and
                    // one doing relocalization. If relocalization is sucessfull
                    // we choose that solution, otherwise we retain the "visual
                    // odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    std::vector<MapPoint*> vpMPsMM;
                    std::vector<bool> vbOutMM;
                    autonomy::common::math::SE3f TcwMM;
                    if (mbVelocity) {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.GetPose();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc) {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if (mbVO) {
                            for (int i = 0; i < mCurrentFrame.N; i++) {
                                if (mCurrentFrame.mvpMapPoints[i] &&
                                    !mCurrentFrame.mvbOutlier[i]) {
                                    mCurrentFrame.mvpMapPoints[i]
                                        ->IncreaseFound();
                                }
                            }
                        }
                    } else if (bOKReloc) {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if (!mbOnlyTracking && !bOK &&
            TryRgbdTemporalPose(pCurrentMap, &temporal_pose_quality,
                                &temporal_pose_result,
                                &temporal_transaction)) {
            bOK = true;
            used_rgbd_temporal_pose_seed = true;
            provisional_temporal_pose = mCurrentFrame.GetPose();
            temporal_pose_is_provisional = true;
            if (mSensor == System::IMU_RGBD) {
                imu_rgbd_visual_recovery_active_ = true;
                is_imu_rgbd_visual_recovery_frame = true;
            }
        }

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred =
            std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<
                                  std::chrono::duration<double, std::milli>>(
                                  time_EndPosePred - time_StartPosePred)
                                  .count();
        vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack =
            std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching.
        // Track the local map.
        if (!mbOnlyTracking) {
            if (bOK) {
                attempted_local_map = true;
                const bool force_visual_pose_optimization =
                    used_rgbd_temporal_pose_seed ||
                    internal::ShouldForceVisualRecoveryPoseOptimization(
                        is_imu_rgbd_visual_recovery_frame,
                        pCurrentMap->isImuInitialized());
                bOK = used_rgbd_temporal_pose_seed
                          ? ValidateAndCommitRgbdTemporalPose(
                                pCurrentMap, temporal_pose_result,
                                &temporal_transaction,
                                &temporal_validation_inliers)
                          : TrackLocalMap(force_visual_pose_optimization);
                temporal_pose_is_provisional &= !bOK;
            }
            if (!bOK && !used_rgbd_temporal_pose_seed &&
                TryRgbdTemporalPose(pCurrentMap, &temporal_pose_quality,
                                    &temporal_pose_result,
                                    &temporal_transaction)) {
                attempted_local_map = true;
                used_rgbd_temporal_pose_seed = true;
                provisional_temporal_pose = mCurrentFrame.GetPose();
                temporal_pose_is_provisional = true;
                if (mSensor == System::IMU_RGBD) {
                    imu_rgbd_visual_recovery_active_ = true;
                    is_imu_rgbd_visual_recovery_frame = true;
                }
                bOK = ValidateAndCommitRgbdTemporalPose(
                    pCurrentMap, temporal_pose_result, &temporal_transaction,
                    &temporal_validation_inliers);
                temporal_pose_is_provisional &= !bOK;
            }
            if (!bOK && used_rgbd_temporal_pose_seed) {
                AINFO << "ATLAS_RGBD_TEMPORAL_PNP_TLM_REJECT"
                      << " final_local_inliers=" << temporal_validation_inliers
                      << " frame_id=" << mCurrentFrame.mnId << std::endl;
                // Do not let a rejected seed become the next visual reference
                // or leak partially validated map associations.
                mCurrentFrame.mvpMapPoints.assign(mCurrentFrame.N, nullptr);
                mCurrentFrame.mvbOutlier.assign(mCurrentFrame.N, false);
                mCurrentFrame.mmProjectPoints.clear();
                mCurrentFrame.mmMatchedInImage.clear();
                mCurrentFrame.SetPose(provisional_temporal_pose);
                temporal_pose_is_provisional =
                    AdvanceRgbdTemporalOdometryReference(pCurrentMap,
                                                         temporal_pose_quality);
                if (!temporal_pose_is_provisional && mLastFrame.isSet()) {
                    mCurrentFrame.SetPose(mLastFrame.GetPose());
                }
            }
            if (!bOK) {
                if (internal::CanHoldLastImuRgbdPose(
                        mSensor == System::IMU_RGBD, mLastFrame.isSet()) &&
                    HoldLastImuRgbdPose()) {
                    bOK = true;
                    imu_rgbd_visual_recovery_active_ = false;
                    used_rgbd_temporal_pose_seed = false;
                    temporal_pose_is_provisional = false;
                    ++consecutive_pose_holds_;
                    if (consecutive_pose_holds_ == 1 ||
                        consecutive_pose_holds_ % 30 == 0) {
                        AWARN << "ATLAS_TRACK event=hold_pose n="
                              << consecutive_pose_holds_
                              << " inliers=" << mnMatchesInliers
                              << " frame=" << mCurrentFrame.mnId;
                    }
                } else {
                    ++consecutive_local_map_fails_;
                    if (consecutive_local_map_fails_ == 1 ||
                        consecutive_local_map_fails_ % 30 == 0) {
                        AWARN << "ATLAS_TRACK event=local_map_fail n="
                              << consecutive_local_map_fails_
                              << " inliers=" << mnMatchesInliers
                              << " frame=" << mCurrentFrame.mnId;
                    }
                }
            } else if (consecutive_local_map_fails_ > 0 ||
                       consecutive_pose_holds_ > 0) {
                if (consecutive_local_map_fails_ > 0) {
                    AINFO << "ATLAS_TRACK event=local_map_recovered after_fails="
                          << consecutive_local_map_fails_
                          << " inliers=" << mnMatchesInliers
                          << " frame=" << mCurrentFrame.mnId;
                }
                consecutive_local_map_fails_ = 0;
                consecutive_pose_holds_ = 0;
            }
        } else {
            // mbVO true means that there are few matches to MapPoints in the
            // map. We cannot retrieve a local map and therefore we do not
            // perform TrackLocalMap(). Once the system relocalizes the camera
            // we will use the local map again.
            if (bOK && !mbVO) {
                attempted_local_map = true;
                bOK = TrackLocalMap();
            }
        }

        if (internal::ShouldDeclareImuRgbdVisualRecoveryLost(
                imu_rgbd_visual_recovery_timed_out, bOK) &&
            !(internal::CanHoldLastImuRgbdPose(mSensor == System::IMU_RGBD,
                                               mLastFrame.isSet()) &&
              HoldLastImuRgbdPose())) {
            imu_rgbd_visual_recovery_active_ = false;
            rgbd_temporal_odometry_state_.Reset();
            mState = LOST;
            LogVisualSlamEvent("lost");
        } else if (internal::ShouldDeclareImuRgbdVisualRecoveryLost(
                       imu_rgbd_visual_recovery_timed_out, bOK)) {
            bOK = true;
            imu_rgbd_visual_recovery_active_ = false;
            rgbd_temporal_odometry_state_.Reset();
            mState = OK;
            LogVisualSlamEvent("hold_pose_timeout");
        }

        if (bOK) {
            imu_rgbd_visual_recovery_active_ = false;
            rgbd_temporal_odometry_state_.Reset();
            mState = OK;
        } else if (mState == OK) {
            if (mSensor == System::IMU_MONOCULAR ||
                mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
                AINFO << "Track lost for less than one second...";
                if (use_imu_rgbd_visual_recovery) {
                    imu_rgbd_visual_recovery_active_ = true;
                    is_imu_rgbd_visual_recovery_frame = true;
                    AINFO << "IMU RGB-D visual recovery before BA2. Keeping "
                             "the active map."
                          << std::endl;
                } else if (!pCurrentMap->isImuInitialized() ||
                           !pCurrentMap->GetIniertialBA2()) {
                    AINFO << "IMU is not or recently initialized. Reseting "
                             "active map..."
                          << std::endl;
                    mpSystem->ResetActiveMap();
                }

                mState = RECENTLY_LOST;
            } else {
                mState = RECENTLY_LOST;  // visual to lost
            }

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
            mTimeStampLost = mCurrentFrame.mTimeStamp;
            //}
        }

        failed_imu_rgbd_visual_recovery =
            is_imu_rgbd_visual_recovery_frame && !bOK;
        if (failed_imu_rgbd_visual_recovery) {
            mark_current_frame_lost = true;
            mbVelocity = false;
            if (!mCurrentFrame.isSet() && mLastFrame.isSet()) {
                mCurrentFrame.SetPose(mLastFrame.GetPose());
            }
        }

        // Save frame if recent relocalization, since they are used for IMU
        // reset (as we are making copy, it shluld be once mCurrFrame is
        // completely modified)
        if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&
            (mCurrentFrame.mnId > mnFramesToResetIMU) &&
            (mSensor == System::IMU_MONOCULAR ||
             mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
            pCurrentMap->isImuInitialized()) {
            // TODO check this situation
            AINFO << "Saving pointer to frame. imu needs reset...";
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            if (mCurrentFrame.mpImuPreintegratedFrame) {
                pF->mpImuPreintegratedFrame = new IMU::Preintegrated(
                    mCurrentFrame.mpImuPreintegratedFrame);
            }
        }

        if (pCurrentMap->isImuInitialized()) {
            if (bOK) {
                if (mCurrentFrame.mnId ==
                    (mnLastRelocFrameId + mnFramesToResetIMU)) {
                    AINFO << "RESETING FRAME!!!" << std::endl;
                    ResetFrameIMU();
                } else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack =
            std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<
                                 std::chrono::duration<double, std::milli>>(
                                 time_EndLMTrack - time_StartLMTrack)
                                 .count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        mpFrameDrawer->Update(this);
        if (mCurrentFrame.isSet())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        if (bOK || mState == RECENTLY_LOST) {
            // Update motion model
            if (is_imu_rgbd_visual_recovery_frame) {
                mbVelocity = false;
            } else if (mLastFrame.isSet() && mCurrentFrame.isSet()) {
                autonomy::common::math::SE3f LastTwc =
                    mLastFrame.GetPose().inverse();
                mVelocity = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity = true;
            } else {
                mbVelocity = false;
            }

            if (mSensor == System::IMU_MONOCULAR ||
                mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // Clean VO matches
            for (int i = 0; i < mCurrentFrame.N; i++) {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                    if (pMP->Observations() < 1) {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i] =
                            static_cast<MapPoint*>(nullptr);
                    }
            }

            // Delete temporal MapPoints
            for (std::list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(),
                                                lend = mlpTemporalPoints.end();
                 lit != lend; lit++) {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF =
                std::chrono::steady_clock::now();
#endif
            const bool evaluate_failed_recovery_key_frame =
                internal::ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
                    mSensor == System::IMU_RGBD,
                    imu_rgbd_visual_recovery_active_,
                    failed_imu_rgbd_visual_recovery,
                    pCurrentMap->isImuInitialized(), attempted_local_map,
                    mCurrentFrame.isSet(), mInsertKFsLost, used_imu_pose_seed);
            bool bNeedKF = !temporal_pose_is_provisional &&
                           (!failed_imu_rgbd_visual_recovery ||
                            evaluate_failed_recovery_key_frame) &&
                           NeedNewKeyFrame();

            // Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)
            if (bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST &&
                                    (mSensor == System::IMU_MONOCULAR ||
                                     mSensor == System::IMU_STEREO ||
                                     mSensor == System::IMU_RGBD))))
                CreateNewKeyFrame();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF =
                std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<
                                   std::chrono::duration<double, std::milli>>(
                                   time_EndNewKF - time_StartNewKF)
                                   .count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the
            // Huber Function) pass to the new keyframe, so that bundle
            // adjustment will finally decide if they are outliers or not. We
            // don't want next frame to estimate its position with those points
            // so we discard them in the frame. Only has effect if lastframe is
            // tracked
            for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] &&
                    mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i] =
                        static_cast<MapPoint*>(nullptr);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST) {
            imu_rgbd_visual_recovery_active_ = false;
            if (internal::ShouldKeepActiveMapAfterLost(
                    mSensor == System::IMU_RGBD,
                    pCurrentMap->isImuInitialized(), mLastFrame.isSet()) &&
                HoldLastImuRgbdPose()) {
                mState = OK;
                mark_current_frame_lost = false;
                AWARN << "ATLAS_TRACK event=keep_map after_lost"
                      << " kfs=" << pCurrentMap->KeyFramesInMap()
                      << " frame=" << mCurrentFrame.mnId;
            } else if (pCurrentMap->KeyFramesInMap() <= 10) {
                mpSystem->ResetActiveMap();
                return;
            } else if ((mSensor == System::IMU_MONOCULAR ||
                        mSensor == System::IMU_STEREO ||
                        mSensor == System::IMU_RGBD) &&
                       !pCurrentMap->isImuInitialized()) {
                AINFO << "Track lost before IMU initialisation, reseting...";
                mpSystem->ResetActiveMap();
                return;
            } else {
                CreateMapInAtlas();
                return;
            }
        }

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    if (mState == OK && !mark_current_frame_lost &&
        pCurrentMap == mpAtlas->GetCurrentMap()) {
        UpdateRgbdTemporalReference(pCurrentMap);
    }

    if (mState == OK || mState == RECENTLY_LOST) {
        const internal::TrajectoryRecordDecision record_decision =
            internal::MakeTrajectoryRecordDecision(mCurrentFrame.mTimeStamp,
                                                   mCurrentFrame.isSet(),
                                                   mark_current_frame_lost);
        // Store frame pose information to retrieve the complete camera
        // trajectory afterwards.
        if (!record_decision.reuse_previous_pose) {
            autonomy::common::math::SE3f Tcr_ =
                mCurrentFrame.GetPose() *
                mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
        } else {
            // This can happen if tracking is lost. The previous pose is kept
            // only to preserve parallel-list alignment; record_decision.lost
            // prevents it from being exported as a valid repeated estimate.
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
        }
        // Every accepted input frame owns its input timestamp, even when its
        // pose storage falls back to the previous relative pose.
        mlFrameTimes.push_back(record_decision.timestamp);
        mlbLost.push_back(record_decision.lost);
    }

#ifdef REGISTER_LOOP
    if (Stop()) {
        // Safe area to stop
        while (isStopped()) {
            usleep(3000);
        }
    }
#endif

    const bool inliers_drop =
        last_logged_inliers_ >= 40 && mnMatchesInliers < 20;
    const bool inliers_recover =
        last_logged_inliers_ >= 0 && last_logged_inliers_ < 20 &&
        mnMatchesInliers >= 40;
    if (mState != last_logged_state_) {
        LogVisualSlamEvent("state");
        last_logged_state_ = mState;
        last_logged_inliers_ = mnMatchesInliers;
    } else if (inliers_drop) {
        LogVisualSlamEvent("inliers_drop");
        last_logged_inliers_ = mnMatchesInliers;
    } else if (inliers_recover) {
        LogVisualSlamEvent("inliers_recover");
        last_logged_inliers_ = mnMatchesInliers;
    }
}

void Tracker::StereoInitialization() {
    if (mCurrentFrame.N > 500) {
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            if (!mCurrentFrame.mpImuPreintegrated ||
                !mLastFrame.mpImuPreintegrated) {
                AINFO << "not IMU meas" << std::endl;
                return;
            }

            // IMU-RGBD can seed a metric map from depth while the platform is
            // parked (AutoSim at spawn). The Δacc gate is for IMU-stereo /
            // fastInit-off cases where IMU excitation is the only scale cue.
            if (mSensor != System::IMU_RGBD && !mFastInit &&
                (mCurrentFrame.mpImuPreintegratedFrame->avgA -
                 mLastFrame.mpImuPreintegratedFrame->avgA)
                        .norm() < 0.5) {
                AINFO << "not enough acceleration" << std::endl;
                return;
            }

            if (mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF =
                new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Metric RGB-D: world = IMU (REP-103 Z-up). Camera-at-identity makes
        // map XY the optical image plane, so Autoviz RobotModel/TF sit on
        // their side relative to the grid and URDF. Stereo-inertial keeps the
        // historical camera-origin seed used by EuRoC / TUM-VI eval.
        if (mSensor == System::IMU_RGBD) {
            mCurrentFrame.SetImuPoseVelocity(Eigen::Matrix3f::Identity(),
                                             Eigen::Vector3f::Zero(),
                                             Eigen::Vector3f::Zero());
        } else if (mSensor == System::IMU_STEREO) {
            Eigen::Matrix3f Rwb0 =
                mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0,
                                             Eigen::Vector3f::Zero());
        } else
            mCurrentFrame.SetPose(autonomy::common::math::SE3f());

        // Create KeyFrame
        KeyFrame* pKFini =
            new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if (!mpCamera2) {
            for (int i = 0; i < mCurrentFrame.N; i++) {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0) {
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    MapPoint* pNewMP =
                        new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }
        } else {
            for (int i = 0; i < mCurrentFrame.Nleft; i++) {
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if (rightIndex != -1) {
                    Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP =
                        new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini, i);
                    pNewMP->AddObservation(pKFini,
                                           rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP, i);
                    pKFini->AddMapPoint(pNewMP,
                                        rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    mCurrentFrame
                        .mvpMapPoints[rightIndex + mCurrentFrame.Nleft] =
                        pNewMP;
                }
            }
        }

        AINFO << "New Map created with " +
                     std::to_string(mpAtlas->MapPointsInMap()) + " points";

        // AINFO << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);
        if (mSensor == System::IMU_RGBD)
            visual_initial_mapping_barrier_.MarkPending();

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        // mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mState = OK;
    }
}

void Tracker::MonocularInitialization() {
    if (!mbReadyToInitializate) {
        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > 100) {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (std::size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            std::fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            if (mSensor == System::IMU_MONOCULAR) {
                if (mpImuPreintegratedFromLastKF) {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF =
                    new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            mbReadyToInitializate = true;

            return;
        }
    } else {
        if ((static_cast<int>(mCurrentFrame.mvKeys.size()) <= 100) ||
            ((mSensor == System::IMU_MONOCULAR) &&
             (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0))) {
            mbReadyToInitializate = false;

            return;
        }

        // Find correspondences
        OrbMatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(
            mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

        // Check if there are enough correspondences
        if (nmatches < 100) {
            mbReadyToInitializate = false;
            return;
        }

        autonomy::common::math::SE3f Tcw;
        std::vector<bool>
            vbTriangulated;  // Triangulated Correspondences (mvIniMatches)

        if (mpCamera->ReconstructWithTwoViews(
                mInitialFrame.mvKeysUn, mCurrentFrame.mvKeysUn, mvIniMatches,
                Tcw, mvIniP3D, vbTriangulated)) {
            for (std::size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(autonomy::common::math::SE3f());
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracker::CreateInitialMapMonocular() {
    // Create KeyFrames
    KeyFrame* pKFini =
        new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
    KeyFrame* pKFcur =
        new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if (mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = nullptr;

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for (std::size_t i = 0; i < mvIniMatches.size(); i++) {
        if (mvIniMatches[i] < 0)
            continue;

        // Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        MapPoint* pMP =
            new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        // Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        // Add to Map
        mpAtlas->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    AINFO << "New Map created with " +
                 std::to_string(mpAtlas->MapPointsInMap()) + " points";
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if (mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;  // 4.0f
    else
        invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 ||
        pKFcur->TrackedMapPoints(1) < 50)  // TODO Check, originally 100 tracks
    {
        AINFO << "Wrong initialization, reseting...";
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    autonomy::common::math::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    std::vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (std::size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if (vpAllMapPoints[iMP]) {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR) {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(
            pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    if (mSensor == System::MONOCULAR)
        visual_initial_mapping_barrier_.MarkPending();
    mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    // mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    std::vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    autonomy::common::math::SE3f deltaT =
        vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) /
                 (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;

    initID = pKFcur->mnId;
}

void Tracker::CreateMapInAtlas() {
    rgbd_temporal_pose_estimator_.Reset();
    rgbd_temporal_odometry_state_.Reset();
    imu_rgbd_visual_recovery_active_ = false;
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
        mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mbSetInit = false;

    mnInitialFrameId = mCurrentFrame.mnId + 1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is
    // the current id, because it is the new starting point for new map
    AINFO << "First frame id in map: " + std::to_string(mnLastInitFrameId + 1);
    mbVO = false;  // Init value for know if there are enough MapPoints in the
                   // last KF
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR) {
        mbReadyToInitializate = false;
    }

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD) &&
        mpImuPreintegratedFromLastKF) {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    }

    if (mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(nullptr);

    if (mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(nullptr);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;
}

void Tracker::CheckReplacedInLastFrame() {
    for (int i = 0; i < mLastFrame.N; i++) {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if (pMP) {
            MapPoint* pRep = pMP->GetReplaced();
            if (pRep) {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool Tracker::HoldLastImuRgbdPose() {
    if (!internal::CanHoldLastImuRgbdPose(mSensor == System::IMU_RGBD,
                                          mLastFrame.isSet())) {
        return false;
    }
    mCurrentFrame.SetImuPoseVelocity(mLastFrame.GetImuRotation(),
                                     mLastFrame.GetImuPosition(),
                                     mLastFrame.GetVelocity());
    mCurrentFrame.mImuBias = mLastFrame.mImuBias;
    mCurrentFrame.mPredBias = mLastFrame.mPredBias;
    mnMatchesInliers = 0;
    mbVelocity = false;
    if (mpReferenceKF) {
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    } else if (mLastFrame.mpReferenceKF) {
        mCurrentFrame.mpReferenceKF = mLastFrame.mpReferenceKF;
        mpReferenceKF = mLastFrame.mpReferenceKF;
    }
    return mCurrentFrame.isSet();
}

bool Tracker::TrackReferenceKeyFrame() {
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    OrbMatcher matcher(0.7, true);
    std::vector<MapPoint*> vpMapPointMatches;

    int nmatches =
        matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    if (nmatches < 15) {
        AINFO << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());

    // mCurrentFrame.PrintPointDistribution();

    // AINFO << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw
    // << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        // if(i >= mCurrentFrame.Nleft) break;
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(nullptr);
                mCurrentFrame.mvbOutlier[i] = false;
                if (i < mCurrentFrame.Nleft) {
                    pMP->mbTrackInView = false;
                } else {
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

bool Tracker::TrackImuRgbdVisualRecoveryReferenceKeyFrames() {
    KeyFrame* const original_reference_keyframe = mpReferenceKF;
    KeyFrame* const original_frame_reference_keyframe =
        mCurrentFrame.mpReferenceKF;

    std::vector<KeyFrame*> candidates;
    candidates.reserve(internal::kImuRgbdRecoveryReferenceCandidateLimit);
    const auto append_candidate = [&candidates](KeyFrame* candidate) {
        const bool candidate_valid = candidate && !candidate->isBad();
        bool already_present = false;
        for (KeyFrame* existing : candidates) {
            if (existing == candidate) {
                already_present = true;
                break;
            }
        }
        if (internal::ShouldAppendImuRgbdRecoveryReferenceCandidate(
                candidate_valid, already_present,
                static_cast<int>(candidates.size()))) {
            candidates.push_back(candidate);
        }
    };

    append_candidate(mpReferenceKF);
    append_candidate(mpLastKeyFrame);
    if (mpLastKeyFrame && !mpLastKeyFrame->isBad()) {
        const std::vector<KeyFrame*> covisible_keyframes =
            mpLastKeyFrame->GetBestCovisibilityKeyFrames(
                internal::kImuRgbdRecoveryCovisibilityCandidateLimit);
        for (KeyFrame* candidate : covisible_keyframes) {
            append_candidate(candidate);
        }
    }

    for (KeyFrame* candidate : candidates) {
        mpReferenceKF = candidate;
        mCurrentFrame.mpReferenceKF = candidate;
        if (TrackReferenceKeyFrame()) {
            return true;
        }
    }

    mpReferenceKF = original_reference_keyframe;
    mCurrentFrame.mpReferenceKF = original_frame_reference_keyframe;
    return false;
}

void Tracker::UpdateLastFrame() {
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    autonomy::common::math::SE3f Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
        mSensor == System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D
    // sensor
    std::vector<std::pair<float, int>> vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for (int i = 0; i < Nfeat; i++) {
        float z = mLastFrame.mvDepth[i];
        if (z > 0) {
            vDepthIdx.push_back(std::make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    std::sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (std::size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1)
            bCreateNew = true;

        if (bCreateNew) {
            Eigen::Vector3f x3D;

            if (mLastFrame.Nleft == -1) {
                mLastFrame.UnprojectStereo(i, x3D);
            } else {
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            MapPoint* pNewMP =
                new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        } else {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

bool Tracker::TrackWithMotionModel() {
    OrbMatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    if (mpAtlas->isImuInitialized() &&
        (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
        // Predict state with IMU if it is initialized and it doesnt need reset
        const bool imu_prediction_succeeded = PredictStateIMU();
        if (mSensor == System::IMU_RGBD) {
            if (!imu_prediction_succeeded && mLastFrame.isSet()) {
                mCurrentFrame.SetPose(mLastFrame.GetPose());
            }
        } else {
            return internal::ShouldUseImuPredictionResult(false)
                       ? imu_prediction_succeeded
                       : true;
        }
    } else {
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }

    std::fill(mCurrentFrame.mvpMapPoints.begin(),
              mCurrentFrame.mvpMapPoints.end(),
              static_cast<MapPoint*>(nullptr));

    // Project points seen in previous frame
    int th;

    if (mSensor == System::STEREO)
        th = 7;
    else
        th = 15;

    int nmatches = matcher.SearchByProjection(
        mCurrentFrame, mLastFrame, th,
        mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

    // Twenty matches is the search-quality threshold: below it, retry the
    // projection with a wider window before deciding whether the pose can be
    // optimized.
    if (nmatches < 20) {
        AINFO << "Not enough matches, wider window search!!";
        std::fill(mCurrentFrame.mvpMapPoints.begin(),
                  mCurrentFrame.mvpMapPoints.end(),
                  static_cast<MapPoint*>(nullptr));

        nmatches = matcher.SearchByProjection(
            mCurrentFrame, mLastFrame, 2 * th,
            mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);
        AINFO << "Matches with wider search: " + std::to_string(nmatches);
    }

    // Fifteen matches is only the minimum pose seed. Pose optimization and the
    // existing map-inlier checks below still validate the tracking result.
    if (!internal::HasEnoughMotionModelPoseSeedMatches(nmatches)) {
        AINFO << "Not enough matches!!";
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(nullptr);
                mCurrentFrame.mvbOutlier[i] = false;
                if (i < mCurrentFrame.Nleft) {
                    pMP->mbTrackInView = false;
                } else {
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

bool Tracker::TrackLocalMap(
    bool force_visual_pose_optimization,
    internal::LocalMapCommitMode commit_mode, int* validation_inliers,
    int required_minimum_inliers,
    internal::LocalMapTrackingTransaction* transaction) {
    const bool transactional =
        commit_mode == internal::LocalMapCommitMode::kTransactional;
    if (transactional && !transaction) {
        return false;
    }
    const auto count_frame_associations = [&]() {
        return static_cast<int>(std::count_if(
            mCurrentFrame.mvpMapPoints.begin(),
            mCurrentFrame.mvpMapPoints.end(),
            [](const MapPoint* map_point) { return map_point != nullptr; }));
    };
    std::vector<MapPoint*> deferred_visible_points;
    // We have an estimation of the camera pose and some map points tracked in
    // the frame. We retrieve the local map and try to find matches to points in
    // the local map.
    if (!transactional) {
        ++mTrackedFr;
    }

    const int associations_before_local_map = count_frame_associations();
    UpdateLocalMap(!transactional, transaction);
    const std::size_t local_keyframe_count = mvpLocalKeyFrames.size();
    const std::size_t local_map_point_count = mvpLocalMapPoints.size();
    bool geometry_reanchor_activated = false;
    Map* geometry_reanchor_expected_map = nullptr;
    unsigned long geometry_reanchor_expected_map_id = 0;
    std::uint64_t geometry_reanchor_expected_coordinate_epoch = 0;
    int geometry_reanchor_expected_map_revision = 0;
    if (internal::ShouldUseStrictRgbdTemporalLocalMapProjection(
            transactional, mSensor == System::IMU_RGBD,
            imu_rgbd_visual_recovery_active_)) {
        const RgbdMapReanchorMatcher::FrameView frame_view = {
            &mCurrentFrame.mDescriptors,
            &mCurrentFrame.mvKeysUn,
            &mCurrentFrame.mvDepth,
            mCurrentFrame.mK_,
            mCurrentFrame.N,
            Frame::mnMinX,
            Frame::mnMaxX,
            Frame::mnMinY,
            Frame::mnMaxY,
        };
        Map* const expected_map = mpAtlas->GetCurrentMap();
        const unsigned long expected_map_id =
            expected_map ? expected_map->GetId() : 0;
        const std::uint64_t expected_coordinate_epoch =
            expected_map ? expected_map->GetCoordinateEpoch() : 0;
        const int expected_map_revision =
            expected_map ? expected_map->GetMapChangeIndex() : 0;
        std::unordered_set<MapPoint*> local_map_point_set;
        local_map_point_set.reserve(mvpLocalMapPoints.size());
        for (MapPoint* const map_point : mvpLocalMapPoints) {
            if (map_point) {
                local_map_point_set.insert(map_point);
            }
        }

        struct DescriptorSourceKeyFrame {
            double squared_camera_distance = 0.0;
            std::uint64_t keyframe_id = 0;
            KeyFrame* keyframe = nullptr;
        };
        std::vector<DescriptorSourceKeyFrame> descriptor_source_keyframes;
        descriptor_source_keyframes.reserve(mvpLocalKeyFrames.size());
        const Eigen::Vector3f current_camera_center =
            mCurrentFrame.GetPose().inverse().translation();
        if (expected_map && current_camera_center.allFinite()) {
            for (KeyFrame* const keyframe : mvpLocalKeyFrames) {
                if (!keyframe || keyframe->isBad() ||
                    keyframe->GetMap() != expected_map) {
                    continue;
                }
                const Eigen::Vector3f camera_center =
                    keyframe->GetCameraCenter();
                const double squared_camera_distance =
                    (camera_center - current_camera_center)
                        .cast<double>()
                        .squaredNorm();
                if (!camera_center.allFinite() ||
                    !std::isfinite(squared_camera_distance)) {
                    continue;
                }
                descriptor_source_keyframes.push_back(
                    {squared_camera_distance,
                     static_cast<std::uint64_t>(keyframe->mnId), keyframe});
            }
        }
        std::sort(descriptor_source_keyframes.begin(),
                  descriptor_source_keyframes.end(),
                  [](const DescriptorSourceKeyFrame& left,
                     const DescriptorSourceKeyFrame& right) {
                      return std::tie(left.squared_camera_distance,
                                      left.keyframe_id) <
                             std::tie(right.squared_camera_distance,
                                      right.keyframe_id);
                  });
        if (descriptor_source_keyframes.size() >
            internal::kRgbdMapReanchorDescriptorKeyFrameLimit) {
            descriptor_source_keyframes.resize(
                internal::kRgbdMapReanchorDescriptorKeyFrameLimit);
        }

        std::unordered_map<MapPoint*, cv::Mat>
            observation_descriptor_banks;
        observation_descriptor_banks.reserve(local_map_point_set.size());
        for (const DescriptorSourceKeyFrame& source :
             descriptor_source_keyframes) {
            const std::vector<MapPoint*> matches =
                source.keyframe->GetMapPointMatches();
            const int descriptor_count = std::min(
                static_cast<int>(matches.size()),
                source.keyframe->mDescriptors.rows);
            for (int feature_index = 0; feature_index < descriptor_count;
                 ++feature_index) {
                MapPoint* const map_point = matches[feature_index];
                if (!map_point ||
                    local_map_point_set.count(map_point) == 0U) {
                    continue;
                }
                cv::Mat& descriptor_bank =
                    observation_descriptor_banks[map_point];
                const cv::Mat descriptor =
                    source.keyframe->mDescriptors.row(feature_index);
                internal::AppendUniqueRgbdMapReanchorDescriptor(
                    descriptor, &descriptor_bank);
            }
        }

        std::vector<RgbdMapReanchorMatcher::MapPointView> map_point_views;
        map_point_views.reserve(mvpLocalMapPoints.size());
        for (std::size_t map_point_index = 0;
             map_point_index < mvpLocalMapPoints.size(); ++map_point_index) {
            MapPoint* const map_point = mvpLocalMapPoints[map_point_index];
            RgbdMapReanchorMatcher::MapPointView map_point_view;
            map_point_view.map_point = map_point;
            map_point_view.source_index = map_point_index;
            if (map_point) {
                map_point_view.map_point_id = map_point->mnId;
                map_point_view.valid_for_expected_map =
                    !map_point->isBad() && !map_point->GetReplaced() &&
                    map_point->GetMap() == expected_map &&
                    map_point->Observations() > 0;
                if (map_point_view.valid_for_expected_map) {
                    map_point_view.world_point = map_point->GetWorldPos();
                    const auto descriptors =
                        observation_descriptor_banks.find(map_point);
                    if (descriptors != observation_descriptor_banks.end() &&
                        !descriptors->second.empty()) {
                        map_point_view.descriptor_bank = descriptors->second;
                        map_point_view.observation_descriptor_rows =
                            descriptors->second.rows;
                    } else {
                        map_point_view.descriptor_bank =
                            map_point->GetDescriptor();
                    }
                }
            }
            map_point_views.push_back(map_point_view);
        }
        const RgbdMapReanchorMatcher matcher;
        const RgbdMapReanchorMatcher::Result shadow_result = matcher.Match(
            frame_view, map_point_views, mCurrentFrame.GetPose());
        const RgbdMapReanchorMatcher::Diagnostics& provisional =
            shadow_result.diagnostics;
        const RgbdMapReanchorMatcher::Diagnostics& refined =
            shadow_result.refined_diagnostics;
        const auto& provisional_thresholds =
            provisional.descriptor_threshold_support;
        const auto& refined_thresholds =
            refined.descriptor_threshold_support;
        const auto& provisional_geometry =
            provisional.geometry_threshold_support;
        const auto& refined_geometry =
            refined.geometry_threshold_support;
        AINFO << "ATLAS_RGBD_MAP_REANCHOR_SHADOW"
              << " input_valid=" << (provisional.input_valid ? 1 : 0)
              << " valid_unique_map_points="
              << provisional.valid_unique_map_points
              << " descriptor_source_keyframes="
              << descriptor_source_keyframes.size()
              << " descriptor_rows=" << provisional.descriptor_rows
              << " multi_descriptor_map_points="
              << provisional.multi_descriptor_map_points
              << " projected_map_points="
              << provisional.projected_map_points
              << " candidates_after_geometry="
              << provisional.candidates_after_geometry
              << " candidates_after_descriptor="
              << provisional.candidates_after_descriptor
              << " mutual_matches=" << provisional.mutual_matches
              << " assigned_matches=" << provisional.assigned_matches
              << " observation_descriptor_matches="
              << provisional.observation_descriptor_matches
              << " provisional_geometry_mutual_5cm="
              << provisional_geometry[0].mutual_matches
              << " provisional_geometry_assigned_5cm="
              << provisional_geometry[0].assigned_matches
              << " provisional_geometry_grid_5cm="
              << provisional_geometry[0].occupied_grid_cells
              << " provisional_geometry_mutual_10cm="
              << provisional_geometry[1].mutual_matches
              << " provisional_geometry_assigned_10cm="
              << provisional_geometry[1].assigned_matches
              << " provisional_geometry_grid_10cm="
              << provisional_geometry[1].occupied_grid_cells
              << " provisional_geometry_mutual_dynamic="
              << provisional_geometry[2].mutual_matches
              << " provisional_geometry_assigned_dynamic="
              << provisional_geometry[2].assigned_matches
              << " provisional_geometry_grid_dynamic="
              << provisional_geometry[2].occupied_grid_cells
              << " provisional_mutual_d50="
              << provisional_thresholds[0].mutual_matches
              << " provisional_assigned_d50="
              << provisional_thresholds[0].assigned_matches
              << " provisional_grid_d50="
              << provisional_thresholds[0].occupied_grid_cells
              << " provisional_mutual_d60="
              << provisional_thresholds[1].mutual_matches
              << " provisional_assigned_d60="
              << provisional_thresholds[1].assigned_matches
              << " provisional_grid_d60="
              << provisional_thresholds[1].occupied_grid_cells
              << " provisional_mutual_d70="
              << provisional_thresholds[2].mutual_matches
              << " provisional_assigned_d70="
              << provisional_thresholds[2].assigned_matches
              << " provisional_grid_d70="
              << provisional_thresholds[2].occupied_grid_cells
              << " provisional_mutual_d80="
              << provisional_thresholds[3].mutual_matches
              << " provisional_assigned_d80="
              << provisional_thresholds[3].assigned_matches
              << " provisional_grid_d80="
              << provisional_thresholds[3].occupied_grid_cells
              << " provisional_mutual_d100="
              << provisional_thresholds[4].mutual_matches
              << " provisional_assigned_d100="
              << provisional_thresholds[4].assigned_matches
              << " provisional_grid_d100="
              << provisional_thresholds[4].occupied_grid_cells
              << " grid_cells=" << provisional.occupied_grid_cells
              << " provisional_reprojection_rmse_px="
              << provisional.reprojection_rmse_pixels
              << " provisional_xyz_rmse_m="
              << provisional.xyz_rmse_meters
              << " pnp_attempted="
              << (shadow_result.pnp_attempted ? 1 : 0)
              << " pnp_succeeded="
              << (shadow_result.pnp_succeeded ? 1 : 0)
              << " pnp_inliers=" << shadow_result.pnp_inliers
              << " refined_candidates_after_geometry="
              << refined.candidates_after_geometry
              << " refined_candidates_after_descriptor="
              << refined.candidates_after_descriptor
              << " refined_mutual_matches=" << refined.mutual_matches
              << " refined_assigned_matches=" << refined.assigned_matches
              << " refined_observation_descriptor_matches="
              << refined.observation_descriptor_matches
              << " refined_geometry_mutual_5cm="
              << refined_geometry[0].mutual_matches
              << " refined_geometry_assigned_5cm="
              << refined_geometry[0].assigned_matches
              << " refined_geometry_grid_5cm="
              << refined_geometry[0].occupied_grid_cells
              << " refined_geometry_mutual_10cm="
              << refined_geometry[1].mutual_matches
              << " refined_geometry_assigned_10cm="
              << refined_geometry[1].assigned_matches
              << " refined_geometry_grid_10cm="
              << refined_geometry[1].occupied_grid_cells
              << " refined_geometry_mutual_dynamic="
              << refined_geometry[2].mutual_matches
              << " refined_geometry_assigned_dynamic="
              << refined_geometry[2].assigned_matches
              << " refined_geometry_grid_dynamic="
              << refined_geometry[2].occupied_grid_cells
              << " refined_mutual_d50="
              << refined_thresholds[0].mutual_matches
              << " refined_assigned_d50="
              << refined_thresholds[0].assigned_matches
              << " refined_grid_d50="
              << refined_thresholds[0].occupied_grid_cells
              << " refined_mutual_d60="
              << refined_thresholds[1].mutual_matches
              << " refined_assigned_d60="
              << refined_thresholds[1].assigned_matches
              << " refined_grid_d60="
              << refined_thresholds[1].occupied_grid_cells
              << " refined_mutual_d70="
              << refined_thresholds[2].mutual_matches
              << " refined_assigned_d70="
              << refined_thresholds[2].assigned_matches
              << " refined_grid_d70="
              << refined_thresholds[2].occupied_grid_cells
              << " refined_mutual_d80="
              << refined_thresholds[3].mutual_matches
              << " refined_assigned_d80="
              << refined_thresholds[3].assigned_matches
              << " refined_grid_d80="
              << refined_thresholds[3].occupied_grid_cells
              << " refined_mutual_d100="
              << refined_thresholds[4].mutual_matches
              << " refined_assigned_d100="
              << refined_thresholds[4].assigned_matches
              << " refined_grid_d100="
              << refined_thresholds[4].occupied_grid_cells
              << " refined_grid_cells=" << refined.occupied_grid_cells
              << " refined_reprojection_rmse_px="
              << refined.reprojection_rmse_pixels
              << " refined_xyz_rmse_m=" << refined.xyz_rmse_meters
              << " frame_id=" << mCurrentFrame.mnId << std::endl;

        const RgbdMapReanchorMatcher::GeometryThresholdSupport&
            five_centimeter_support = provisional_geometry[0];
        const bool geometry_reanchor_eligible =
            internal::ShouldActivateRgbdGeometryReanchor(
                provisional.input_valid,
                five_centimeter_support.assigned_matches,
                five_centimeter_support.occupied_grid_cells);
        const std::vector<MapPoint*>& geometry_associations =
            shadow_result.geometry_five_centimeter_associations;
        bool geometry_identity_valid =
            geometry_reanchor_eligible && expected_map &&
            expected_map == mpAtlas->GetCurrentMap() &&
            expected_map->GetId() == expected_map_id &&
            expected_map->GetCoordinateEpoch() ==
                expected_coordinate_epoch &&
            expected_map->GetMapChangeIndex() == expected_map_revision &&
            geometry_associations.size() ==
                static_cast<std::size_t>(mCurrentFrame.N);
        std::unordered_set<MapPoint*> geometry_map_points;
        std::unordered_set<std::uint64_t> geometry_map_point_ids;
        geometry_map_points.reserve(
            five_centimeter_support.assigned_matches);
        geometry_map_point_ids.reserve(
            five_centimeter_support.assigned_matches);
        int geometry_seeded_associations = 0;
        if (geometry_identity_valid) {
            for (MapPoint* const map_point : geometry_associations) {
                if (!map_point) {
                    continue;
                }
                if (map_point->isBad() || map_point->GetReplaced() ||
                    map_point->GetMap() != expected_map ||
                    map_point->Observations() <= 0 ||
                    local_map_point_set.count(map_point) == 0U ||
                    !geometry_map_points.insert(map_point).second ||
                    !geometry_map_point_ids.insert(map_point->mnId).second) {
                    geometry_identity_valid = false;
                    break;
                }
                ++geometry_seeded_associations;
            }
            geometry_identity_valid =
                geometry_identity_valid &&
                geometry_seeded_associations ==
                    five_centimeter_support.assigned_matches;
            geometry_identity_valid =
                geometry_identity_valid &&
                expected_map == mpAtlas->GetCurrentMap() &&
                expected_map->GetId() == expected_map_id &&
                expected_map->GetCoordinateEpoch() ==
                    expected_coordinate_epoch &&
                expected_map->GetMapChangeIndex() == expected_map_revision;
        }
        if (geometry_identity_valid) {
            mCurrentFrame.mvpMapPoints = geometry_associations;
            mCurrentFrame.mvbOutlier.assign(mCurrentFrame.N, false);
            mCurrentFrame.mmProjectPoints.clear();
            mCurrentFrame.mmMatchedInImage.clear();
            geometry_reanchor_activated = true;
            geometry_reanchor_expected_map = expected_map;
            geometry_reanchor_expected_map_id = expected_map_id;
            geometry_reanchor_expected_coordinate_epoch =
                expected_coordinate_epoch;
            geometry_reanchor_expected_map_revision =
                expected_map_revision;
        }
        AINFO << "ATLAS_RGBD_GEOMETRY_REANCHOR_SEED"
              << " eligible=" << (geometry_reanchor_eligible ? 1 : 0)
              << " identity_valid=" << (geometry_identity_valid ? 1 : 0)
              << " activated=" << (geometry_reanchor_activated ? 1 : 0)
              << " assigned_5cm="
              << five_centimeter_support.assigned_matches
              << " grid_5cm="
              << five_centimeter_support.occupied_grid_cells
              << " seeded=" << geometry_seeded_associations
              << " frame_id=" << mCurrentFrame.mnId << std::endl;
    }
    const int associations_before_search = count_frame_associations();
    SearchLocalPoints(!transactional,
                      transactional ? &deferred_visible_points : nullptr,
                      transaction);
    const int associations_after_search = count_frame_associations();

    int geometry_valid_associations = 0;
    int geometry_depth_checked = 0;
    int geometry_reprojection_consistent = 0;
    int geometry_xyz_consistent = 0;
    int geometry_joint_consistent = 0;
    double geometry_squared_reprojection_error = 0.0;
    double geometry_squared_xyz_error = 0.0;
    std::array<bool, 16> geometry_occupied_grid_cells{};
    if (internal::ShouldUseStrictRgbdTemporalLocalMapProjection(
            transactional, mSensor == System::IMU_RGBD,
            imu_rgbd_visual_recovery_active_)) {
        Map* const current_map = mpAtlas->GetCurrentMap();
        const autonomy::common::math::SE3f camera_from_world =
            mCurrentFrame.GetPose();
        const double fx = static_cast<double>(mCurrentFrame.mK_(0, 0));
        const double fy = static_cast<double>(mCurrentFrame.mK_(1, 1));
        const double cx = static_cast<double>(mCurrentFrame.mK_(0, 2));
        const double cy = static_cast<double>(mCurrentFrame.mK_(1, 2));
        const double image_width =
            static_cast<double>(Frame::mnMaxX - Frame::mnMinX);
        const double image_height =
            static_cast<double>(Frame::mnMaxY - Frame::mnMinY);
        const bool intrinsics_valid =
            std::isfinite(fx) && std::isfinite(fy) &&
            std::isfinite(cx) && std::isfinite(cy) && fx > 0.0 && fy > 0.0;
        for (int index = 0; index < mCurrentFrame.N; ++index) {
            MapPoint* const map_point = mCurrentFrame.mvpMapPoints[index];
            if (!map_point || !current_map || map_point->isBad() ||
                map_point->GetMap() != current_map ||
                map_point->Observations() <= 0 || !intrinsics_valid ||
                static_cast<std::size_t>(index) >=
                    mCurrentFrame.mvKeysUn.size() ||
                static_cast<std::size_t>(index) >=
                    mCurrentFrame.mvDepth.size()) {
                continue;
            }
            ++geometry_valid_associations;

            const double measured_depth =
                static_cast<double>(mCurrentFrame.mvDepth[index]);
            const cv::Point2f& observed_pixel =
                mCurrentFrame.mvKeysUn[index].pt;
            const Eigen::Vector3f world_point = map_point->GetWorldPos();
            const Eigen::Vector3f predicted_point =
                camera_from_world * world_point;
            if (!world_point.allFinite() || !predicted_point.allFinite() ||
                !std::isfinite(measured_depth) || measured_depth <= 0.0 ||
                predicted_point.z() <= 0.0F ||
                !std::isfinite(observed_pixel.x) ||
                !std::isfinite(observed_pixel.y)) {
                continue;
            }

            const Eigen::Vector3d measured_point(
                (static_cast<double>(observed_pixel.x) - cx) *
                    measured_depth / fx,
                (static_cast<double>(observed_pixel.y) - cy) *
                    measured_depth / fy,
                measured_depth);
            const Eigen::Vector3d predicted_point_double =
                predicted_point.cast<double>();
            const double projected_x =
                fx * predicted_point_double.x() /
                    predicted_point_double.z() +
                cx;
            const double projected_y =
                fy * predicted_point_double.y() /
                    predicted_point_double.z() +
                cy;
            const double reprojection_error = std::hypot(
                projected_x - static_cast<double>(observed_pixel.x),
                projected_y - static_cast<double>(observed_pixel.y));
            const double xyz_error =
                (predicted_point_double - measured_point).norm();
            if (!std::isfinite(reprojection_error) ||
                !std::isfinite(xyz_error)) {
                continue;
            }
            ++geometry_depth_checked;
            geometry_squared_reprojection_error +=
                reprojection_error * reprojection_error;
            geometry_squared_xyz_error += xyz_error * xyz_error;
            const bool reprojection_consistent =
                internal::IsRgbdTemporalMapReprojectionConsistent(
                    reprojection_error);
            const bool xyz_consistent =
                internal::IsRgbdTemporalMapXyzConsistent(
                    static_cast<double>(predicted_point.z()), measured_depth,
                    xyz_error);
            geometry_reprojection_consistent +=
                reprojection_consistent ? 1 : 0;
            geometry_xyz_consistent += xyz_consistent ? 1 : 0;
            if (!reprojection_consistent || !xyz_consistent) {
                continue;
            }
            ++geometry_joint_consistent;
            if (image_width > 0.0 && image_height > 0.0 &&
                observed_pixel.x >= Frame::mnMinX &&
                observed_pixel.x <= Frame::mnMaxX &&
                observed_pixel.y >= Frame::mnMinY &&
                observed_pixel.y <= Frame::mnMaxY) {
                const int column = std::clamp(
                    static_cast<int>((observed_pixel.x - Frame::mnMinX) /
                                     image_width * 4.0),
                    0, 3);
                const int row = std::clamp(
                    static_cast<int>((observed_pixel.y - Frame::mnMinY) /
                                     image_height * 4.0),
                    0, 3);
                geometry_occupied_grid_cells[row * 4 + column] = true;
            }
        }
        const double reprojection_rmse =
            geometry_depth_checked > 0
                ? std::sqrt(geometry_squared_reprojection_error /
                            geometry_depth_checked)
                : std::numeric_limits<double>::infinity();
        const double xyz_rmse =
            geometry_depth_checked > 0
                ? std::sqrt(geometry_squared_xyz_error /
                            geometry_depth_checked)
                : std::numeric_limits<double>::infinity();
        const int joint_grid_cells = static_cast<int>(std::count(
            geometry_occupied_grid_cells.begin(),
            geometry_occupied_grid_cells.end(), true));
        AINFO << "ATLAS_RGBD_TEMPORAL_TLM_GEOMETRY"
              << " valid_associations=" << geometry_valid_associations
              << " depth_checked=" << geometry_depth_checked
              << " reprojection_consistent="
              << geometry_reprojection_consistent
              << " xyz_consistent=" << geometry_xyz_consistent
              << " joint_consistent=" << geometry_joint_consistent
              << " joint_grid_cells=" << joint_grid_cells
              << " reprojection_rmse_px=" << reprojection_rmse
              << " xyz_rmse_m=" << xyz_rmse
              << " frame_id=" << mCurrentFrame.mnId << std::endl;
    }

    // TOO check outliers before PO
    int aux1 = 0, aux2 = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
        if (mCurrentFrame.mvpMapPoints[i]) {
            aux1++;
            if (mCurrentFrame.mvbOutlier[i])
                aux2++;
        }
    const int associations_before_optimization = aux1;
    const int outliers_before_optimization = aux2;

    int inliers;
    if (internal::ShouldUseVisualPoseOptimization(
            mpAtlas->isImuInitialized(), force_visual_pose_optimization))
        Optimizer::PoseOptimization(&mCurrentFrame);
    else {
        if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
            AINFO << "TLM: PoseOptimization ";
            Optimizer::PoseOptimization(&mCurrentFrame);
        } else {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if (!mbMapUpdated)  //  && (mnMatchesInliers>30))
            {
                ADEBUG << "TLM: PoseInertialOptimizationLastFrame ";
                inliers = Optimizer::PoseInertialOptimizationLastFrame(
                    &mCurrentFrame);  // ,
                                      // !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            } else {
                ADEBUG << "TLM: PoseInertialOptimizationLastKeyFrame ";
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(
                    &mCurrentFrame);  // ,
                                      // !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
        if (mCurrentFrame.mvpMapPoints[i]) {
            aux1++;
            if (mCurrentFrame.mvbOutlier[i])
                aux2++;
        }
    const int associations_after_optimization = aux1;
    const int outliers_after_optimization = aux2;

    if (geometry_reanchor_activated) {
        Map* const current_map = mpAtlas->GetCurrentMap();
        const bool identity_stable =
            current_map && current_map == geometry_reanchor_expected_map &&
            current_map->GetId() == geometry_reanchor_expected_map_id &&
            current_map->GetCoordinateEpoch() ==
                geometry_reanchor_expected_coordinate_epoch &&
            current_map->GetMapChangeIndex() ==
                geometry_reanchor_expected_map_revision;
        std::vector<RgbdMapReanchorMatcher::AssociationView>
            association_views;
        association_views.reserve(associations_after_optimization);
        for (int index = 0; index < mCurrentFrame.N; ++index) {
            MapPoint* const map_point = mCurrentFrame.mvpMapPoints[index];
            if (!map_point) {
                continue;
            }
            RgbdMapReanchorMatcher::AssociationView association;
            association.feature_index = index;
            association.map_point = map_point;
            association.map_point_id = map_point->mnId;
            association.optimizer_outlier =
                static_cast<std::size_t>(index) >=
                    mCurrentFrame.mvbOutlier.size() ||
                mCurrentFrame.mvbOutlier[index];
            association.valid_for_expected_map =
                identity_stable && !map_point->isBad() &&
                !map_point->GetReplaced() &&
                map_point->GetMap() == geometry_reanchor_expected_map &&
                map_point->Observations() > 0;
            if (association.valid_for_expected_map) {
                association.world_point = map_point->GetWorldPos();
            }
            association_views.push_back(association);
        }
        const RgbdMapReanchorMatcher::FrameView optimized_frame_view = {
            &mCurrentFrame.mDescriptors,
            &mCurrentFrame.mvKeysUn,
            &mCurrentFrame.mvDepth,
            mCurrentFrame.mK_,
            mCurrentFrame.N,
            Frame::mnMinX,
            Frame::mnMaxX,
            Frame::mnMinY,
            Frame::mnMaxY,
        };
        const RgbdMapReanchorMatcher matcher;
        const RgbdMapReanchorMatcher::AssociationValidation validation =
            matcher.ValidateFiveCentimeterAssociations(
                optimized_frame_view, association_views,
                mCurrentFrame.GetPose());
        Map* const map_after_validation = mpAtlas->GetCurrentMap();
        const bool identity_stable_after_validation =
            identity_stable && map_after_validation &&
            map_after_validation == geometry_reanchor_expected_map &&
            map_after_validation->GetId() ==
                geometry_reanchor_expected_map_id &&
            map_after_validation->GetCoordinateEpoch() ==
                geometry_reanchor_expected_coordinate_epoch &&
            map_after_validation->GetMapChangeIndex() ==
                geometry_reanchor_expected_map_revision;
        const bool post_optimization_valid =
            internal::PassesRgbdGeometryReanchorPostOptimization(
                identity_stable_after_validation, validation.input_valid,
                validation.consistent_associations,
                validation.occupied_grid_cells);
        AINFO << "ATLAS_RGBD_GEOMETRY_REANCHOR_POST_OPT"
              << " identity_stable_before=" << (identity_stable ? 1 : 0)
              << " identity_stable_after="
              << (identity_stable_after_validation ? 1 : 0)
              << " input_valid=" << (validation.input_valid ? 1 : 0)
              << " input_associations="
              << validation.input_associations
              << " valid_unique="
              << validation.valid_unique_associations
              << " consistent_5cm="
              << validation.consistent_associations
              << " grid_5cm=" << validation.occupied_grid_cells
              << " accepted=" << (post_optimization_valid ? 1 : 0)
              << " frame_id=" << mCurrentFrame.mnId << std::endl;
        if (!post_optimization_valid) {
            if (validation_inliers) {
                *validation_inliers = validation.consistent_associations;
            }
            return false;
        }

        std::vector<MapPoint*> strictly_validated_associations(
            mCurrentFrame.N, nullptr);
        for (int feature_index : validation.accepted_feature_indices) {
            if (feature_index >= 0 && feature_index < mCurrentFrame.N) {
                strictly_validated_associations[feature_index] =
                    mCurrentFrame.mvpMapPoints[feature_index];
            }
        }
        mCurrentFrame.mvpMapPoints =
            std::move(strictly_validated_associations);
        mCurrentFrame.mvbOutlier.assign(mCurrentFrame.N, false);
    }

    int matches_inliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                if (!transactional) {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                }
                if (!mbOnlyTracking) {
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        ++matches_inliers;
                } else
                    ++matches_inliers;
            } else if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(nullptr);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (validation_inliers) {
        *validation_inliers = matches_inliers;
    }
    if (transactional) {
        AINFO << "ATLAS_RGBD_TEMPORAL_TLM"
              << " local_keyframes=" << local_keyframe_count
              << " local_map_points=" << local_map_point_count
              << " associations_before_local_map="
              << associations_before_local_map
              << " associations_before_search=" << associations_before_search
              << " associations_after_search=" << associations_after_search
              << " associations_before_optimization="
              << associations_before_optimization
              << " outliers_before_optimization="
              << outliers_before_optimization
              << " associations_after_optimization="
              << associations_after_optimization
              << " outliers_after_optimization="
              << outliers_after_optimization
              << " final_local_inliers=" << matches_inliers
              << " required_inliers=" << required_minimum_inliers
              << " frame_id=" << mCurrentFrame.mnId << std::endl;
    }
    if (required_minimum_inliers > 0 &&
        matches_inliers < required_minimum_inliers) {
        return false;
    }
    if (!transactional) {
        mnMatchesInliers = matches_inliers;
        mpLocalMapper->mnMatchesInliers = matches_inliers;
    }
    const auto commit_transactional_tracking = [&]() {
        if (!transactional) {
            return;
        }
        ++mTrackedFr;
        for (MapPoint* point : deferred_visible_points) {
            point->IncreaseVisible();
        }
        for (int index = 0; index < mCurrentFrame.N; ++index) {
            if (mCurrentFrame.mvpMapPoints[index] &&
                !mCurrentFrame.mvbOutlier[index]) {
                mCurrentFrame.mvpMapPoints[index]->IncreaseFound();
            }
        }
        mnMatchesInliers = matches_inliers;
        mpLocalMapper->mnMatchesInliers = matches_inliers;
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
    };
    const bool recently_relocalized =
        mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames;
    if (internal::FailsRecentRelocalizationInlierThreshold(
            recently_relocalized, matches_inliers,
            force_visual_pose_optimization))
        return false;

    if ((mState == RECENTLY_LOST) &&
        internal::HasEnoughRecentlyLostInliers(matches_inliers,
                                               mSensor == System::IMU_RGBD)) {
        commit_transactional_tracking();
        return true;
    }

    if (mSensor == System::IMU_MONOCULAR) {
        if ((matches_inliers < 15 && mpAtlas->isImuInitialized()) ||
            (matches_inliers < 50 && !mpAtlas->isImuInitialized())) {
            return false;
        } else {
            commit_transactional_tracking();
            return true;
        }
    } else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        if (matches_inliers < 15) {
            return false;
        } else {
            commit_transactional_tracking();
            return true;
        }
    } else {
        if (matches_inliers < 30)
            return false;
        else {
            commit_transactional_tracking();
            return true;
        }
    }
}

bool Tracker::NeedNewKeyFrame() {
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD) &&
        !mpAtlas->GetCurrentMap()->isImuInitialized()) {
        return mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp >=
               mImuInitKeyFrameMinInterval;
    }

    if (mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            AINFO << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last
    // relocalisation
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&
        nKFs > mMaxFrames) {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMapperIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be
    // potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;

    if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
        int N =
            (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for (int i = 0; i < N; i++) {
            if (mCurrentFrame.mvDepth[i] > 0 &&
                mCurrentFrame.mvDepth[i] < mThDepth) {
                if (mCurrentFrame.mvpMapPoints[i] &&
                    !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
        mapping_telemetry_.RecordClosePointThresholds(
            nTrackedClose<100, nNonTrackedClose> 70);
        // Debug counters are intentionally omitted from the normal log path.
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO ||
    mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident
    about the stereo observations. thRefRatio = 0.9f;
    }*/

    if (mSensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    if (mpCamera2)
        thRefRatio = 0.75f;

    if (mSensor == System::IMU_MONOCULAR) {
        if (mnMatchesInliers > 350)  // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe
    // insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b =
        ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) &&
         bLocalMapperIdle);  // mpLocalMapper->KeyframesInQueue() < 2);
    // Condition 1c: tracking is weak or close-point creation is needed.
    const bool supports_noninertial_emergency =
        mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR &&
        mSensor != System::IMU_STEREO && mSensor != System::IMU_RGBD;
    const bool weak_tracking_emergency =
        supports_noninertial_emergency && mnMatchesInliers < nRefMatches * 0.25;
    const bool c1c = supports_noninertial_emergency &&
                     (weak_tracking_emergency || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of
    // visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers < nRefMatches * thRefRatio ||
                       bNeedToInsertClose)) &&
                     mnMatchesInliers > 15);

    // AINFO << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c
    // << "; c2=" << c2 << std::endl;
    //  Temporal condition for Inertial cases
    bool c3 = false;
    if (mpLastKeyFrame) {
        if (mSensor == System::IMU_MONOCULAR) {
            if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        } else if (mSensor == System::IMU_STEREO ||
                   mSensor == System::IMU_RGBD) {
            if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) ||
         mState == RECENTLY_LOST) &&
        (mSensor ==
         System::IMU_MONOCULAR))  // MODIFICATION_2, originally
                                  // ((((mnMatchesInliers<75) &&
                                  // (mnMatchesInliers>15)) ||
                                  // mState==RECENTLY_LOST) && ((mSensor ==
                                  // System::IMU_MONOCULAR)))
        c4 = true;
    else
        c4 = false;

    internal::KeyFrameAdmissionSignals admission_signals;
    admission_signals.c1a_max_frames_elapsed = c1a;
    admission_signals.c1b_min_frames_and_idle = c1b;
    admission_signals.c1c_noninertial_weak_or_close = c1c;
    admission_signals.c2_tracking_or_close_need = c2;
    admission_signals.c3_inertial_time_elapsed = c3;
    admission_signals.c4_imu_monocular_recovery = c4;
    admission_signals.need_close_point_insertion = bNeedToInsertClose;
    if (!internal::IsKeyFrameRequested(admission_signals)) {
        return false;
    }

    const bool local_mapper_initializing =
        !bLocalMapperIdle && mpLocalMapper->IsInitializing();
    const bool local_mapper_busy =
        !bLocalMapperIdle && !local_mapper_initializing;
    admission_signals.local_mapper_idle = bLocalMapperIdle;
    admission_signals.local_mapper_initializing = local_mapper_initializing;
    admission_signals.supports_busy_admission =
        mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR;
    if (local_mapper_busy) {
        admission_signals.queued_keyframes = mpLocalMapper->KeyframesInQueue();
    }

    const internal::KeyFrameAdmissionDecision admission =
        internal::DecideKeyFrameAdmission(admission_signals);

    const bool busy_emergency_request =
        local_mapper_busy && admission_signals.supports_busy_admission && c2 &&
        (weak_tracking_emergency || bNeedToInsertClose);

    if (mapping_telemetry_.enabled() && busy_emergency_request) {
        internal::BusyKeyFrameReason reason =
            internal::BusyKeyFrameReason::kWeakAndClose;
        if (weak_tracking_emergency && !bNeedToInsertClose) {
            reason = internal::BusyKeyFrameReason::kWeakOnly;
        } else if (!weak_tracking_emergency && bNeedToInsertClose) {
            reason = internal::BusyKeyFrameReason::kCloseOnly;
        }
        const bool admitted =
            admission ==
            internal::KeyFrameAdmissionDecision::kInterruptBaAndInsert;
        const bool queue_limit_deferred =
            !admitted && admission_signals.queued_keyframes >=
                             internal::kBusyKeyFrameQueueLimit;
        const std::uint64_t frame_gap =
            mCurrentFrame.mnId >= mnLastKeyFrameId
                ? static_cast<std::uint64_t>(mCurrentFrame.mnId -
                                             mnLastKeyFrameId)
                : 0;
        mapping_telemetry_.RecordBusyKeyFrameRequest(
            reason, admitted, queue_limit_deferred, frame_gap,
            mpLocalMapper->GetLbaCompletionGeneration());
    }

    if (admission ==
        internal::KeyFrameAdmissionDecision::kInsertWithoutInterrupt) {
        return true;
    }
    if (admission !=
        internal::KeyFrameAdmissionDecision::kInterruptBaAndInsert) {
        return false;
    }

    // Interrupt only after policy admits the emergency keyframe. Routine
    // cadence must let the current LBA finish.
    mpLocalMapper->InterruptBA();
    return true;
}

void Tracker::CreateNewKeyFrame() {
    if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
        return;

    if (!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF =
        new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if (mpAtlas->isImuInitialized())  //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if (mpLastKeyFrame) {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    } else
        AINFO << "No last KF in KF creation!!";

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD) {
        mpImuPreintegratedFromLastKF =
            new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
    }

    if (mSensor != System::MONOCULAR &&
        mSensor != System::IMU_MONOCULAR)  // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // AINFO << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            maxPoint = 100;

        std::vector<std::pair<float, int>> vDepthIdx;
        int N =
            (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for (int i = 0; i < N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(std::make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty()) {
            std::sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (std::size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->Observations() < 1) {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] =
                        static_cast<MapPoint*>(nullptr);
                }

                if (bCreateNew) {
                    Eigen::Vector3f x3D;

                    if (mCurrentFrame.Nleft == -1) {
                        mCurrentFrame.UnprojectStereo(i, x3D);
                    } else {
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP =
                        new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF, i);

                    // Check if it is a stereo observation in order to not
                    // duplicate mappoints
                    if (mCurrentFrame.Nleft != -1 &&
                        mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
                        mCurrentFrame
                            .mvpMapPoints[mCurrentFrame.Nleft +
                                          mCurrentFrame.mvLeftToRightMatch[i]] =
                            pNewMP;
                        pNewMP->AddObservation(
                            pKF, mCurrentFrame.Nleft +
                                     mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(
                            pNewMP, mCurrentFrame.Nleft +
                                        mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                } else {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
                    break;
                }
            }
            // AINFO << "new mps for stereo KF: " + to_string(nPoints);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    LogVisualSlamEvent("keyframe");
}

void Tracker::SearchLocalPoints(
    bool commit_statistics, std::vector<MapPoint*>* deferred_visible_points,
    internal::LocalMapTrackingTransaction* transaction) {
    // Do not search map points already matched
    for (std::vector<MapPoint*>::iterator
             vit = mCurrentFrame.mvpMapPoints.begin(),
             vend = mCurrentFrame.mvpMapPoints.end();
         vit != vend; vit++) {
        MapPoint* pMP = *vit;
        if (pMP) {
            if (pMP->isBad()) {
                *vit = static_cast<MapPoint*>(nullptr);
            } else {
                if (transaction) {
                    transaction->Capture(pMP);
                }
                if (commit_statistics) {
                    pMP->IncreaseVisible();
                } else if (deferred_visible_points) {
                    deferred_visible_points->push_back(pMP);
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (std::vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(),
                                          vend = mvpLocalMapPoints.end();
         vit != vend; vit++) {
        MapPoint* pMP = *vit;

        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        if (transaction) {
            transaction->Capture(pMP);
        }
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
            if (commit_statistics) {
                pMP->IncreaseVisible();
            } else if (deferred_visible_points) {
                deferred_visible_points->push_back(pMP);
            }
            nToMatch++;
        }
        if (pMP->mbTrackInView) {
            mCurrentFrame.mmProjectPoints[pMP->mnId] =
                cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    int projection_matches = 0;
    int projection_search_radius = 0;
    if (nToMatch > 0) {
        OrbMatcher matcher(0.8);
        int th = 1;
        if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
            th = 3;
        if (mpAtlas->isImuInitialized()) {
            if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th = 2;
            else
                th = 6;
        } else if (!mpAtlas->isImuInitialized() &&
                   (mSensor == System::IMU_MONOCULAR ||
                    mSensor == System::IMU_STEREO ||
                    mSensor == System::IMU_RGBD)) {
            th = 10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;

        if (mState == LOST ||
            mState == RECENTLY_LOST)  // Lost for less than 1 second
            th = 15;                  // 15

        if (internal::ShouldUseStrictRgbdTemporalLocalMapProjection(
                transaction != nullptr, mSensor == System::IMU_RGBD,
                imu_rgbd_visual_recovery_active_)) {
            th = internal::kRgbdTemporalLocalMapProjectionRadius;
        }

        projection_search_radius = th;
        projection_matches = matcher.SearchByProjection(
            mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints,
            mpLocalMapper->mThFarPoints);
    }
    if (transaction) {
        const int associations_after_projection = static_cast<int>(
            std::count_if(mCurrentFrame.mvpMapPoints.begin(),
                          mCurrentFrame.mvpMapPoints.end(),
                          [](const MapPoint* map_point) {
                              return map_point != nullptr;
                          }));
        AINFO << "ATLAS_RGBD_TEMPORAL_TLM_SEARCH"
              << " frustum_candidates=" << nToMatch
              << " projection_matches=" << projection_matches
              << " search_radius=" << projection_search_radius
              << " associations_after_projection="
              << associations_after_projection
              << " frame_id=" << mCurrentFrame.mnId << std::endl;
    }
}

void Tracker::UpdateLocalMap(
    bool publish_reference_points,
    internal::LocalMapTrackingTransaction* transaction) {
    // This is for visualization
    if (publish_reference_points) {
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
    }

    // Update
    UpdateLocalKeyFrames(transaction);
    UpdateLocalPoints(transaction);
}

void Tracker::UpdateLocalPoints(
    internal::LocalMapTrackingTransaction* transaction) {
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    for (std::vector<KeyFrame*>::const_reverse_iterator
             itKF = mvpLocalKeyFrames.rbegin(),
             itEndKF = mvpLocalKeyFrames.rend();
         itKF != itEndKF; ++itKF) {
        KeyFrame* pKF = *itKF;
        const std::vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for (std::vector<MapPoint*>::const_iterator itMP = vpMPs.begin(),
                                                    itEndMP = vpMPs.end();
             itMP != itEndMP; itMP++) {
            MapPoint* pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if (!pMP->isBad()) {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                if (transaction) {
                    transaction->Capture(pMP);
                }
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void Tracker::UpdateLocalKeyFrames(
    internal::LocalMapTrackingTransaction* transaction) {
    // Each map point vote for the keyframes in which it has been observed
    std::map<KeyFrame*, int> keyframeCounter;
    Map* const current_map = mpAtlas->GetCurrentMap();
    const bool recently_relocalized =
        mCurrentFrame.mnId < mnLastRelocFrameId + 2;
    bool current_frame_has_visual_votes = false;
    if (mSensor == System::IMU_RGBD && imu_rgbd_visual_recovery_active_) {
        for (MapPoint* map_point : mCurrentFrame.mvpMapPoints) {
            if (map_point && !map_point->isBad() &&
                map_point->Observations() > 0) {
                current_frame_has_visual_votes = true;
                break;
            }
        }
    }
    const bool merge_recovery_votes =
        internal::ShouldMergeRgbdRecoveryLocalKeyFrameVotes(
            transaction != nullptr, mSensor == System::IMU_RGBD,
            imu_rgbd_visual_recovery_active_,
            current_frame_has_visual_votes);
    std::unordered_set<MapPoint*> voted_recovery_map_points;
    if (merge_recovery_votes) {
        voted_recovery_map_points.reserve(mCurrentFrame.mvpMapPoints.size() +
                                          mLastFrame.mvpMapPoints.size());
        constexpr int kMaximumRecoveryReplacementHops = 16;
        const auto add_recovery_frame_votes = [&](Frame* frame) {
            for (MapPoint*& frame_map_point : frame->mvpMapPoints) {
                if (!frame_map_point) {
                    continue;
                }
                MapPoint* resolved_map_point = frame_map_point;
                std::array<MapPoint*, kMaximumRecoveryReplacementHops + 1>
                    replacement_chain{};
                int replacement_chain_size = 1;
                replacement_chain[0] = resolved_map_point;
                bool replacement_chain_valid = true;
                while (MapPoint* replacement =
                           resolved_map_point->GetReplaced()) {
                    if (replacement_chain_size >=
                            static_cast<int>(replacement_chain.size()) ||
                        std::find(
                            replacement_chain.begin(),
                            replacement_chain.begin() +
                                replacement_chain_size,
                            replacement) !=
                            replacement_chain.begin() +
                                replacement_chain_size) {
                        replacement_chain_valid = false;
                        break;
                    }
                    replacement_chain[replacement_chain_size++] = replacement;
                    resolved_map_point = replacement;
                }
                if (!replacement_chain_valid) {
                    continue;
                }
                if (resolved_map_point->isBad() ||
                    resolved_map_point->GetMap() != current_map ||
                    resolved_map_point->Observations() <= 0) {
                    continue;
                }
                frame_map_point = resolved_map_point;
                if (!voted_recovery_map_points.insert(resolved_map_point)
                         .second) {
                    continue;
                }
                const std::map<KeyFrame*, std::tuple<int, int>> observations =
                    resolved_map_point->GetObservations();
                for (const auto& [keyframe, observation] : observations) {
                    static_cast<void>(observation);
                    if (!keyframe || keyframe->isBad() ||
                        keyframe->GetMap() != current_map) {
                        continue;
                    }
                    ++keyframeCounter[keyframe];
                }
            }
        };
        add_recovery_frame_votes(&mCurrentFrame);
        add_recovery_frame_votes(&mLastFrame);
    } else if (internal::ShouldUseCurrentFrameForLocalKeyFrameVoting(
                   mpAtlas->isImuInitialized(), recently_relocalized,
                   mSensor == System::IMU_RGBD,
                   imu_rgbd_visual_recovery_active_,
                   current_frame_has_visual_votes)) {
        for (int i = 0; i < mCurrentFrame.N; ++i) {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    const std::map<KeyFrame*, std::tuple<int, int>>
                        observations = pMP->GetObservations();
                    for (std::map<KeyFrame*,
                                  std::tuple<int, int>>::const_iterator
                             it = observations.begin(),
                             itend = observations.end();
                         it != itend; ++it) {
                        ++keyframeCounter[it->first];
                    }
                } else {
                    mCurrentFrame.mvpMapPoints[i] = nullptr;
                }
            }
        }
    } else {
        for (int i = 0; i < mLastFrame.N; ++i) {
            // Using lastframe since current frame has not matches yet.
            if (mLastFrame.mvpMapPoints[i]) {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if (!pMP) {
                    continue;
                }
                if (!pMP->isBad()) {
                    const std::map<KeyFrame*, std::tuple<int, int>>
                        observations = pMP->GetObservations();
                    for (std::map<KeyFrame*,
                                  std::tuple<int, int>>::const_iterator
                             it = observations.begin(),
                             itend = observations.end();
                         it != itend; ++it) {
                        ++keyframeCounter[it->first];
                    }
                } else {
                    mLastFrame.mvpMapPoints[i] = nullptr;
                }
            }
        }
    }

    if (transaction) {
        AINFO << "ATLAS_RGBD_TEMPORAL_TLM_VOTES"
              << " merged=" << (merge_recovery_votes ? 1 : 0)
              << " unique_map_points=" << voted_recovery_map_points.size()
              << " voting_keyframes=" << keyframeCounter.size()
              << " frame_id=" << mCurrentFrame.mnId << std::endl;
    }

    int max = 0;
    KeyFrame* pKFmax = static_cast<KeyFrame*>(nullptr);
    // This cap only bounds recovery seeds and graph/temporal expansion. Every
    // valid keyframe with a visual vote remains in the local map.
    constexpr std::size_t kMaximumLocalKeyFrameCount = 80;
    const auto mark_local_keyframe = [&](KeyFrame* keyframe) {
        if (transaction) {
            transaction->Capture(keyframe);
        }
        keyframe->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    };

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // Find the keyframe with the most votes before adding all voting keyframes.
    for (std::map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(),
                                                  itEnd = keyframeCounter.end();
         it != itEnd; ++it) {
        KeyFrame* pKF = it->first;
        if (pKF->isBad())
            continue;
        if (it->second > max) {
            max = it->second;
            pKFmax = pKF;
        }
    }

    if (pKFmax) {
        mvpLocalKeyFrames.push_back(pKFmax);
        mark_local_keyframe(pKFmax);
    }
    // Add the remaining voting keyframes without sorting the hot path.
    for (std::map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(),
                                                  itEnd = keyframeCounter.end();
         it != itEnd; ++it) {
        KeyFrame* pKF = it->first;
        if (pKF->isBad() || pKF == pKFmax)
            continue;
        mvpLocalKeyFrames.push_back(pKF);
        mark_local_keyframe(pKF);
    }

    KeyFrame* recovery_reference_seed = nullptr;
    const bool seed_recovery_local_keyframes =
        internal::ShouldSeedImuRgbdRecoveryLocalKeyFrames(
            mSensor == System::IMU_RGBD, imu_rgbd_visual_recovery_active_,
            current_frame_has_visual_votes);
    if (seed_recovery_local_keyframes) {
        int recovery_seed_count = 0;
        const auto append_recovery_seed = [&](KeyFrame* candidate) {
            const bool candidate_valid = candidate && !candidate->isBad();
            if (!candidate_valid || candidate->GetMap() != current_map ||
                mvpLocalKeyFrames.size() >= kMaximumLocalKeyFrameCount) {
                return;
            }

            bool already_present = false;
            for (KeyFrame* local_keyframe : mvpLocalKeyFrames) {
                if (local_keyframe == candidate) {
                    already_present = true;
                    break;
                }
            }
            if (!internal::ShouldAppendImuRgbdRecoveryReferenceCandidate(
                    candidate_valid, already_present, recovery_seed_count)) {
                return;
            }

            mvpLocalKeyFrames.push_back(candidate);
            mark_local_keyframe(candidate);
            if (!recovery_reference_seed) {
                recovery_reference_seed = candidate;
            }
            ++recovery_seed_count;
        };

        append_recovery_seed(mpReferenceKF);
        append_recovery_seed(mpLastKeyFrame);
        if (mpLastKeyFrame && !mpLastKeyFrame->isBad() &&
            mpLastKeyFrame->GetMap() == current_map) {
            const std::vector<KeyFrame*> covisible_keyframes =
                mpLastKeyFrame->GetBestCovisibilityKeyFrames(
                    internal::kImuRgbdRecoveryCovisibilityCandidateLimit);
            for (KeyFrame* covisible_keyframe : covisible_keyframes) {
                append_recovery_seed(covisible_keyframe);
            }
        }
    }

    // Include also some not-already-included keyframes that are neighbors to
    // already-included keyframes
    const std::size_t local_keyframe_count_to_expand = mvpLocalKeyFrames.size();
    for (std::size_t keyframe_index = 0;
         keyframe_index < local_keyframe_count_to_expand &&
         mvpLocalKeyFrames.size() < kMaximumLocalKeyFrameCount;
         ++keyframe_index) {
        KeyFrame* pKF = mvpLocalKeyFrames[keyframe_index];

        const std::vector<KeyFrame*> vNeighs =
            pKF->GetBestCovisibilityKeyFrames(10);

        for (std::vector<KeyFrame*>::const_iterator
                 itNeighKF = vNeighs.begin(),
                 itEndNeighKF = vNeighs.end();
             itNeighKF != itEndNeighKF; itNeighKF++) {
            KeyFrame* pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad()) {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId &&
                    mvpLocalKeyFrames.size() < kMaximumLocalKeyFrameCount) {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    mark_local_keyframe(pNeighKF);
                    break;
                }
            }
        }

        const std::set<KeyFrame*> spChilds = pKF->GetChilds();
        for (std::set<KeyFrame*>::const_iterator sit = spChilds.begin(),
                                                 send = spChilds.end();
             sit != send; sit++) {
            KeyFrame* pChildKF = *sit;
            if (!pChildKF->isBad()) {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId &&
                    mvpLocalKeyFrames.size() < kMaximumLocalKeyFrameCount) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    mark_local_keyframe(pChildKF);
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if (pParent && !pParent->isBad()) {
            if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId &&
                mvpLocalKeyFrames.size() < kMaximumLocalKeyFrameCount) {
                mvpLocalKeyFrames.push_back(pParent);
                mark_local_keyframe(pParent);
                break;
            }
        }
    }

    // Add recent temporal KFs (mainly for IMU).
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD) &&
        mvpLocalKeyFrames.size() < kMaximumLocalKeyFrameCount) {
        KeyFrame* temporal_keyframe = mCurrentFrame.mpLastKeyFrame;

        constexpr int kTemporalKeyFrameLimit = 20;
        for (int i = 0; i < kTemporalKeyFrameLimit; ++i) {
            if (!temporal_keyframe ||
                mvpLocalKeyFrames.size() >= kMaximumLocalKeyFrameCount) {
                break;
            }
            KeyFrame* candidate = temporal_keyframe;
            temporal_keyframe = temporal_keyframe->mPrevKF;
            if (candidate->isBad() || candidate->GetMap() != current_map ||
                candidate->mnTrackReferenceForFrame == mCurrentFrame.mnId) {
                continue;
            }
            mvpLocalKeyFrames.push_back(candidate);
            mark_local_keyframe(candidate);
        }
    }

    if (pKFmax) {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    } else if (recovery_reference_seed &&
               (!mpReferenceKF || mpReferenceKF->isBad())) {
        mpReferenceKF = recovery_reference_seed;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracker::Relocalization() {
    AINFO << "Starting relocalization";
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for
    // relocalisation
    std::vector<KeyFrame*> vpCandidateKFs =
        mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame,
                                                     mpAtlas->GetCurrentMap());

    if (vpCandidateKFs.empty()) {
        AINFO << "There are not candidates";
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    OrbMatcher matcher(0.75, true);

    std::vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    std::vector<std::vector<MapPoint*>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    std::vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++) {
        KeyFrame* pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else {
            int nmatches =
                matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                MLPnPsolver* pSolver =
                    new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(
                    0.99, 10, 300, 6, 0.5,
                    5.991);  // This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    OrbMatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nKFs; i++) {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            std::vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool bTcw =
                pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (bTcw) {
                autonomy::common::math::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.mTcw);

                std::set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        mCurrentFrame.mvpMapPoints[j] =
                            vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    } else
                        mCurrentFrame.mvpMapPoints[j] = nullptr;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < mCurrentFrame.N; io++)
                    if (mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io] =
                            static_cast<MapPoint*>(nullptr);

                // If few inliers, search by projection in a coarse window and
                // optimize again
                if (nGood < 50) {
                    int nadditional = matcher2.SearchByProjection(
                        mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by
                        // projection again in a narrower window the camera has
                        // been already optimized with many points
                        if (nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                if (mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(
                                        mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(
                                mCurrentFrame, vpCandidateKFs[i], sFound, 3,
                                64);

                            // Final optimization
                            if (nGood + nadditional >= 50) {
                                nGood =
                                    Optimizer::PoseOptimization(&mCurrentFrame);

                                for (int io = 0; io < mCurrentFrame.N; io++)
                                    if (mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io] =
                                            nullptr;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and
                // continue
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        return false;
    } else {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        AINFO << "Relocalized!!" << std::endl;
        return true;
    }
}

void Tracker::Reset(bool bLocMap) {
    AINFO << "System Reseting";
    rgbd_temporal_pose_estimator_.Reset();
    rgbd_temporal_odometry_state_.Reset();
    imu_rgbd_visual_recovery_active_ = false;

    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap) {
        AINFO << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        AINFO << "done";
    }

    // Reset Loop Closing
    AINFO << "Reseting Loop Closing...";
    mpLoopCloser->RequestReset();
    AINFO << "done";

    // Clear BoW Database
    AINFO << "Reseting Database...";
    mpKeyFrameDB->clear();
    AINFO << "done";

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
        mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit = false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(nullptr);
    mpLastKeyFrame = static_cast<KeyFrame*>(nullptr);
    mvIniMatches.clear();

    if (mpViewer)
        mpViewer->Release();

    AINFO << "   End reseting! ";
}

void Tracker::ResetActiveMap(bool bLocMap) {
    AINFO << "Active map Reseting";
    rgbd_temporal_pose_estimator_.Reset();
    rgbd_temporal_odometry_state_.Reset();
    imu_rgbd_visual_recovery_active_ = false;
    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap) {
        AINFO << "Reseting Local Mapper...";
        mpLocalMapper->RequestResetActiveMap(pMap);
        ADEBUG << "done";
    }

    // Reset Loop Closing
    AINFO << "Resetting loop closing";
    mpLoopCloser->RequestResetActiveMap(pMap);
    AINFO << "done";

    // Clear BoW Database
    AINFO << "Reseting Database";
    mpKeyFrameDB->clearMap(pMap);  // Only clear the active map references
    AINFO << "done";

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();

    // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    // Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    // mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET;  // NOT_INITIALIZED;

    mbReadyToInitializate = false;

    std::list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    AINFO << "mnFirstFrameId = " << mnFirstFrameId << std::endl;
    for (Map* pMap : mpAtlas->GetAllMaps()) {
        if (pMap->GetAllKeyFrames().size() > 0) {
            if (index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    // AINFO << "First Frame id: " << index << endl;
    int num_lost = 0;
    AINFO << "mnInitialFrameId = " << mnInitialFrameId << std::endl;

    for (std::list<bool>::iterator ilbL = mlbLost.begin();
         ilbL != mlbLost.end(); ilbL++) {
        if (index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    AINFO << num_lost << " Frames set to lost" << std::endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(nullptr);
    mpLastKeyFrame = static_cast<KeyFrame*>(nullptr);
    mvIniMatches.clear();

    mbVelocity = false;

    if (mpViewer)
        mpViewer->Release();

    AINFO << "   End reseting! ";
}

std::vector<MapPoint*> Tracker::GetLocalMapMPS() {
    return mvpLocalMapPoints;
}

void Tracker::ChangeCalibration(const std::string& strSettingPath) {
    rgbd_temporal_pose_estimator_.Reset();
    rgbd_temporal_odometry_state_.Reset();
    imu_rgbd_visual_recovery_active_ = false;
    cv::FileStorage fConfigLoader(strSettingPath, cv::FileStorage::READ);
    float fx = fConfigLoader["Camera.fx"];
    float fy = fConfigLoader["Camera.fy"];
    float cx = fConfigLoader["Camera.cx"];
    float cy = fConfigLoader["Camera.cy"];

    mK_.setIdentity();
    mK_(0, 0) = fx;
    mK_(1, 1) = fy;
    mK_(0, 2) = cx;
    mK_(1, 2) = cy;

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fConfigLoader["Camera.k1"];
    DistCoef.at<float>(1) = fConfigLoader["Camera.k2"];
    DistCoef.at<float>(2) = fConfigLoader["Camera.p1"];
    DistCoef.at<float>(3) = fConfigLoader["Camera.p2"];
    const float k3 = fConfigLoader["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fConfigLoader["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracker::InformOnlyTracking(const bool& flag) {
    mbOnlyTracking = flag;
    if (flag) {
        rgbd_temporal_pose_estimator_.Reset();
        rgbd_temporal_odometry_state_.Reset();
        imu_rgbd_visual_recovery_active_ = false;
    }
}

void Tracker::UpdateFrameIMU(const float s, const IMU::Bias& b,
                             KeyFrame* pCurrentKeyFrame) {
    Map* pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    std::list<::autonomy::localization::atlas::KeyFrame*>::iterator lRit =
        mlpReferences.begin();
    std::list<bool>::iterator lbL = mlbLost.begin();
    for (auto lit = mlRelativeFramePoses.begin(),
              lend = mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lbL++) {
        if (*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while (pKF->isBad()) {
            pKF = pKF->GetParent();
        }

        if (pKF->GetMap() == pMap) {
            (*lit).translation() *= s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    // PreintegrateIMU always marks integration now; keep a bounded wait so a
    // racing frame cannot hang the local-mapping / loop-closing thread forever.
    for (int spin = 0; spin < 2000 && !mCurrentFrame.imuIsPreintegrated();
         ++spin) {
        usleep(500);
    }
    if (!mCurrentFrame.imuIsPreintegrated()) {
        AWARN << "UpdateFrameIMU: current frame never finished IMU "
                 "preintegration; forcing integrated flag";
        mCurrentFrame.setIntegrated();
    }

    if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
        mLastFrame.SetImuPoseVelocity(
            mLastFrame.mpLastKeyFrame->GetImuRotation(),
            mLastFrame.mpLastKeyFrame->GetImuPosition(),
            mLastFrame.mpLastKeyFrame->GetVelocity());
    } else {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 =
            mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 =
            mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 *
                mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 *
                    mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated) {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 =
            mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 =
            mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 =
            mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 *
                mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 *
                    mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 * mCurrentFrame.mpImuPreintegrated
                           ->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracker::NewDataset() {
    rgbd_temporal_pose_estimator_.Reset();
    rgbd_temporal_odometry_state_.Reset();
    imu_rgbd_visual_recovery_active_ = false;
    mnNumDataset++;
}

int Tracker::GetNumberDataset() {
    return mnNumDataset;
}

int Tracker::GetMatchesInliers() {
    return mnMatchesInliers;
}

void Tracker::SaveSubTrajectory(std::string strNameFile_frames,
                                std::string strNameFile_kf,
                                std::string strFolder) {
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracker::SaveSubTrajectory(std::string strNameFile_frames,
                                std::string strNameFile_kf, Map* pMap) {
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    if (!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracker::GetImageScale() {
    return mImageScale;
}

#ifdef REGISTER_LOOP
void Tracker::RequestStop() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool Tracker::Stop() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    if (mbStopRequested && !mbNotStop) {
        mbStopped = true;
        AINFO << "Tracker STOP" << std::endl;
        return true;
    }

    return false;
}

bool Tracker::stopRequested() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool Tracker::isStopped() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopped;
}

void Tracker::Release() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}
#endif

}  // namespace autonomy::localization::atlas
