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

#include "atlas/system.hpp"
#include <openssl/md5.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include "atlas/common/io/binary_archive.hpp"
#include "atlas/common/math/converter.hpp"
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {

namespace {
constexpr int kOpenCvThreadCount = 2;
constexpr auto kShutdownDrainTimeout = std::chrono::seconds(30);
constexpr auto kShutdownStablePeriod = std::chrono::milliseconds(200);
constexpr auto kLoopCorrectionWaitTimeout = std::chrono::seconds(30);
}  // namespace

bool internal::MapChangeObserver::Observe(std::uint64_t map_id, int revision) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        initialized_ = true;
        map_id_ = map_id;
        revision_ = revision;
        // Preserve the previous API behavior for a loaded map that already
        // contains a loop or global-BA revision on the first observation.
        return revision > 0;
    }

    const bool changed = map_id != map_id_ || revision != revision_;
    map_id_ = map_id;
    revision_ = revision;
    return changed;
}

System::System(const std::string& strVocFile,
               const std::string& strConfigLoaderFile, const eSensor sensor,
               const bool bUseViewer, const int initFr,
               const std::string& strSequence)
    : mSensor(sensor),
      mpViewer(static_cast<Viewer*>(nullptr)),
      mbReset(false),
      mbResetActiveMap(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false),
      mbLocalizationModeActive(false),
      mbShutDown(false) {
    cv::setNumThreads(kOpenCvThreadCount);
    AINFO << "Atlas OpenCV thread count: " << cv::getNumThreads() << std::endl;

    // Output welcome message
    AINFO << std::endl
          << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, "
             "Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University "
             "of Zaragoza."
          << std::endl
          << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. "
             "Montiel and Juan D. Tardós, University of Zaragoza."
          << std::endl
          << "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl
          << "This is free software, and you are welcome to redistribute it"
          << std::endl
          << "under certain conditions. See LICENSE.txt." << std::endl
          << std::endl;

    AINFO << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
        AINFO << "Monocular" << std::endl;
    else if (mSensor == STEREO)
        AINFO << "Stereo" << std::endl;
    else if (mSensor == RGBD)
        AINFO << "RGB-D" << std::endl;
    else if (mSensor == IMU_MONOCULAR)
        AINFO << "Monocular-Inertial" << std::endl;
    else if (mSensor == IMU_STEREO)
        AINFO << "Stereo-Inertial" << std::endl;
    else if (mSensor == IMU_RGBD)
        AINFO << "RGB-D-Inertial" << std::endl;

    // Check settings file
    cv::FileStorage fsConfigLoader(strConfigLoaderFile.c_str(),
                                   cv::FileStorage::READ);
    if (!fsConfigLoader.isOpened()) {
        AERROR << "Failed to open settings file at: " << strConfigLoaderFile
               << std::endl;
        exit(-1);
    }

    cv::FileNode node = fsConfigLoader["File.version"];
    if (!node.empty() && node.isString() && node.string() == "1.0") {
        settings_ = new ConfigLoader(strConfigLoaderFile, mSensor);

        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile = settings_->atlasSaveFile();

        AINFO << (*settings_) << std::endl;
    } else {
        settings_ = nullptr;
        cv::FileNode node = fsConfigLoader["System.LoadAtlasFromFile"];
        if (!node.empty() && node.isString()) {
            mStrLoadAtlasFromFile = static_cast<std::string>(node);
        }

        node = fsConfigLoader["System.SaveAtlasToFile"];
        if (!node.empty() && node.isString()) {
            mStrSaveAtlasToFile = static_cast<std::string>(node);
        }
    }

    node = fsConfigLoader["loopClosing"];
    bool activeLC = true;
    if (!node.empty()) {
        activeLC = static_cast<int>(fsConfigLoader["loopClosing"]) != 0;
    }

    mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if (mStrLoadAtlasFromFile.empty()) {
        // Load ORB Vocabulary
        AINFO << std::endl
              << "Loading ORB Vocabulary. This could take a while..."
              << std::endl;

        mpVocabulary = new Vocabulary();
        bool bVocLoad = mpVocabulary->LoadFromFile(strVocFile);
        if (!bVocLoad) {
            AERROR << "Wrong path to vocabulary. " << std::endl;
            AERROR << "Falied to open at: " << strVocFile << std::endl;
            throw std::runtime_error("failed to load FBoW vocabulary: " +
                                     strVocFile);
        }
        AINFO << "Vocabulary loaded!" << std::endl << std::endl;

        // Create KeyFrame Database
        mpPlaceRecognizer = new PlaceRecognizer(*mpVocabulary);

        // Create the Atlas
        AINFO << "Initialization of Atlas from scratch " << std::endl;
        mpAtlas = new Atlas(0);
    } else {
        // Load ORB Vocabulary
        AINFO << std::endl
              << "Loading ORB Vocabulary. This could take a while..."
              << std::endl;

        mpVocabulary = new Vocabulary();
        bool bVocLoad = mpVocabulary->LoadFromFile(strVocFile);
        if (!bVocLoad) {
            AERROR << "Wrong path to vocabulary. " << std::endl;
            AERROR << "Falied to open at: " << strVocFile << std::endl;
            throw std::runtime_error("failed to load FBoW vocabulary: " +
                                     strVocFile);
        }
        AINFO << "Vocabulary loaded!" << std::endl << std::endl;

        // Create KeyFrame Database
        mpPlaceRecognizer = new PlaceRecognizer(*mpVocabulary);

        AINFO << "Load File" << std::endl;

        // Load the file with an earlier session
        // clock_t start = clock();
        AINFO << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile
              << std::endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if (!isRead) {
            AINFO << "Error to load the file, please try with other session "
                     "file or vocabulary file"
                  << std::endl;
            exit(-1);
        }
        // mpPlaceRecognizer = new PlaceRecognizer(*mpVocabulary);

        // AINFO << "KF in DB: " << mpPlaceRecognizer->mnNumKFs << "; words: "
        // << mpPlaceRecognizer->mnNumWords << endl;

        loadedAtlas = true;

        mpAtlas->CreateNewMap();

        // clock_t timeElapsed = clock() - start;
        // unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        // AINFO << "Binary file read in " << msElapsed << " ms" << endl;

        // usleep(10*1000*1000);
    }

    if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR ||
        mSensor == IMU_RGBD)
        mpAtlas->SetInertialSensor();

    // Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strConfigLoaderFile, settings_);

    // Initialize the Tracker thread
    //(it will live in the main thread of execution, the one that called this
    // constructor)
    AINFO << "Seq. Name: " << strSequence << std::endl;
    mpTracker = new Tracker(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                            mpAtlas, mpPlaceRecognizer, strConfigLoaderFile,
                            mSensor, settings_, strSequence);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapper(
        this, mpAtlas, mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,
        mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD,
        strSequence);
    mpLocalMapper->mInitFr = initFr;
    if (settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsConfigLoader["thFarPoints"];
    if (mpLocalMapper->mThFarPoints != 0) {
        AINFO << "Discard points further than " << mpLocalMapper->mThFarPoints
              << " m from current camera" << std::endl;
        mpLocalMapper->mbFarPoints = true;
    } else
        mpLocalMapper->mbFarPoints = false;

    // Initialize the Loop Closing thread and launch
    //  mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser =
        new LoopCloser(mpAtlas, mpPlaceRecognizer, mpVocabulary,
                       mSensor != MONOCULAR, activeLC);  // mSensor!=MONOCULAR);

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopCloser(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // usleep(10*1000*1000);

    // Initialize the Viewer thread and launch
    if (bUseViewer)
    // if(false) // TODO
    {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                              strConfigLoaderFile, settings_);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    const std::size_t worker_count = mpViewer ? 3U : 2U;
    mWorkerPool = std::make_unique<ThreadPool>(worker_count);
    mWorkerPool->Submit(&LocalMapper::Run, mpLocalMapper);
    mWorkerPool->Submit(&LoopCloser::Run, mpLoopCloser);
    if (mpViewer)
        mWorkerPool->Submit(&Viewer::Run, mpViewer);
}

System::~System() {
    Shutdown();

    delete mpViewer;
    delete mpLoopCloser;
    delete mpLocalMapper;
    delete mpTracker;
    delete mpMapDrawer;
    delete mpFrameDrawer;
    delete mpAtlas;
    delete mpPlaceRecognizer;
    delete mpVocabulary;
    delete settings_;

    mpViewer = nullptr;
    mpLoopCloser = nullptr;
    mpLocalMapper = nullptr;
    mpTracker = nullptr;
    mpMapDrawer = nullptr;
    mpFrameDrawer = nullptr;
    mpAtlas = nullptr;
    mpPlaceRecognizer = nullptr;
    mpVocabulary = nullptr;
    settings_ = nullptr;
}

void System::WaitForLocalMappingResume() {
    if (!mpLocalMapper || mbLocalizationModeActive) {
        return;
    }

    const auto local_mapping_is_paused = [this]() {
        return mpLocalMapper->stopRequested() || mpLocalMapper->isStopped();
    };
    if (!local_mapping_is_paused())
        return;

    AINFO << "Waiting for loop correction to release local mapping"
          << std::endl;
    const auto deadline =
        std::chrono::steady_clock::now() + kLoopCorrectionWaitTimeout;
    while (local_mapping_is_paused() && !mpLocalMapper->isFinished() &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (local_mapping_is_paused()) {
        AWARN << "Timed out waiting for loop correction; continuing tracking"
              << std::endl;
    } else {
        AINFO << "Local mapping released; resuming tracking" << std::endl;
    }
}

autonomy::common::math::SE3f System::TrackStereo(
    const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp,
    const std::vector<IMU::Point>& vImuMeas, std::string filename) {
    if (mSensor != STEREO && mSensor != IMU_STEREO) {
        AERROR << "ERROR: you called TrackStereo but input sensor was not set "
                  "to Stereo nor Stereo-Inertial."
               << std::endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if (settings_ && settings_->needToRectify()) {
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    } else if (settings_ && settings_->needToResize()) {
        cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
        cv::resize(imRight, imRightToFeed, settings_->newImSize());
    } else {
        imLeftToFeed = imLeft;
        imRightToFeed = imRight;
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbLocalizationModeActive = true;
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mbLocalizationModeActive = false;
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        } else if (mbResetActiveMap) {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    WaitForLocalMappingResume();

    if (mSensor == System::IMU_STEREO)
        for (std::size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // AINFO << "start GrabImageStereo" << std::endl;
    autonomy::common::math::SE3f Tcw = mpTracker->GrabImageStereo(
        imLeftToFeed, imRightToFeed, timestamp, filename);

    // AINFO << "out grabber" << std::endl;

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

autonomy::common::math::SE3f System::TrackRGBD(
    const cv::Mat& im, const cv::Mat& depthmap, const double& timestamp,
    const std::vector<IMU::Point>& vImuMeas, std::string filename) {
    if (mSensor != RGBD && mSensor != IMU_RGBD) {
        AERROR << "ERROR: you called TrackRGBD but input sensor was not set to "
                  "RGBD."
               << std::endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if (settings_ && settings_->needToResize()) {
        cv::Mat resizedIm;
        cv::resize(im, resizedIm, settings_->newImSize());
        imToFeed = resizedIm;

        cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbLocalizationModeActive = true;
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mbLocalizationModeActive = false;
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        } else if (mbResetActiveMap) {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    WaitForLocalMappingResume();

    if (mSensor == System::IMU_RGBD)
        for (std::size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    autonomy::common::math::SE3f Tcw =
        mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

autonomy::common::math::SE3f System::TrackMonocular(
    const cv::Mat& im, const double& timestamp,
    const std::vector<IMU::Point>& vImuMeas, std::string filename) {
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbShutDown)
            return autonomy::common::math::SE3f();
    }

    if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR) {
        AERROR << "ERROR: you called TrackMonocular but input sensor was not "
                  "set to Monocular nor Monocular-Inertial."
               << std::endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if (settings_ && settings_->needToResize()) {
        cv::Mat resizedIm;
        cv::resize(im, resizedIm, settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbLocalizationModeActive = true;
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mbLocalizationModeActive = false;
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        } else if (mbResetActiveMap) {
            AINFO << "SYSTEM-> Reseting active map in monocular case"
                  << std::endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    WaitForLocalMappingResume();

    if (mSensor == System::IMU_MONOCULAR)
        for (std::size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    autonomy::common::math::SE3f Tcw =
        mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode() {
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
    if (mpAtlas == nullptr)
        return false;
    Map* current_map = mpAtlas->GetCurrentMap();
    if (current_map == nullptr)
        return false;
    return map_change_observer_.Observe(
        static_cast<std::uint64_t>(current_map->GetId()),
        current_map->GetLastBigChangeIdx());
}

bool System::GetCurrentMapId(std::uint64_t* map_id) {
    if (map_id == nullptr || mpAtlas == nullptr)
        return false;
    Map* current_map = mpAtlas->GetCurrentMap();
    if (current_map == nullptr)
        return false;
    *map_id = static_cast<std::uint64_t>(current_map->GetId());
    return true;
}

void System::Reset() {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap() {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown() {
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbShutDown)
            return;
        mbShutDown = true;
    }

    AINFO << "Shutdown" << std::endl;

    const auto drain_deadline =
        std::chrono::steady_clock::now() + kShutdownDrainTimeout;
    auto idle_since = std::chrono::steady_clock::time_point();
    bool workers_drained = false;
    while (std::chrono::steady_clock::now() < drain_deadline) {
        const bool local_mapper_idle =
            !mpLocalMapper || mpLocalMapper->IsIdle();
        const bool loop_closer_idle = !mpLoopCloser || mpLoopCloser->IsIdle();
        const auto now = std::chrono::steady_clock::now();
        if (local_mapper_idle && loop_closer_idle) {
            if (idle_since == std::chrono::steady_clock::time_point())
                idle_since = now;
            if (now - idle_since >= kShutdownStablePeriod) {
                workers_drained = true;
                break;
            }
        } else {
            idle_since = std::chrono::steady_clock::time_point();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (workers_drained) {
        AINFO << "Atlas mapping and loop-closing workers drained" << std::endl;
    } else {
        AWARN << "Timed out waiting for Atlas workers; stopping pending work"
              << std::endl;
    }

    if (mpLocalMapper)
        mpLocalMapper->RequestFinish();
    if (mpLoopCloser)
        mpLoopCloser->RequestFinish();
    if (mpViewer)
        mpViewer->RequestFinish();

    // Closing the pool drains submitted workers and joins every worker thread.
    if (mWorkerPool) {
        mWorkerPool->Stop();
        mWorkerPool.reset();
    }
    if (mpLoopCloser)
        mpLoopCloser->WaitForGlobalBundleAdjustment();

    if (!mStrSaveAtlasToFile.empty()) {
        AINFO << "Atlas saving to file " + mStrSaveAtlasToFile;
        SaveAtlas(FileType::BINARY_FILE);
    }

    /*if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif
}

bool System::isShutDown() {
    std::unique_lock<std::mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const std::string& filename) {
    AINFO << std::endl
          << "Saving camera trajectory to " << filename << " ..." << std::endl;
    if (mSensor == MONOCULAR) {
        AERROR << "ERROR: SaveTrajectoryTUM cannot be used for monocular."
               << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    if (vpKFs.empty()) {
        AWARN << "Cannot save an empty Atlas trajectory." << std::endl;
        return;
    }
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    autonomy::common::math::SE3f Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<::autonomy::localization::atlas::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for (std::list<autonomy::common::math::SE3f>::iterator
             lit = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++) {
        if (*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        autonomy::common::math::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        while (pKF->isBad()) {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        autonomy::common::math::SE3f Tcw = (*lit) * Trw;
        autonomy::common::math::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        f << std::setprecision(6) << *lT << " " << std::setprecision(9)
          << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " "
          << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    f.close();
    // AINFO << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const std::string& filename) {
    AINFO << std::endl
          << "Saving keyframe trajectory to " << filename << " ..."
          << std::endl;

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (pKF->isBad())
            continue;

        autonomy::common::math::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7)
          << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " "
          << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const std::string& filename) {
    AINFO << std::endl
          << "Saving trajectory to " << filename << " ..." << std::endl;
    /*if(mSensor==MONOCULAR)
    {
        AERROR << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
    endl; return;
    }*/

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    int numMaxKFs = 0;
    Map* pBiggerMap = nullptr;
    AINFO << "There are " << std::to_string(vpMaps.size())
          << " maps in the atlas" << std::endl;
    for (Map* pMap : vpMaps) {
        AINFO << "  Map " << std::to_string(pMap->GetId()) << " has "
              << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs"
              << std::endl;
        if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if (pBiggerMap == nullptr) {
        AWARN << "Cannot save an empty Atlas trajectory." << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    autonomy::common::math::SE3f
        Twb;  // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    // AINFO << "file open" << endl;
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<::autonomy::localization::atlas::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    // AINFO << "size mlpReferences: " << mpTracker->mlpReferences.size() <<
    // endl; AINFO << "size mlRelativeFramePoses: " <<
    // mpTracker->mlRelativeFramePoses.size() << endl; AINFO << "size
    // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    // AINFO << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() <<
    // endl;

    for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++) {
        // AINFO << "1" << endl;
        if (*lbL)
            continue;

        KeyFrame* pKF = *lRit;
        // AINFO << "KF: " << pKF->mnId << endl;

        autonomy::common::math::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        if (!pKF)
            continue;

        // AINFO << "2.5" << endl;

        while (pKF->isBad()) {
            // AINFO << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            // AINFO << "--Parent KF: " << pKF->mnId << endl;
        }

        if (!pKF || pKF->GetMap() != pBiggerMap) {
            // AINFO << "--Parent KF is from another map" << endl;
            continue;
        }

        // AINFO << "3" << endl;

        Trw = Trw * pKF->GetPose() *
              Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // AINFO << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD) {
            autonomy::common::math::SE3f Twb =
                (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        } else {
            autonomy::common::math::SE3f Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twc(0) << " " << twc(1) << " "
              << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }

        // AINFO << "5" << endl;
    }
    // AINFO << "end saving trajectory" << endl;
    f.close();
    AINFO << std::endl
          << "End of saving trajectory to " << filename << " ..." << std::endl;
}

void System::SaveTrajectoryEuRoC(const std::string& filename, Map* pMap) {
    AINFO << std::endl
          << "Saving trajectory of map " << pMap->GetId() << " to " << filename
          << " ..." << std::endl;
    /*if(mSensor==MONOCULAR)
    {
        AERROR << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
    endl; return;
    }*/

    int numMaxKFs = 0;

    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    autonomy::common::math::SE3f
        Twb;  // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    // AINFO << "file open" << endl;
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<::autonomy::localization::atlas::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    // AINFO << "size mlpReferences: " << mpTracker->mlpReferences.size() <<
    // endl; AINFO << "size mlRelativeFramePoses: " <<
    // mpTracker->mlRelativeFramePoses.size() << endl; AINFO << "size
    // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    // AINFO << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() <<
    // endl;

    for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++) {
        // AINFO << "1" << endl;
        if (*lbL)
            continue;

        KeyFrame* pKF = *lRit;
        // AINFO << "KF: " << pKF->mnId << endl;

        autonomy::common::math::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        if (!pKF)
            continue;

        // AINFO << "2.5" << endl;

        while (pKF->isBad()) {
            // AINFO << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            // AINFO << "--Parent KF: " << pKF->mnId << endl;
        }

        if (!pKF || pKF->GetMap() != pMap) {
            // AINFO << "--Parent KF is from another map" << endl;
            continue;
        }

        // AINFO << "3" << endl;

        Trw = Trw * pKF->GetPose() *
              Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // AINFO << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD) {
            autonomy::common::math::SE3f Twb =
                (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        } else {
            autonomy::common::math::SE3f Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << twc(0) << " " << twc(1) << " "
              << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;
        }

        // AINFO << "5" << endl;
    }
    // AINFO << "end saving trajectory" << endl;
    f.close();
    AINFO << std::endl
          << "End of saving trajectory to " << filename << " ..." << std::endl;
}

/*void System::SaveTrajectoryEuRoC(const string &filename)
{

    AINFO << endl << "Saving trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        AERROR << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
endl; return;
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    autonomy::common::math::SE3f Twb; // Can be word to cam0 or world to b
dependingo on IMU or not. if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO ||
mSensor==IMU_RGBD) Twb = vpKFs[0]->GetImuPose_(); else Twb =
vpKFs[0]->GetPoseInverse_();

    ofstream f;
    f.open(filename.c_str());
    // AINFO << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    list<::autonomy::localization::atlas::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); list<bool>::iterator lbL =
mpTracker->mlbLost.begin();

    //AINFO << "size mlpReferences: " << mpTracker->mlpReferences.size() <<
endl;
    //AINFO << "size mlRelativeFramePoses: " <<
mpTracker->mlRelativeFramePoses.size() << endl;
    //AINFO << "size mpTracker->mlFrameTimes: " <<
mpTracker->mlFrameTimes.size() << endl;
    //AINFO << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(list<autonomy::common::math::SE3f>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++,
lT++, lbL++)
    {
        //AINFO << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //AINFO << "KF: " << pKF->mnId << endl;

        autonomy::common::math::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
get a suitable keyframe. if (!pKF) continue;

        //AINFO << "2.5" << endl;

        while(pKF->isBad())
        {
            //AINFO << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //AINFO << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //AINFO << "--Parent KF is from another map" << endl;
            continue;
        }

        //AINFO << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new
world reference

        // AINFO << "4" << endl;


        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
mSensor==IMU_RGBD)
        {
            autonomy::common::math::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) *
Trw; autonomy::common::math::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) <<
twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
<< q.z() << " " << q.w() << endl;
        }
        else
        {
            autonomy::common::math::SE3f Tcw = (*lit) * Trw;
            autonomy::common::math::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) <<
twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
<< q.z() << " " << q.w() << endl;
        }

        // AINFO << "5" << endl;
    }
    //AINFO << "end saving trajectory" << endl;
    f.close();
    AINFO << endl << "End of saving trajectory to " << filename << " ..." <<
endl;
}*/

/*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
{
    AINFO << endl << "Saving keyframe trajectory to " << filename << " ..." <<
endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
mSensor==IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<
setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " <<
twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<
setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " <<
t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename) {
    AINFO << std::endl
          << "Saving keyframe trajectory to " << filename << " ..."
          << std::endl;

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for (Map* pMap : vpMaps) {
        if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs) {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if (!pBiggerMap) {
        AINFO << "There is not a map!!" << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD) {
            autonomy::common::math::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;

        } else {
            autonomy::common::math::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename,
                                         Map* pMap) {
    AINFO << std::endl
          << "Saving keyframe trajectory of map " << pMap->GetId() << " to "
          << filename << " ..." << std::endl;

    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKF = vpKFs[i];

        if (!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD) {
            autonomy::common::math::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << twb(0) << " " << twb(1) << " "
              << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << std::endl;

        } else {
            autonomy::common::math::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << std::setprecision(9) << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;
        }
    }
    f.close();
}

/*void System::SaveTrajectoryKITTI(const string &filename)
{
    AINFO << endl << "Saving camera trajectory to " << filename << " ..." <<
endl; if(mSensor==MONOCULAR)
    {
        AERROR << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." <<
endl; return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    list<::autonomy::localization::atlas::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); for(list<cv::Mat>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ::autonomy::localization::atlas::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)
<< " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " <<
Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " << Rwc.at<float>(2,0) << "
" << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  <<
twc.at<float>(2) << endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const std::string& filename) {
    AINFO << std::endl
          << "Saving camera trajectory to " << filename << " ..." << std::endl;
    if (mSensor == MONOCULAR) {
        AERROR << "ERROR: SaveTrajectoryKITTI cannot be used for monocular."
               << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    autonomy::common::math::SE3f Tow = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    std::list<::autonomy::localization::atlas::KeyFrame*>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (std::list<autonomy::common::math::SE3f>::iterator
             lit = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++) {
        ::autonomy::localization::atlas::KeyFrame* pKF = *lRit;

        autonomy::common::math::SE3f Trw;

        if (!pKF)
            continue;

        while (pKF->isBad()) {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

        autonomy::common::math::SE3f Tcw = (*lit) * Trw;
        autonomy::common::math::SE3f Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << std::setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " "
          << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1)
          << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " "
          << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << std::endl;
    }
    f.close();
}

void System::SaveDebugData(const int& initIdx) {
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" +
                        std::to_string(mpLocalMapper->mInitSect) + "_" +
                        std::to_string(initIdx) + ".txt");

    // 1. Save scale
    std::ofstream f;
    f.open("init_Scale_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mScale << std::endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << ","
      << mpLocalMapper->mRwg(0, 2) << std::endl;
    f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << ","
      << mpLocalMapper->mRwg(1, 2) << std::endl;
    f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << ","
      << mpLocalMapper->mRwg(2, 2) << std::endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mCostTime << std::endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << ","
      << mpLocalMapper->mbg(2) << std::endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << ","
      << mpLocalMapper->mba(2) << std::endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" + std::to_string(mpLocalMapper->mInitSect) + "_" +
               std::to_string(initIdx) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++) {
        for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++) {
            if (j != 0)
                f << ",";
            f << std::setprecision(15) << mpLocalMapper->mcovInertial(i, j);
        }
        f << std::endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << mpLocalMapper->mInitTime << std::endl;
    f.close();
}

int System::GetTrackingState() {
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackingState;
}

std::vector<MapPoint*> System::GetTrackedMapPoints() {
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

cv::Mat System::DrawCurrentFrame() {
    if (mpFrameDrawer == nullptr)
        return cv::Mat();
    return mpFrameDrawer->DrawFrame();
}

cv::Mat System::DrawFeatureMatches() {
    if (mpFrameDrawer == nullptr)
        return cv::Mat();
    return mpFrameDrawer->DrawFeatureMatches();
}

std::vector<TrajectoryPose> System::GetOptimizedKeyFrameTrajectory() {
    std::vector<TrajectoryPose> trajectory;
    if (mpAtlas == nullptr)
        return trajectory;

    Map* map = mpAtlas->GetCurrentMap();
    if (map == nullptr)
        return trajectory;
    std::unique_lock<std::mutex> map_lock(map->mMutexMapUpdate);
    std::vector<KeyFrame*> keyframes = map->GetAllKeyFrames();
    std::sort(keyframes.begin(), keyframes.end(),
              [](const KeyFrame* lhs, const KeyFrame* rhs) {
                  if (lhs == nullptr)
                      return false;
                  if (rhs == nullptr)
                      return true;
                  if (lhs->mTimeStamp != rhs->mTimeStamp)
                      return lhs->mTimeStamp < rhs->mTimeStamp;
                  return lhs->mnId < rhs->mnId;
              });
    trajectory.reserve(keyframes.size());
    for (KeyFrame* keyframe : keyframes) {
        if (keyframe == nullptr || keyframe->isBad() ||
            !std::isfinite(keyframe->mTimeStamp)) {
            continue;
        }
        const autonomy::common::math::SE3f world_from_camera =
            keyframe->GetPoseInverse();
        if (!world_from_camera.matrix().allFinite())
            continue;
        Eigen::Quaternionf rotation(world_from_camera.rotationMatrix());
        if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6f)
            continue;
        rotation.normalize();

        TrajectoryPose sample;
        sample.time = keyframe->mTimeStamp;
        sample.pose_world_camera.translation =
            world_from_camera.translation().cast<double>();
        sample.pose_world_camera.rotation = rotation.cast<double>();
        trajectory.push_back(std::move(sample));
    }
    return trajectory;
}

std::vector<double> System::GetPendingMappingKeyFrameTimestamps() {
    if (mpLocalMapper == nullptr)
        return {};
    return mpLocalMapper->GetPendingKeyFrameTimestamps();
}

std::vector<LoopClosureEdge> System::GetLoopClosureEdges() {
    std::vector<LoopClosureEdge> edges;
    if (mpAtlas == nullptr)
        return edges;

    Map* map = mpAtlas->GetCurrentMap();
    if (map == nullptr)
        return edges;
    std::unique_lock<std::mutex> map_lock(map->mMutexMapUpdate);
    const std::vector<KeyFrame*> keyframes = map->GetAllKeyFrames();
    std::set<std::pair<unsigned long, unsigned long>> seen;
    for (KeyFrame* keyframe : keyframes) {
        if (keyframe == nullptr || keyframe->isBad())
            continue;
        for (KeyFrame* connected : keyframe->GetLoopEdges()) {
            if (connected == nullptr || connected == keyframe ||
                connected->isBad()) {
                continue;
            }
            const unsigned long first_id =
                std::min(keyframe->mnId, connected->mnId);
            const unsigned long second_id =
                std::max(keyframe->mnId, connected->mnId);
            if (!seen.emplace(first_id, second_id).second)
                continue;
            const Eigen::Vector3f first = keyframe->GetCameraCenter();
            const Eigen::Vector3f second = connected->GetCameraCenter();
            if (!first.allFinite() || !second.allFinite())
                continue;
            LoopClosureEdge edge;
            edge.first = first.cast<double>();
            edge.second = second.cast<double>();
            edges.push_back(std::move(edge));
        }
    }
    return edges;
}

double System::GetTimeFromIMUInit() {
    double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    if ((aux > 0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost() {
    if (!mpAtlas->isImuInitialized())
        return false;
    else {
        if ((mpTracker->mState ==
             Tracker::LOST))  //||(mpTracker->mState==Tracker::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}

bool System::isFinished() {
    return (GetTimeFromIMUInit() > 0.1);
}

void System::ChangeDataset() {
    if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12) {
        mpTracker->ResetActiveMap();
    } else {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale() {
    return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time) {
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time) {
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time) {
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

void System::SaveAtlas(int type) {
    if (!mStrSaveAtlasToFile.empty()) {
        // clock_t start = clock();

        // Save the current session
        mpAtlas->PreSave();

        std::string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
        pathSaveFileName = pathSaveFileName.append(".osa");

        std::string strVocabularyChecksum =
            CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
        std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
        std::string strVocabularyName =
            mStrVocabularyFilePath.substr(found + 1);

        if (type == TEXT_FILE)  // File text
        {
            AINFO << "Starting to write the save text file " << std::endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            BinaryOutputArchive oa(ofs);

            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            AINFO << "End to write the save text file" << std::endl;
        } else if (type == BINARY_FILE)  // File binary
        {
            AINFO << "Starting to write the save binary file" << std::endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            BinaryOutputArchive oa(ofs);
            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            AINFO << "End to write save binary file" << std::endl;
        }
    }
}

bool System::LoadAtlas(int type) {
    std::string strFileVoc, strVocChecksum;
    bool isRead = false;

    std::string pathLoadFileName = "./";
    pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
    pathLoadFileName = pathLoadFileName.append(".osa");

    if (type == TEXT_FILE)  // File text
    {
        AINFO << "Starting to read the save text file " << std::endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good()) {
            AINFO << "Load file not found" << std::endl;
            return false;
        }
        BinaryInputArchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        AINFO << "End to load the save text file " << std::endl;
        isRead = true;
    } else if (type == BINARY_FILE)  // File binary
    {
        AINFO << "Starting to read the save binary file" << std::endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good()) {
            AINFO << "Load file not found" << std::endl;
            return false;
        }
        BinaryInputArchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        AINFO << "End to load the save binary file" << std::endl;
        isRead = true;
    }

    if (isRead) {
        // Check if the vocabulary is the same
        std::string strInputVocabularyChecksum =
            CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

        if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
            AINFO << "The vocabulary load isn't the same which the load "
                     "session was created "
                  << std::endl;
            AINFO << "-Vocabulary name: " << strFileVoc << std::endl;
            return false;  // Both are differents
        }

        mpAtlas->SetPlaceRecognizer(mpPlaceRecognizer);
        mpAtlas->SetVocabulary(mpVocabulary);
        mpAtlas->PostLoad();

        return true;
    }
    return false;
}

std::string System::CalculateCheckSum(std::string filename, int type) {
    std::string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

    std::ios_base::openmode flags = std::ios::in;
    if (type == BINARY_FILE)  // Binary file
        flags = std::ios::in | std::ios::binary;

    std::ifstream f(filename.c_str(), flags);
    if (!f.is_open()) {
        AINFO << "[E] Unable to open the in file " << filename
              << " for Md5 hash." << std::endl;
        return checksum;
    }

    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init(&md5Context);
    while (int count = f.readsome(buffer, sizeof(buffer))) {
        MD5_Update(&md5Context, buffer, count);
    }

    f.close();

    MD5_Final(c, &md5Context);

    for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
        char aux[10];
        sprintf(aux, "%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

}  // namespace autonomy::localization::atlas
