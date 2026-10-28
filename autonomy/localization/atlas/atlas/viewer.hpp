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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_VIEWER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_VIEWER_HPP_

#include "atlas/common/io/config_loader.hpp"
#include "atlas/frame_drawer.hpp"
#include "atlas/map_drawer.hpp"
#include "atlas/system.hpp"
#include "atlas/tracking/tracker.hpp"

#include <mutex>

namespace autonomy::localization::atlas {

class Tracker;
class FrameDrawer;
class MapDrawer;
class System;
class ConfigLoader;

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer,
           Tracker* pTracker, const std::string& strSettingPath,
           ConfigLoader* settings);

    void newParameterLoader(ConfigLoader* settings);

    // Main thread function. Draw points, keyframes, the current camera pose and
    // the last processed frame. Drawing is refreshed according to the camera
    // fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    bool isStepByStep();

    void Release();

    // void SetTrackerPause();

    bool both;

private:
    bool ParseViewerParamFile(cv::FileStorage& fConfigLoader);

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracker* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageViewerScale;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool mbStopTrack;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_VIEWER_HPP_
