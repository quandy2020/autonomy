/**
 * This file is part of the ORB-SLAM3 port in Atlas.
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gomez
 * Rodriguez, Jose M.M. Montiel and Juan D. Tardos, University of Zaragoza.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 or later.
 */

#include "atlas/viewer.hpp"

namespace autonomy::localization::atlas {

Viewer::Viewer(System* system, FrameDrawer* frame_drawer, MapDrawer* map_drawer,
               Tracker* tracker, const std::string&, ConfigLoader*)
    : both(false),
      mpSystem(system),
      mpFrameDrawer(frame_drawer),
      mpMapDrawer(map_drawer),
      mpTracker(tracker),
      mT(0.0),
      mImageWidth(0.0f),
      mImageHeight(0.0f),
      mImageViewerScale(1.0f),
      mViewpointX(0.0f),
      mViewpointY(0.0f),
      mViewpointZ(0.0f),
      mViewpointF(0.0f),
      mbFinishRequested(false),
      mbFinished(false),
      mbStopped(false),
      mbStopRequested(false),
      mbStopTrack(false) {}

void Viewer::newParameterLoader(ConfigLoader*) {}

void Viewer::Run() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

void Viewer::RequestFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

void Viewer::RequestStop() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopRequested = true;
    mbStopped = true;
}

bool Viewer::isFinished() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

bool Viewer::isStopped() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::isStepByStep() {
    return false;
}

void Viewer::Release() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}

bool Viewer::ParseViewerParamFile(cv::FileStorage&) {
    return true;
}

bool Viewer::Stop() {
    std::unique_lock<std::mutex> lock(mMutexStop);
    if (!mbStopRequested)
        return false;
    mbStopped = true;
    return true;
}

bool Viewer::CheckFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

}  // namespace autonomy::localization::atlas
