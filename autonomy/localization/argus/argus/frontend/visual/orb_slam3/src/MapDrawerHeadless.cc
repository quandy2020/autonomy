/**
 * This file is part of the ORB-SLAM3 port in Atlas.
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gomez
 * Rodriguez, Jose M.M. Montiel and Juan D. Tardos, University of Zaragoza.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 or later.
 */

#include "MapDrawer.h"

namespace ORB_SLAM3 {

MapDrawer::MapDrawer(Atlas* atlas, const std::string&, Settings*)
    : mpAtlas(atlas),
      mKeyFrameSize(0.0f),
      mKeyFrameLineWidth(0.0f),
      mGraphLineWidth(0.0f),
      mPointSize(0.0f),
      mCameraSize(0.0f),
      mCameraLineWidth(0.0f),
      mCameraPose() {}

void MapDrawer::newParameterLoader(Settings*) {}
void MapDrawer::DrawMapPoints() {}
void MapDrawer::DrawKeyFrames(bool, bool, bool, bool) {}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f& camera_from_world) {
  std::unique_lock<std::mutex> lock(mMutexCamera);
  mCameraPose = camera_from_world;
}

void MapDrawer::SetReferenceKeyFrame(KeyFrame*) {}

}  // namespace ORB_SLAM3
