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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAP_DRAWER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAP_DRAWER_HPP_

#include "atlas/common/io/config_loader.hpp"
#include "atlas/map/atlas.hpp"
#include "atlas/sensor/keyframe.hpp"
#include "atlas/sensor/map_point.hpp"

#include <mutex>

namespace autonomy::localization::atlas {

class ConfigLoader;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas, const std::string& strSettingPath,
              ConfigLoader* settings);

    void newParameterLoader(ConfigLoader* settings);

    Atlas* mpAtlas;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph,
                       const bool bDrawInertialGraph, const bool bDrawOptLba);
    void SetCurrentCameraPose(const Sophus::SE3f& Tcw);
    void SetReferenceKeyFrame(KeyFrame* pKF);

private:
    bool ParseViewerParamFile(cv::FileStorage& fConfigLoader);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f}, {0.8f, 0.4f, 1.0f},
                                 {1.0f, 0.2f, 0.4f}, {0.6f, 0.0f, 1.0f},
                                 {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f}};
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAP_DRAWER_HPP_
