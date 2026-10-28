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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAP_ATLAS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAP_ATLAS_HPP_

#include "atlas/map/map.hpp"
#include "atlas/sensor/camera/geometric_camera.hpp"
#include "atlas/sensor/camera/kannala_brandt_camera.hpp"
#include "atlas/sensor/camera/pinhole_camera.hpp"
#include "atlas/sensor/keyframe.hpp"
#include "atlas/sensor/map_point.hpp"

#include <mutex>
#include <set>

namespace autonomy::localization::atlas {
class Viewer;
class Map;
class MapPoint;
class KeyFrame;
class PlaceRecognizer;
class Frame;
class KannalaBrandtCamera;
class PinholeCamera;

class Atlas
{
    friend class ArchiveAccess;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar.template register_type<PinholeCamera>();
        ar.template register_type<KannalaBrandtCamera>();

        // Persist ordered backup vectors instead of runtime pointer sets.
        // ar & mspMaps;
        ar& mvpBackupMaps;
        ar& mvpCameras;
        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and
        // Map
        ar& Map::nNextId;
        ar& Frame::nNextId;
        ar& KeyFrame::nNextId;
        ar& MapPoint::nNextId;
        ar& GeometricCamera::nNextId;
        ar& mnLastInitKFidMap;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Atlas();
    Atlas(int initKFid);  // When its initialization the first map is created
    ~Atlas();

    void CreateNewMap();
    void ChangeMap(Map* pMap);

    unsigned long int GetLastInitKFid();

    void SetViewer(Viewer* pViewer);

    // Method for change components in the current map
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    // void EraseMapPoint(MapPoint* pMP);
    // void EraseKeyFrame(KeyFrame* pKF);

    GeometricCamera* AddCamera(GeometricCamera* pCam);
    std::vector<GeometricCamera*> GetAllCameras();

    /* All methods without Map pointer work on current map */
    void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    // Method for get data in current map
    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    std::vector<Map*> GetAllMaps();

    int CountMaps();

    void clearMap();

    void clearAtlas();

    Map* GetCurrentMap();

    void SetMapBad(Map* pMap);
    void RemoveBadMaps();

    bool isInertial();
    void SetInertialSensor();
    void SetImuInitialized();
    bool isImuInitialized();

    // Function for garantee the correction of serialization of this object
    void PreSave();
    void PostLoad();

    std::map<long unsigned int, KeyFrame*> GetAtlasKeyframes();

    void SetPlaceRecognizer(PlaceRecognizer* pKFDB);
    PlaceRecognizer* GetPlaceRecognizer();

    void SetVocabulary(Vocabulary* pORBVoc);
    Vocabulary* GetVocabulary();

    long unsigned int GetNumLivedKF();

    long unsigned int GetNumLivedMP();

protected:
    std::set<Map*> mspMaps;
    std::set<Map*> mspBadMaps;
    // Stable vector representation used by the binary archive.
    std::vector<Map*> mvpBackupMaps;

    Map* mpCurrentMap;

    std::vector<GeometricCamera*> mvpCameras;

    unsigned long int mnLastInitKFidMap;

    Viewer* mpViewer;
    bool mHasViewer;

    // Class references for the map reconstruction from the save file
    PlaceRecognizer* mpKeyFrameDB;
    Vocabulary* mpVocabulary;

    // Mutex
    std::mutex mMutexAtlas;

};  // class Atlas

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAP_ATLAS_HPP_
