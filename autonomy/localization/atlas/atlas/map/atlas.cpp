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

#include "atlas/map/atlas.hpp"
#include "atlas/viewer.hpp"
#include "autolink/common/log.hpp"

#include "atlas/sensor/camera/geometric_camera.hpp"
#include "atlas/sensor/camera/kannala_brandt_camera.hpp"
#include "atlas/sensor/camera/pinhole_camera.hpp"

namespace autonomy::localization::atlas {

Atlas::Atlas() {
    mpCurrentMap = static_cast<Map*>(nullptr);
}

Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid), mHasViewer(false) {
    mpCurrentMap = static_cast<Map*>(nullptr);
    CreateNewMap();
}

Atlas::~Atlas() {
    for (std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end();
         it != end;) {
        Map* pMi = *it;

        if (pMi) {
            delete pMi;
            pMi = static_cast<Map*>(nullptr);

            it = mspMaps.erase(it);
        } else
            ++it;
    }
}

void Atlas::CreateNewMap() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    AINFO << "Creation of new map with id: " << Map::nNextId << std::endl;
    if (mpCurrentMap) {
        if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap =
                mpCurrentMap->GetMaxKFid() +
                1;  // The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        AINFO << "Stored map with ID: " << mpCurrentMap->GetId() << std::endl;

        // if(mHasViewer)
        //     mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    AINFO << "Creation of new map with last KF id: " << mnLastInitKFidMap
          << std::endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map* pMap) {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    AINFO << "Change to map with id: " << pMap->GetId() << std::endl;
    if (mpCurrentMap) {
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer) {
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF) {
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP) {
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam) {
    // Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam = -1;
    for (std::size_t i = 0; i < mvpCameras.size(); ++i) {
        GeometricCamera* pCam_i = mvpCameras[i];
        if (!pCam)
            AINFO << "Not pCam" << std::endl;
        if (!pCam_i)
            AINFO << "Not pCam_i" << std::endl;
        if (pCam->GetType() != pCam_i->GetType())
            continue;

        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
            if ((static_cast<PinholeCamera*>(pCam_i))->IsEqual(pCam)) {
                bAlreadyInMap = true;
                index_cam = i;
            }
        } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
            if ((static_cast<KannalaBrandtCamera*>(pCam_i))->IsEqual(pCam)) {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if (bAlreadyInMap) {
        return mvpCameras[index_cam];
    } else {
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera*> Atlas::GetAllCameras() {
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs) {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

std::vector<Map*> Atlas::GetAllMaps() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    struct compFunctor {
        inline bool operator()(Map* elem1, Map* elem2) {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::vector<Map*> vMaps(mspMaps.begin(), mspMaps.end());
    std::sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end();
    it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(nullptr);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    if (!mpCurrentMap)
        CreateNewMap();
    while (mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap) {
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps() {
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(nullptr);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave() {
    if (mpCurrentMap) {
        if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap =
                mpCurrentMap->GetMaxKFid() +
                1;  // The init KF is the next of current maximum
    }

    struct compFunctor {
        inline bool operator()(Map* elem1, Map* elem2) {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(),
              std::back_inserter(mvpBackupMaps));
    std::sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    for (Map* pMi : mvpBackupMaps) {
        if (!pMi || pMi->IsBad())
            continue;

        if (pMi->GetAllKeyFrames().size() == 0) {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        pMi->PreSave(spCams);
    }
    RemoveBadMaps();
}

void Atlas::PostLoad() {
    std::map<unsigned int, GeometricCamera*> mpCams;
    for (GeometricCamera* pCam : mvpCameras) {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    for (Map* pMi : mvpBackupMaps) {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();
}

void Atlas::SetPlaceRecognizer(PlaceRecognizer* pKFDB) {
    mpKeyFrameDB = pKFDB;
}

PlaceRecognizer* Atlas::GetPlaceRecognizer() {
    return mpKeyFrameDB;
}

void Atlas::SetVocabulary(Vocabulary* pORBVoc) {
    mpVocabulary = pORBVoc;
}

Vocabulary* Atlas::GetVocabulary() {
    return mpVocabulary;
}

long unsigned int Atlas::GetNumLivedKF() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map* pMap_i : mspMaps) {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map* pMap_i : mspMaps) {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

std::map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes() {
    std::map<long unsigned int, KeyFrame*> mpIdKFs;
    for (Map* pMap_i : mvpBackupMaps) {
        std::vector<KeyFrame*> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for (KeyFrame* pKF_j_Mi : vpKFs_Mi) {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

}  // namespace autonomy::localization::atlas
