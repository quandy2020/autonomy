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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_LOCAL_MAPPER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_LOCAL_MAPPER_HPP_

#include "atlas/closure/loop_closer.hpp"
#include "atlas/closure/place_recognizer.hpp"
#include "atlas/common/io/config_loader.hpp"
#include "atlas/map/atlas.hpp"
#include "atlas/sensor/keyframe.hpp"
#include "atlas/tracking/tracker.hpp"

#include <fstream>
#include <mutex>

namespace autonomy::localization::atlas {

class System;
class Tracker;
class LoopCloser;
class Atlas;

class LocalMapper
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapper(System* pSys, Atlas* pAtlas, const float bMonocular,
                bool bInertial, const std::string& _strSeqName = std::string());

    void SetLoopCloser(LoopCloser* pLoopCloser);

    void SetTracker(Tracker* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue() {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;

    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    std::string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

#ifdef REGISTER_TIMES
    std::vector<double> vdKFInsert_ms;
    std::vector<double> vdMPCulling_ms;
    std::vector<double> vdMPCreation_ms;
    std::vector<double> vdLBA_ms;
    std::vector<double> vdKFCulling_ms;
    std::vector<double> vdLMTotal_ms;

    std::vector<double> vdLBASync_ms;
    std::vector<double> vdKFCullingSync_ms;
    std::vector<int> vnLBA_edges;
    std::vector<int> vnLBA_KFopt;
    std::vector<int> vnLBA_KFfixed;
    std::vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif
protected:
    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    System* mpSystem;

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    LoopCloser* mpLoopCloser;
    Tracker* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    void InitializeIMU(float priorG = 1e2, float priorA = 1e6,
                       bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    // DEBUG
    std::ofstream f_lm;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_LOCAL_MAPPER_HPP_
