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

#include "atlas/frame_drawer.hpp"
#include "atlas/tracking/tracker.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <mutex>
#include <string>

namespace autonomy::localization::atlas {

FrameDrawer::FrameDrawer(Atlas* pAtlas) : both(false), mpAtlas(pAtlas) {
    mState = Tracker::SYSTEM_NOT_READY;
    mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    mImRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

cv::Mat FrameDrawer::DrawFrame(float imageScale) {
    cv::Mat im;
    std::vector<cv::KeyPoint>
        vIniKeys;  // Initialization: KeyPoints in reference frame
    std::vector<int>
        vMatches;  // Initialization: correspondeces with reference keypoints
    std::vector<cv::KeyPoint> vCurrentKeys;  // KeyPoints in current frame
    std::vector<bool> vbVO, vbMap;  // Tracked MapPoints in current frame
    std::vector<std::pair<cv::Point2f, cv::Point2f> > vTracks;
    int state;  // Tracker state
    std::vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    std::vector<MapPoint*> vpLocalMap;
    std::vector<cv::KeyPoint> vMatchesKeys;
    std::vector<MapPoint*> vpMatchedMPs;
    std::vector<cv::KeyPoint> vOutlierKeys;
    std::vector<MapPoint*> vpOutlierMPs;
    std::map<long unsigned int, cv::Point2f> mProjectPoints;
    std::map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0, 255, 0);
    cv::Scalar odometryColor(255, 0, 0);
    cv::Scalar detectionColor(0, 215, 255);
    cv::Scalar trackColor(255, 255, 0);

    // Copy variables within scoped mutex
    {
        std::unique_lock<std::mutex> lock(mMutex);
        state = mState;
        if (mState == Tracker::SYSTEM_NOT_READY)
            mState = Tracker::NO_IMAGES_YET;

        mIm.copyTo(im);

        if (mState == Tracker::NOT_INITIALIZED) {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        } else if (mState == Tracker::OK) {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            vTracks = mvTracks;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;

        } else if (mState == Tracker::LOST) {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if (imageScale != 1.f) {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if (im.channels() < 3)  // this should be always true
        cvtColor(im, im, cv::COLOR_GRAY2BGR);

    // Detected ORB features are yellow. Matched points are overdrawn below.
    for (const cv::KeyPoint& keypoint : vCurrentKeys) {
        cv::Point2f point = keypoint.pt;
        if (imageScale != 1.f)
            point /= imageScale;
        cv::circle(im, point, 1, detectionColor, -1);
    }

    // Draw
    if (state == Tracker::NOT_INITIALIZED) {
        for (unsigned int i = 0; i < vMatches.size(); i++) {
            if (vMatches[i] >= 0) {
                cv::Point2f pt1, pt2;
                if (imageScale != 1.f) {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                } else {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im, pt1, pt2, standardColor);
            }
        }
        for (std::vector<std::pair<cv::Point2f, cv::Point2f> >::iterator it =
                 vTracks.begin();
             it != vTracks.end(); it++) {
            cv::Point2f pt1, pt2;
            if (imageScale != 1.f) {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            } else {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im, pt1, pt2, standardColor, 5);
        }

    } else if (state == Tracker::OK)  // TRACKING
    {
        mnTracked = 0;
        mnTrackedVO = 0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for (int i = 0; i < n; i++) {
            if (vbVO[i] || vbMap[i]) {
                cv::Point2f pt1, pt2;
                cv::Point2f point;
                if (imageScale != 1.f) {
                    point = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x = px - r;
                    pt1.y = py - r;
                    pt2.x = px + r;
                    pt2.y = py + r;
                } else {
                    point = vCurrentKeys[i].pt;
                    pt1.x = vCurrentKeys[i].pt.x - r;
                    pt1.y = vCurrentKeys[i].pt.y - r;
                    pt2.x = vCurrentKeys[i].pt.x + r;
                    pt2.y = vCurrentKeys[i].pt.y + r;
                }

                // This is a match to a MapPoint in the map
                if (vbMap[i]) {
                    cv::rectangle(im, pt1, pt2, standardColor);
                    cv::circle(im, point, 2, standardColor, -1);
                    mnTracked++;
                } else  // This is match to a "visual odometry" MapPoint created
                        // in the last frame
                {
                    cv::rectangle(im, pt1, pt2, odometryColor);
                    cv::circle(im, point, 2, odometryColor, -1);
                    mnTrackedVO++;
                }
            }
        }
        // Draw temporal matches last so green/blue point boxes cannot hide the
        // short cyan motion segments between adjacent KITTI frames.
        for (const auto& track : vTracks) {
            cv::Point2f previous = track.first;
            cv::Point2f current = track.second;
            if (imageScale != 1.f) {
                previous /= imageScale;
                current /= imageScale;
            }
            cv::line(im, previous, current, trackColor, 2, cv::LINE_AA);
            cv::circle(im, previous, 2, trackColor, -1, cv::LINE_AA);
            cv::circle(im, current, 2, trackColor, -1, cv::LINE_AA);
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im, state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawFeatureMatches(float imageScale) {
    cv::Mat previous;
    cv::Mat current;
    std::vector<std::pair<cv::Point2f, cv::Point2f> > tracks;
    {
        std::unique_lock<std::mutex> lock(mMutex);
        if (!mbHasPreviousFrame || !mbHasCurrentFrame)
            return cv::Mat();
        mPreviousIm.copyTo(previous);
        mIm.copyTo(current);
        tracks = mvTracks;
    }

    if (previous.empty() || current.empty() || imageScale <= 0.f)
        return cv::Mat();
    if (previous.channels() == 1)
        cv::cvtColor(previous, previous, cv::COLOR_GRAY2BGR);
    else if (previous.channels() == 4)
        cv::cvtColor(previous, previous, cv::COLOR_BGRA2BGR);
    if (current.channels() == 1)
        cv::cvtColor(current, current, cv::COLOR_GRAY2BGR);
    else if (current.channels() == 4)
        cv::cvtColor(current, current, cv::COLOR_BGRA2BGR);
    if (previous.type() != CV_8UC3 || current.type() != CV_8UC3 ||
        previous.size() != current.size()) {
        return cv::Mat();
    }

    if (imageScale != 1.f) {
        const cv::Size scaled(
            std::max(1, static_cast<int>(current.cols / imageScale)),
            std::max(1, static_cast<int>(current.rows / imageScale)));
        cv::resize(previous, previous, scaled);
        cv::resize(current, current, scaled);
    }

    cv::Mat matches;
    cv::hconcat(previous, current, matches);
    const cv::Scalar match_color(255, 255, 0);  // Cyan in BGR.
    for (const auto& track : tracks) {
        cv::Point2f from = track.first;
        cv::Point2f to = track.second;
        if (imageScale != 1.f) {
            from /= imageScale;
            to /= imageScale;
        }
        to.x += static_cast<float>(previous.cols);
        cv::line(matches, from, to, match_color, 1, cv::LINE_AA);
        cv::circle(matches, from, 3, match_color, -1, cv::LINE_AA);
        cv::circle(matches, to, 3, match_color, -1, cv::LINE_AA);
    }

    cv::putText(matches, "Previous", cv::Point(10, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2,
                cv::LINE_AA);
    cv::putText(matches, "Current",
                cv::Point(previous.cols + 10, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2,
                cv::LINE_AA);
    cv::putText(matches, "Matches: " + std::to_string(tracks.size()),
                cv::Point(10, matches.rows - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, match_color, 2,
                cv::LINE_AA);
    return matches;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale) {
    cv::Mat im;
    std::vector<cv::KeyPoint>
        vIniKeys;  // Initialization: KeyPoints in reference frame
    std::vector<int>
        vMatches;  // Initialization: correspondeces with reference keypoints
    std::vector<cv::KeyPoint> vCurrentKeys;  // KeyPoints in current frame
    std::vector<bool> vbVO, vbMap;  // Tracked MapPoints in current frame
    int state;                      // Tracker state

    // Copy variables within scoped mutex
    {
        std::unique_lock<std::mutex> lock(mMutex);
        state = mState;
        if (mState == Tracker::SYSTEM_NOT_READY)
            mState = Tracker::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if (mState == Tracker::NOT_INITIALIZED) {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        } else if (mState == Tracker::OK) {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
        } else if (mState == Tracker::LOST) {
            vCurrentKeys = mvCurrentKeysRight;
        }
    }  // destroy scoped mutex -> release mutex

    if (imageScale != 1.f) {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if (im.channels() < 3)  // this should be always true
        cvtColor(im, im, cv::COLOR_GRAY2BGR);

    // Draw
    if (state == Tracker::NOT_INITIALIZED)  // INITIALIZING
    {
        for (unsigned int i = 0; i < vMatches.size(); i++) {
            if (vMatches[i] >= 0) {
                cv::Point2f pt1, pt2;
                if (imageScale != 1.f) {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                } else {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }

                cv::line(im, pt1, pt2, cv::Scalar(0, 255, 0));
            }
        }
    } else if (state == Tracker::OK)  // TRACKING
    {
        mnTracked = 0;
        mnTrackedVO = 0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for (int i = 0; i < n; i++) {
            if (vbVO[i + Nleft] || vbMap[i + Nleft]) {
                cv::Point2f pt1, pt2;
                cv::Point2f point;
                if (imageScale != 1.f) {
                    point = mvCurrentKeysRight[i].pt / imageScale;
                    float px = mvCurrentKeysRight[i].pt.x / imageScale;
                    float py = mvCurrentKeysRight[i].pt.y / imageScale;
                    pt1.x = px - r;
                    pt1.y = py - r;
                    pt2.x = px + r;
                    pt2.y = py + r;
                } else {
                    point = mvCurrentKeysRight[i].pt;
                    pt1.x = mvCurrentKeysRight[i].pt.x - r;
                    pt1.y = mvCurrentKeysRight[i].pt.y - r;
                    pt2.x = mvCurrentKeysRight[i].pt.x + r;
                    pt2.y = mvCurrentKeysRight[i].pt.y + r;
                }

                // This is a match to a MapPoint in the map
                if (vbMap[i + Nleft]) {
                    cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
                    cv::circle(im, point, 2, cv::Scalar(0, 255, 0), -1);
                    mnTracked++;
                } else  // This is match to a "visual odometry" MapPoint created
                        // in the last frame
                {
                    cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
                    cv::circle(im, point, 2, cv::Scalar(255, 0, 0), -1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im, state, imWithInfo);

    return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat& im, int nState, cv::Mat& imText) {
    std::stringstream s;
    if (nState == Tracker::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if (nState == Tracker::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if (nState == Tracker::OK) {
        if (!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs
          << ", Matches: " << mnTracked;
        if (mnTrackedVO > 0)
            s << ", + VO matches: " << mnTrackedVO;
    } else if (nState == Tracker::LOST) {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    } else if (nState == Tracker::SYSTEM_NOT_READY) {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline = 0;
    cv::Size textSize =
        cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
    im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
    imText.rowRange(im.rows, imText.rows) =
        cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
    cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
                cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

void FrameDrawer::Update(Tracker* pTracker) {
    std::unique_lock<std::mutex> lock(mMutex);
    if (mbHasCurrentFrame) {
        mIm.copyTo(mPreviousIm);
        mbHasPreviousFrame = !mPreviousIm.empty();
    } else {
        mPreviousIm.release();
        mbHasPreviousFrame = false;
    }
    pTracker->mImGray.copyTo(mIm);
    mbHasCurrentFrame = !mIm.empty();
    mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
    mThDepth = pTracker->mCurrentFrame.mThDepth;
    mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

    if (both) {
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    } else {
        N = mvCurrentKeys.size();
    }

    mvbVO = std::vector<bool>(N, false);
    mvbMap = std::vector<bool>(N, false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    // Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    const auto previous_matches = mmMatchedInImage;
    mmMatchedInImage.clear();
    mvTracks.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);

    if (pTracker->mLastProcessedState == Tracker::NOT_INITIALIZED) {
        mvIniKeys = pTracker->mInitialFrame.mvKeys;
        mvIniMatches = pTracker->mvIniMatches;
    } else if (pTracker->mLastProcessedState == Tracker::OK) {
        const int map_point_count =
            static_cast<int>(pTracker->mCurrentFrame.mvpMapPoints.size());
        const int outlier_count =
            static_cast<int>(pTracker->mCurrentFrame.mvbOutlier.size());
        const int safe_count =
            std::min(N, std::min(map_point_count, outlier_count));
        const int left_count = static_cast<int>(mvCurrentKeys.size());
        for (int i = 0; i < safe_count; i++) {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if (pMP) {
                if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
                    if (pMP->Observations() > 0)
                        mvbMap[i] = true;
                    else
                        mvbVO[i] = true;

                    // The published tracking image is the left/primary image.
                    // Keep right-camera matches in the state flags without
                    // indexing the left keypoint vector.
                    if (i < left_count) {
                        const cv::Point2f current = mvCurrentKeys[i].pt;
                        mmMatchedInImage[pMP->mnId] = current;
                        const auto previous = previous_matches.find(pMP->mnId);
                        if (previous != previous_matches.end()) {
                            mvTracks.emplace_back(previous->second, current);
                        }
                    }
                } else {
                    mvpOutlierMPs.push_back(pMP);
                    if (i < left_count)
                        mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
            }
        }
    }
    mState = static_cast<int>(pTracker->mLastProcessedState);
}

}  // namespace autonomy::localization::atlas
