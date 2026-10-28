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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_CLOSURE_PLACE_RECOGNIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_CLOSURE_PLACE_RECOGNIZER_HPP_

#include <list>
#include <set>
#include <vector>

#include "atlas/feature/vocabulary.hpp"
#include "atlas/map/map.hpp"
#include "atlas/sensor/frame.hpp"
#include "atlas/sensor/keyframe.hpp"

#include <mutex>

namespace autonomy::localization::atlas {

class KeyFrame;
class Frame;
class Map;

class PlaceRecognizer
{
    friend class ArchiveAccess;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& mvBackupInvertedFileId;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlaceRecognizer() {}
    PlaceRecognizer(const Vocabulary& voc);

    void add(KeyFrame* pKF);

    void erase(KeyFrame* pKF);

    void clear();
    void clearMap(Map* pMap);

    // Loop Detection(DEPRECATED)
    std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);

    // Loop and Merge Detection
    void DetectCandidates(KeyFrame* pKF, float minScore,
                          std::vector<KeyFrame*>& vpLoopCand,
                          std::vector<KeyFrame*>& vpMergeCand);
    void DetectBestCandidates(KeyFrame* pKF, std::vector<KeyFrame*>& vpLoopCand,
                              std::vector<KeyFrame*>& vpMergeCand,
                              int nMinWords);
    void DetectNBestCandidates(KeyFrame* pKF,
                               std::vector<KeyFrame*>& vpLoopCand,
                               std::vector<KeyFrame*>& vpMergeCand,
                               int nNumCandidates);

    // Relocalization
    std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);

    void PreSave();
    void PostLoad(std::map<long unsigned int, KeyFrame*> mpKFid);
    void SetVocabulary(Vocabulary* pORBVoc);

protected:
    // Associated vocabulary
    const Vocabulary* mpVoc;

    // Inverted file
    std::vector<std::list<KeyFrame*> > mvInvertedFile;

    // For save relation without pointer, this is necessary for save/load
    // function
    std::vector<std::list<long unsigned int> > mvBackupInvertedFileId;

    // Mutex
    std::mutex mMutex;
};

}  // namespace autonomy::localization::atlas

#endif
