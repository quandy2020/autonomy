/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Adapted from ORB-SLAM3 ORBextractor (octree ORB distribution).
 */

/**
 * @file orb_extractor.hpp
 * @brief Octree-uniform ORB feature extractor (from ORB-SLAM3 ORBextractor).
 *
 * Builds an image pyramid, detects FAST corners, redistributes them with an
 * octree for spatial uniformity, then computes ORB descriptors.
 * The current implementation ignores the mask.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_EXTRACTOR_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_EXTRACTOR_HPP_

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>


namespace autonomy {
namespace localization {
namespace atlas {
namespace feature
{

/**
 * @class autonomy::localization::atlas::feature::ExtractorNode
 * @brief Octree node: keypoints pending distribution inside a rectangle.
 *
 * Used by DistributeOctTree to recursively quarter dense regions until each
 * leaf holds at most one strong-response point.
 */
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    /**
     * @brief Quarter this node into n1..n4 and assign vKeys by quadrant.
     * @param n1 Upper-left child.
     * @param n2 Upper-right child.
     * @param n3 Lower-left child.
     * @param n4 Lower-right child.
     */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;              ///< Keypoints inside this node
    cv::Point2i UL, UR, BL, BR;                   ///< Rectangle corners (pixels)
    std::list<ExtractorNode>::iterator lit;       ///< Iterator in the parent list
    bool bNoMore;                                 ///< true if this node cannot split further
};

/**
 * @class autonomy::localization::atlas::feature::OrbExtractor
 * @brief Multi-scale ORB extraction: pyramid + FAST + octree + descriptors.
 *
 * Call `operator()` on a single gray image; outputs keypoints and descriptors,
 * and may fill fisheye overlap info `vLappingArea`.
 *
 * @note Typically Tracker holds one instance per left/right camera; invoked on
 *       the Tracking thread during Frame construction.
 */
class OrbExtractor
{
public:
    
    /** @brief Response score mode: Harris or FAST response. */
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /**
     * @brief Construct the extractor.
     * @param nfeatures Target total feature count (shared across levels).
     * @param scaleFactor Pyramid scale ratio between levels (e.g. 1.2).
     * @param nlevels Number of pyramid levels.
     * @param iniThFAST Initial FAST threshold.
     * @param minThFAST Minimum FAST threshold when features are scarce.
     */
    OrbExtractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~OrbExtractor(){}

    /**
     * @brief Compute ORB features and descriptors on an image (octree uniform).
     * @param _image Input gray image.
     * @param _mask Mask (ignored by the current implementation).
     * @param[out] _keypoints Output keypoints (with octave).
     * @param[out] _descriptors Output descriptor matrix.
     * @param[in,out] vLappingArea Fisheye overlap columns [begin,end) for L/R packing.
     * @return Number of extracted features (impl convention).
     */
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    /** @brief Number of pyramid levels. */
    int inline GetLevels(){
        return nlevels;}

    /** @brief Inter-level scale factor. */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /** @brief Scale of each level relative to level 0. */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /** @brief Inverse scale of each level. */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /** @brief Per-level σ² (scale-dependent noise). */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /** @brief Per-level 1/σ². */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;  ///< Image pyramid cache

protected:

    /** @brief Build the Gaussian pyramid. */
    void ComputePyramid(cv::Mat image);
    /** @brief Per-level FAST + octree keypoint distribution. */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    /**
     * @brief Select up to nFeatures points inside a rectangular ROI via octree.
     */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    /** @brief Legacy grid distribution (kept for compatibility). */
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;  ///< ORB sampling pattern

    int nfeatures;       ///< Total feature budget
    double scaleFactor;  ///< Inter-level scale
    int nlevels;         ///< Level count
    int iniThFAST;       ///< Initial FAST threshold
    int minThFAST;       ///< Minimum FAST threshold

    std::vector<int> mnFeaturesPerLevel;  ///< Target features per level

    std::vector<int> umax;  ///< Circular patch bounds (orientation)

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

}  // namespace feature
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FEATURE_ORB_ORB_EXTRACTOR_HPP_
