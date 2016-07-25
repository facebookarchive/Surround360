/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "KeypointMatchers.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace surround360 {
namespace calibration {

using namespace std;
using namespace cv;
using namespace cv::detail;

void getKeypointMatchesWithBRISK(
    const Mat& imageL,
    const Mat& imageR,
    vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

  static const int kFlannMaxDistScale = 3;
  static const double kFlannMaxDistThreshold = 0.04;

  vector<KeyPoint> kptsL, kptsR;
  Mat descL, descR;
  vector<DMatch> goodMatches;

  Ptr<BRISK> brisk = BRISK::create();
  brisk->detectAndCompute(imageL, noArray(), kptsL, descL);
  brisk->detectAndCompute(imageR, noArray(), kptsR, descR);

  // FlannBasedMatcher with KD-Trees needs CV_32
  descL.convertTo(descL, CV_32F);
  descR.convertTo(descR, CV_32F);

  // KD-Tree param: # of parallel kd-trees
  static const int kFlannNumTrees = 4;
  FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
  vector<DMatch> flannMatches;
  matcher.match(descL, descR, flannMatches);

  double maxDist = 0;
  double minDist = numeric_limits<float>::max() ;
  for(int i = 0; i < flannMatches.size(); ++i) {
    double dist = flannMatches[i].distance;
    maxDist = max(maxDist, dist);
    minDist = min(minDist, dist);
  }

  for(int i = 0; i < flannMatches.size(); ++i) {
    double distThresh = kFlannMaxDistScale * minDist;
    if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
      goodMatches.push_back(flannMatches[i]);
    }
  }

  for (const DMatch& match : goodMatches) {
    const Point2f& kptL = kptsL[match.queryIdx].pt;
    const Point2f& kptR = kptsR[match.trainIdx].pt;
    matchPointPairsLR.push_back(make_pair(kptL, kptR));
  }

  LOG(INFO) << "# matches from BRISK = " << goodMatches.size();
}

void getKeypointMatchesWithORB(
    const Mat& imageL,
    const Mat& imageR,
    vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

  static const bool kUseGPU = false;
  static const float kMatchConfidence = 0.4;

  OrbFeaturesFinder finder;

  ImageFeatures imgFeaturesL;
  finder(imageL, imgFeaturesL);
  imgFeaturesL.img_idx = 0;

  ImageFeatures imgFeaturesR;
  finder(imageR, imgFeaturesR);
  imgFeaturesR.img_idx = 1;

  vector<ImageFeatures> features = {imgFeaturesL, imgFeaturesR};
  vector<MatchesInfo> pairwiseMatches;
  BestOf2NearestMatcher matcher(kUseGPU, kMatchConfidence);
  matcher(features, pairwiseMatches);

  for (const MatchesInfo& matchInfo : pairwiseMatches) {
    if (matchInfo.src_img_idx != 0 || matchInfo.dst_img_idx != 1) {
      continue;
    }
    for (const DMatch& match : matchInfo.matches) {
      const Point2f& kptL = imgFeaturesL.keypoints[match.queryIdx].pt;
      const Point2f& kptR = imgFeaturesR.keypoints[match.trainIdx].pt;
      matchPointPairsLR.push_back(make_pair(kptL, kptR));
    }
    LOG(INFO) << "# matches from ORB = " << matchInfo.matches.size();
    return; // just to be safe. there should ony be one valid matchInfo
  }
}

void getKeypointMatchesWithAKAZE(
    const Mat& imageL,
    const Mat& imageR,
    vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

  static const int kFlannMaxDistScale = 3;
  static const double kFlannMaxDistThreshold = 0.04;

  Mat descL, descR;
  vector<KeyPoint> kptsL, kptsR;
  vector<DMatch> goodMatches;

  Ptr<AKAZE> akaze = AKAZE::create();
  akaze->detectAndCompute(imageL, noArray(), kptsL, descL);
  akaze->detectAndCompute(imageR, noArray(), kptsR, descR);

  // FlannBasedMatcher with KD-Trees needs CV_32
  descL.convertTo(descL, CV_32F);
  descR.convertTo(descR, CV_32F);

  // KD-Tree param: # of parallel kd-trees
  static const int kFlannNumTrees = 4;
  FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
  vector<DMatch> flannMatches;
  matcher.match(descL, descR, flannMatches);

  double maxDist = 0;
  double minDist = numeric_limits<float>::max() ;
  for(int i = 0; i < flannMatches.size(); ++i) {
    double dist = flannMatches[i].distance;
    maxDist = max(maxDist, dist);
    minDist = min(minDist, dist);
  }

  for(int i = 0; i < flannMatches.size(); ++i) {
    double distThresh = kFlannMaxDistScale * minDist;
    if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
      goodMatches.push_back(flannMatches[i]);
    }
  }

  for (const DMatch& match : goodMatches) {
    const Point2f& kptL = kptsL[match.queryIdx].pt;
    const Point2f& kptR = kptsR[match.trainIdx].pt;
    matchPointPairsLR.push_back(make_pair(kptL, kptR));
  }

  LOG(INFO) << "# matches from AKAZE = " << goodMatches.size();
}

void getKeypointMatchesWithAllAlgorithms(
    const Mat& imageL,
    const Mat& imageR,
    vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

  vector<pair<Point2f, Point2f>> matchPointPairsLRAll;
  getKeypointMatchesWithAKAZE(imageL, imageR, matchPointPairsLRAll);
  getKeypointMatchesWithBRISK(imageL, imageR, matchPointPairsLRAll);
  getKeypointMatchesWithORB(imageL, imageR, matchPointPairsLRAll);

  LOG(INFO) << "# matches total = " << matchPointPairsLRAll.size();

  // TODO: remove duplicate keypoints

  // Apply RANSAC to filter weak matches (like, really weak)
  vector<Point2f> matchesL, matchesR;
  vector<uchar> inlinersMask;
  for(int i = 0; i < matchPointPairsLRAll.size(); ++i) {
    matchesL.push_back(matchPointPairsLRAll[i].first);
    matchesR.push_back(matchPointPairsLRAll[i].second);
  }

  static const int kRansacReprojThreshold = 100;
  findHomography(
    matchesL,
    matchesR,
    CV_RANSAC,
    kRansacReprojThreshold,
    inlinersMask);

  for (int i = 0; i < inlinersMask.size(); ++i) {
    if (inlinersMask[i]) {
      matchPointPairsLR.push_back(make_pair(matchesL[i], matchesR[i]));
    }
  }

  LOG(INFO) << "# matches after RANSAC = " << matchPointPairsLR.size();
}

Mat visualizeKeypointMatches(
    const Mat& imageL,
    const Mat& imageR,
    vector< pair<Point2f, Point2f> >& matchPointPairsLR) {

  Mat visualization;
  hconcat(imageL, imageR, visualization);

  static const Scalar kVisPointColor = Scalar(110, 220, 0);
  for (auto& pointPair : matchPointPairsLR) {
    line(
      visualization,
      pointPair.first,
      pointPair.second + Point2f(imageL.cols, 0),
      kVisPointColor,
      1, // thickness
      CV_AA);
  }
  return visualization;
}

} // namespace optical_flow
} // namespace surround360
