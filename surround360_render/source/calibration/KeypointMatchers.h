/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "CvUtil.h"

namespace surround360 {
namespace calibration {

using namespace std;
using namespace cv;

// find keypoints in imageL and imageR, find matches between them using BRISK features.
// returns a vector of pairs of points, where the first point is in imageL and the second
// point is imageR. similar variants for ORB and AKAZE features.
void getKeypointMatchesWithBRISK(
  const Mat& imageL,
  const Mat& imageR,
  vector< pair<Point2f, Point2f> >& matchPointPairsLR);

void getKeypointMatchesWithORB(
  const Mat& imageL,
  const Mat& imageR,
  vector< pair<Point2f, Point2f> >& matchPointPairsLR);

void getKeypointMatchesWithAKAZE(
  const Mat& imageL,
  const Mat& imageR,
  vector< pair<Point2f, Point2f> >& matchPointPairsLR);

// combines keypoint matches from BRISK, ORB, and AKAZE, and filters outliers with RANSAC
void getKeypointMatchesWithAllAlgorithms(
  const Mat& imageL,
  const Mat& imageR,
  vector< pair<Point2f, Point2f> >& matchPointPairsLR);

// returns an image consisting of imageL and imageR stacked horizontally, with lines
// between matched keypoints
Mat visualizeKeypointMatches(
  const Mat& imageL,
  const Mat& imageR,
  vector< pair<Point2f, Point2f> >& matchPointPairsLR);

} // namespace optical_flow
} // namespace surround360
