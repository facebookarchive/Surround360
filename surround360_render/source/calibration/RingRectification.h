/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <string>
#include <vector>

#include "CameraMetadata.h"
#include "CvUtil.h"

namespace surround360 {
namespace calibration {

using namespace std;
using namespace cv;
using namespace surround360;

// stores the indices of a pair of images, and the indices of a keypoint in each image
struct KeypointMatch {
  int imageA, imageB, keypointA, keypointB;
  KeypointMatch(int imageA, int imageB, int keypointA, int keypointB) :
    imageA(imageA),
    imageB(imageB),
    keypointA(keypointA),
    keypointB(keypointB) {}
};

// given an image size, and 4 vectors representing displacements of the corners from their
// original positions, find the perspective transform matrix that is closest to generating
// that distortion.
Mat getPerspectiveTransformFrom4CornerDisplacement(
  const cv::Size& imageSize,
  const Point2f& deltaTopLeft,
  const Point2f& deltaTopRight,
  const Point2f& deltaBottomRight,
  const Point2f& deltaBottomLeft);

// a solution vector is a vector of size 8 * n where n = # of cameras. this function
// converts a solution vector to a vector of n perspective transform matrices.
vector<Mat> solutionVectorToTransforms(
  const cv::Size& imageSize,
  const vector<float>& solution);

// We rectify (= perspective transform) the keypoints in rectilinear coords, but
// the objective is evaluated on the keypoints in spherical coords.
// The perspective transform applied to a rectilinear image is equivalent to
// a change in camera pose. It is not true however that a perspective transform
// applied to a spherical image means anything like that
float rectificationObjective(
  const float regularizationCoef,
  const int numCameras,
  const cv::Size& imageSize,
  const vector<float>& solution,
  const vector<vector<Point2f>>& keypoints,
  const vector<KeypointMatch>& matches,
  const vector<CameraMetadata>& camModelArray);

// returns the gradient of rectificationObjective, computed by finite differences
vector<float> rectificationObjectiveGradient(
  const float regularizationCoef,
  const int numCameras,
  const cv::Size& imageSize,
  const vector<float>& solution,
  const vector<vector<Point2f>>& keypoints,
  const vector<KeypointMatch>& matches,
  const vector<CameraMetadata>& camModelArray);

// takes a dataset consisting of one or more collections of images from the side cameras
// of a ring-shaped rig (i.e. frames from several different scenes). finds matching
// keypoints between adjacent image pairs, and uses these to optimize an objective that
// measures stereo rectification jointly across all neighbor pairs.
// returns a vector of perspective transform matrices, one per image. these are only valid
// when applied to rectilinear projections.
vector<Mat> optimizeRingRectification(
  const vector<CameraMetadata>& camModelArray, // side cameras only
  const vector<vector<Mat>> sideCamImagesFeatures, // rectilinear images
  const int numImgsPerCam,
  const string matchVisDir);

} // namespace calibration
} // namespace surround360
