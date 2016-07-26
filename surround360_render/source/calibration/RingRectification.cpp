/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "RingRectification.h"

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "CameraMetadata.h"
#include "CvUtil.h"
#include "IntrinsicCalibration.h"
#include "KeypointMatchers.h"
#include "MathUtil.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace surround360 {
namespace calibration {

using namespace std;
using namespace cv;
using namespace surround360;

static const int kDimPerImage = 8; // # of elements of the solution vector per image

Mat getPerspectiveTransformFrom4CornerDisplacement(
    const cv::Size& imageSize,
    const Point2f& deltaTopLeft,
    const Point2f& deltaTopRight,
    const Point2f& deltaBottomRight,
    const Point2f& deltaBottomLeft) {

  Point2f inputQuad[4] = {
    Point2f(0, 0),
    Point2f(imageSize.width - 1, 0),
    Point2f(imageSize.width - 1, imageSize.height - 1),
    Point2f(0, imageSize.height - 1)
  };

  Point2f outputQuad[4] = {
    deltaTopLeft * imageSize.width,
    Point2f(imageSize.width - 1, 0) + deltaTopRight * imageSize.width,
    Point2f(imageSize.width - 1, imageSize.height - 1) + deltaBottomRight * imageSize.width,
    Point2f(0, imageSize.height - 1) + deltaBottomLeft * imageSize.width,
  };

  return getPerspectiveTransform(inputQuad, outputQuad);
}

vector<Mat> solutionVectorToTransforms(
    const cv::Size& imageSize,
    const vector<float>& solution) {

  assert(solution.size() % kDimPerImage == 0);
  int n = solution.size() / kDimPerImage;
  vector<Mat> transforms;
  for (int i = 0; i < n; ++i) {
    transforms.push_back(
      getPerspectiveTransformFrom4CornerDisplacement(
        imageSize,
        Point2f(solution[i * kDimPerImage + 0], solution[i * kDimPerImage + 1]),
        Point2f(solution[i * kDimPerImage + 2], solution[i * kDimPerImage + 3]),
        Point2f(solution[i * kDimPerImage + 4], solution[i * kDimPerImage + 5]),
        Point2f(solution[i * kDimPerImage + 6], solution[i * kDimPerImage + 7])));
  }
  return transforms;
}

float rectificationObjective(
    const float regularizationCoef,
    const int numCameras,
    const cv::Size& imageSize,
    const vector<float>& solution,
    const vector<vector<Point2f>>& keypoints,
    const vector<KeypointMatch>& matches,
    const vector<CameraMetadata>& camModelArray) {

  // map the solution vector to a transform for each image
  const vector<Mat> perspectiveTransforms = solutionVectorToTransforms(
    imageSize, solution);

  // transform each image's keypoints
  vector<vector<Point2f>> transformedKeypoints(numCameras, vector<Point2f>());
  for (int imageIdx = 0; imageIdx < numCameras; ++imageIdx) {
    const Matx33f perspectiveTransform33 = perspectiveTransforms[imageIdx];
    for (const Point2f& point : keypoints[imageIdx]) {
      // Apply projective transform
      const Point3f homogeneousPoint = perspectiveTransform33 * point;

      // Get 2D by normalizing to Z=1
      const Point2f transformedPoint(
        homogeneousPoint.x / homogeneousPoint.z,
        homogeneousPoint.y / homogeneousPoint.z);

      // Transform to spherical
      const Point2f shpericalPoint = rectilinearToSpherical(
        transformedPoint,
        imageSize,
        camModelArray[imageIdx]);

      transformedKeypoints[imageIdx].push_back(shpericalPoint);
    }
  }

  // The objective includes the sum of squared y-differences (MSE) between matched
  // keypoint coordinates in spherical space
  float objective = 0.0f;
  for (const KeypointMatch& match : matches) {
    const Point2f& pointA = transformedKeypoints[match.imageA][match.keypointA];
    const Point2f& pointB = transformedKeypoints[match.imageB][match.keypointB];
    objective += (pointA.y - pointB.y) * (pointA.y - pointB.y);
  }
  objective /= float(matches.size());

  float regularization = 0.0f;
  for (int i = 0; i < solution.size(); ++i) {
    regularization += fabsf(solution[i]);
  }
  regularization /= float(solution.size());
  return objective + regularizationCoef * regularization;
}

vector<float> rectificationObjectiveGradient(
    const float regularizationCoef,
    const int numCameras,
    const cv::Size& imageSize,
    const vector<float>& solution,
    const vector<vector<Point2f>>& keypoints,
    const vector<KeypointMatch>& matches,
    const vector<CameraMetadata>& camModelArray) {

  static const float kEpsilon = 0.000001f;
  const float objectiveAtCurrVal = rectificationObjective(
    regularizationCoef,
    numCameras,
    imageSize,
    solution,
    keypoints,
    matches,
    camModelArray);
  vector<float> gradient(solution.size(), 0.0f);
  for (int i = 0; i < solution.size(); ++i) {
    vector<float> perturbedSolution(solution);
    perturbedSolution[i] += kEpsilon;
    const float objectiveAtPerturbed = rectificationObjective(
      regularizationCoef,
      numCameras,
      imageSize,
      perturbedSolution,
      keypoints,
      matches,
      camModelArray);
    gradient[i] = (objectiveAtPerturbed - objectiveAtCurrVal) / kEpsilon;
  }
  return gradient;
}

vector<Mat> optimizeRingRectification(
    const vector<CameraMetadata>& camModelArray, // side cameras only
    const vector<vector<Mat>> sideCamImagesFeatures, // rectilinear images
    const int numImgsPerCam,
    const string matchVisDir) { // path to write debug images

  const int numSideCameras = camModelArray.size();
  vector<float> rectificationVector(numSideCameras * kDimPerImage, 0.0f);

  // these data structures describe all of the matching keypoints between image pairs
  vector<KeypointMatch> matches;
  vector<vector<Point2f>> keypoints(numSideCameras, vector<Point2f>());

  // for each pair of adjacent cameras, find matching keypoints
  for (int iFeat = 0; iFeat < numImgsPerCam; iFeat++) {
    for (int leftIdx = 0; leftIdx < numSideCameras; ++leftIdx) {
      const int rightIdx = (leftIdx + 1) % numSideCameras;
      LOG(INFO) << "matching keypoints between camera pair: "
        << to_string(iFeat) << " "
        << camModelArray[leftIdx].cameraId << " "
        << camModelArray[rightIdx].cameraId;
      vector< pair<Point2f, Point2f> > matchPointPairsLR;
      getKeypointMatchesWithAllAlgorithms(
        sideCamImagesFeatures[iFeat][leftIdx],
        sideCamImagesFeatures[iFeat][rightIdx],
        matchPointPairsLR);
      Mat visualization = visualizeKeypointMatches(
        sideCamImagesFeatures[iFeat][leftIdx],
        sideCamImagesFeatures[iFeat][rightIdx],
        matchPointPairsLR);

      string imgPath =
        matchVisDir + "/" + to_string(iFeat) + "_" +
        camModelArray[leftIdx].cameraId + "_" +
        camModelArray[rightIdx].cameraId + ".png";

      imwriteExceptionOnFail(imgPath, visualization);

      for (auto& ptPair : matchPointPairsLR) {
        keypoints[leftIdx].push_back(ptPair.first);
        keypoints[rightIdx].push_back(ptPair.second);
        matches.push_back(KeypointMatch(
          leftIdx,
          rightIdx,
          keypoints[leftIdx].size() - 1,
          keypoints[rightIdx].size() - 1));
      }
    }
  }

  // optimize rectification
  static const float kRegularizationCoef = 1000.0f;
  static const float kGradientStepSize   = 0.0000005f;
  static const int kNumGradientItrs      = 2000;
  for (int itr = 0; itr < kNumGradientItrs; ++itr) {
    const float currObjective = rectificationObjective(
      kRegularizationCoef,
      numSideCameras,
      sideCamImagesFeatures[0][0].size(),
      rectificationVector,
      keypoints,
      matches,
      camModelArray);

    vector<float> gradient = rectificationObjectiveGradient(
      kRegularizationCoef,
      numSideCameras,
      sideCamImagesFeatures[0][0].size(),
      rectificationVector,
      keypoints,
      matches,
      camModelArray);
    for(int i = 0; i < rectificationVector.size(); ++i) {
      rectificationVector[i] -= kGradientStepSize * gradient[i];
    }
    LOG(INFO) << itr << "\t" << currObjective;
  }

  vector<Mat> finalTransforms = solutionVectorToTransforms(
    sideCamImagesFeatures[0][0].size(),
    rectificationVector);
  return finalTransforms;
}

} // namespace calibration
} // namespace surround360
