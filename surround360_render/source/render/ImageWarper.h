/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <vector>

#include "Camera.h"
#include "CvUtil.h"

namespace surround360 {
namespace warper {

using namespace cv;
using namespace std;

enum CubemapFace {
  CUBEMAP_FACE_BACK,
  CUBEMAP_FACE_LEFT,
  CUBEMAP_FACE_FRONT,
  CUBEMAP_FACE_RIGHT,
  CUBEMAP_FACE_TOP,
  CUBEMAP_FACE_BOTTOM
};


// x and y are normalized pixel coordinates on a cubemap face, in [-0.5, 0.5). returns a
// vector direction, which is not normalized. used in mapEquirectToCubemapCoordinate
inline Vec3f cubemapIndexToVec3(
  const float x,
  const float y,
  const CubemapFace face);

// x and y are normalized pixel coordinates on a cubemap face, in [-0.5, 0.5). results are
// written into srcX and srcY, which specify the pixel coordinate in the source equirect
// image to sample corresponding to the cubemap-face coordinate (x, y).
void mapEquirectToCubemapCoordinate(
  const float x,
  const float y,
  const CubemapFace& face,
  const Mat& srcEqrMat, // only used to get its size()
  const float fisheyeFovRadians, // most common use case: pass in pi
  float& srcX,
  float& srcY);

// takes a spherical projection aka equirect image and generates a cubemap
vector<Mat> convertSphericalToCubemapBicubicRemap(
  const cv::Mat& srcSphericalImage,
  const float fisheyeFovRadians, // for a full equirect source, pass in pi
  const int faceWidth,
  const int faceHeight);

void bicubicRemapToSpherical(
  Mat& dst,
  const Mat& src,
  const Camera& camera,
  const float leftAngle,
  const float rightAngle,
  const float topAngle,
  const float bottomAngle);

Camera::Vector2 projectEquirectToCam(
  const float srcTheta,
  const float srcPhi,
  const Camera& destCam,
  const float depth);

} // namespace warper
} // namespace surround360
