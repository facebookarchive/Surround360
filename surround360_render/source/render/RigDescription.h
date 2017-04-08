/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "Camera.h"
#include "CvUtil.h"
#include "SystemUtil.h"

namespace surround360 {

using namespace cv;
using namespace std;
using namespace surround360::util;

struct RigDescription {
  Camera::Rig rig;
  Camera::Rig rigSideOnly;
  RigDescription(const string& filename);

  RigDescription(const Camera::Rig& rig);

  // find the camera that is closest to pointing in the provided direction
  // ignore those with excessive distance from the camera axis to the rig center
  const Camera& findCameraByDirection(
    const Camera::Vector3& direction,
    const Camera::Real distCamAxisToRigCenterMax = 1.0) const;

  // find the camera with the largest distance from camera axis to rig center
  const Camera& findLargestDistCamAxisToRigCenter() const;

  string getTopCameraId() const;

  string getBottomCameraId() const;

  string getBottomCamera2Id() const;

  int getSideCameraCount() const;

  string getSideCameraId(const int idx) const;

  float getRingRadius() const;

  vector<Mat> loadSideCameraImages(
    const string& imageDir,
    const string& frameNumber) const;

private:
  static Camera::Real distCamAxisToRigCenter(const Camera& camera) {
    return camera.rig(camera.principal).distance(Camera::Vector3::Zero());
  }
};

// measured in radians from forward
inline float approximateFov(const Camera& camera, const bool vertical) {
  Camera::Vector2 a = camera.principal;
  Camera::Vector2 b = camera.principal;
  if (vertical) {
    a.y() = 0;
    b.y() = camera.resolution.y();
  } else {
    a.x() = 0;
    b.x() = camera.resolution.x();
  }
  return acos(max(
      camera.rig(a).direction().dot(camera.forward()),
      camera.rig(b).direction().dot(camera.forward())));
}

// measured in radians from forward
inline float approximateFov(const Camera::Rig& rig, const bool vertical) {
  float result = 0;
  for (const auto& camera : rig) {
    result = std::max(result, approximateFov(camera, vertical));
  }
  return result;
}

} // namespace surround360
