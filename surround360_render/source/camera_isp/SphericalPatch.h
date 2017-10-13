/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

namespace surround360 {

class SphericalPatch {
 protected:
  // True if we wrap in theta
  bool straddlesBoundary_;

  // Start and end longitude
  const float startTheta_;
  const float endTheta_;
  // Start and latitude
  const float startPhi_;
  const float endPhi_;

  // Patch range in pixels
  const float width_;
  const float widthRecip_;
  const float height_;
  const float heightRecip_;

  // Patch range in degrees
  const float thetaRange_;
  const float thetaRangeRecip_;
  const float phiRange_;
  const float phiRangeRecip_;

  bool lastQueryInRange_;

 public:
  SphericalPatch(
    const float startTheta,
    const float endTheta,
    const float startPhi,
    const float endPhi,
    const float width,
    const float height) :
    straddlesBoundary_(endTheta > 360.0f),
    startTheta_(startTheta),
    endTheta_(endTheta),
    startPhi_(startPhi),
    endPhi_(endPhi),
    width_(width),
    widthRecip_(1.0f / width),
    height_(height),
    heightRecip_(1.0f / height),
    thetaRange_(endTheta - startTheta),
    thetaRangeRecip_(1.0f / thetaRange_),
    phiRange_(endPhi - startPhi),
    phiRangeRecip_(1.0f / phiRange_),
    lastQueryInRange_(true) {}

  inline float getX(const float theta) {
    const float thetaP = straddlesBoundary_ && theta < 180.0f ? theta + 360.0f : theta;
    const float x = (thetaP - startTheta_) * width_ * thetaRangeRecip_;
    lastQueryInRange_ &= (0.0f <= x && x < width_);
   return x;
  }

  inline float getY(const float phi) {
    const float y = (phi - startPhi_) * height_ * phiRangeRecip_;
    lastQueryInRange_ &= (0.0f <= y && y < height_);
    return y;
  }

  inline float getTheta(const float x) {
    const float theta = x * widthRecip_ * thetaRange_ + startTheta_;
    lastQueryInRange_ &= (startTheta_ <= theta && theta < endTheta_);
    return theta;
  }

  inline float getPhi(const float y) {
    const float phi = y * heightRecip_ * phiRange_ + startPhi_;
    lastQueryInRange_ &= (startPhi_ <= phi && phi < endPhi_);
    return phi;

  }

  bool inRange() {
    const bool b = lastQueryInRange_;
    lastQueryInRange_ = true;
    return b;
  }
};

}
