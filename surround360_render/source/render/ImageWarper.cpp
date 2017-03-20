/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "ImageWarper.h"

#include <iostream>
#include <string>
#include <vector>

#include "MathUtil.h"

namespace surround360 {
namespace warper {

using namespace cv;
using namespace std;
using namespace surround360::math_util;
using namespace surround360::util;

inline Vec3f cubemapIndexToVec3(
    const float x,
    const float y,
    const CubemapFace face) {

  // rotate and flip the direction as a function of the face
  Vec3f dir(x, y, 0.5f);
  Vec3f dirOut = dir;
  switch(face) {
    case CUBEMAP_FACE_BACK:
      dirOut[0] =  dir[0];
      dirOut[1] =  dir[2];
      dirOut[2] = -dir[1];
      break;
    case CUBEMAP_FACE_LEFT:
      dirOut[0] = -dir[2];
      dirOut[1] =  dir[0];
      dirOut[2] = -dir[1];
      break;
    case CUBEMAP_FACE_TOP: break; // no-op
    case CUBEMAP_FACE_BOTTOM:
      dirOut[0] =  dir[0];
      dirOut[1] = -dir[1];
      dirOut[2] = -dir[2];
      break;
    case CUBEMAP_FACE_FRONT:
      dirOut[0] = -dir[0];
      dirOut[1] = -dir[2];
      dirOut[2] = -dir[1];
      break;
    case CUBEMAP_FACE_RIGHT:
      dirOut[0] =  dir[2];
      dirOut[1] = -dir[0];
      dirOut[2] = -dir[1];
      break;
  }
  return dirOut;
}

void mapEquirectToCubemapCoordinate(
    const float x,
    const float y,
    const CubemapFace& face,
    const Mat& srcEqrMat,
    const float fisheyeFovRadians,
    float& srcX,
    float& srcY) {

  const Vec3f dir = cubemapIndexToVec3(x, y, face);
  const float r = sqrtf(square(dir[0]) + square(dir[1]));
  const float phi = acosf(dir[2] / norm(dir));
  float theta = r > 0.0f ? acosf(fabs(dir[0] / r)) : 0.0f;

  if (dir[0] > 0 && dir[1] > 0) { // Quadrant I
    // (nothing to do)
  } else if (dir[0] <= 0 && dir[1] > 0) { // Quadrant II
    theta = M_PI - theta;
  } else if (dir[0] <= 0 && dir[1] <= 0) { // Quadrant III
    theta = M_PI + theta;
  } else  { // Quadrant IV
    theta = 2 * M_PI - theta;
  }

  const float phiPrime =  clamp(phi, 0.0f, fisheyeFovRadians);
  const float thetaPrime = clamp(theta, 0.0f, float(2.0f * M_PI));
  srcX = float(srcEqrMat.cols) * thetaPrime / (2.0f * M_PI);
  srcY = float(srcEqrMat.rows) * phiPrime / fisheyeFovRadians;
}

vector<Mat> convertSphericalToCubemapBicubicRemap(
    const cv::Mat& srcSphericalImage,
    const float fisheyeFovRadians,
    const int faceWidth,
    const int faceHeight) {

  static const vector<CubemapFace> faces = {
    CUBEMAP_FACE_RIGHT,
    CUBEMAP_FACE_LEFT,
    CUBEMAP_FACE_TOP,
    CUBEMAP_FACE_BOTTOM,
    CUBEMAP_FACE_BACK,
    CUBEMAP_FACE_FRONT
  };

  const float dy = 1.0f / float(faceWidth);
  const float dx = 1.0f / float(faceHeight);

  vector<Mat> faceImages;
  for (const CubemapFace& face : faces) {
    Mat warpMat = Mat(Size(faceWidth, faceHeight), CV_32FC2);
    for (int j = 0; j < faceHeight; ++j) {
      for (int i = 0; i < faceWidth; ++i) {
        float srcX;
        float srcY;
        mapEquirectToCubemapCoordinate(
          float(i) * dy - 0.5f,
          float(j) * dx - 0.5f,
          face,
          srcSphericalImage,
          fisheyeFovRadians,
          srcX, srcY);
        warpMat.at<Point2f>(j, i) = Point2f(srcX, srcY);
      }
    }
    Mat faceImage;
    remap(
      srcSphericalImage,
      faceImage,
      warpMat,
      Mat(),
      CV_INTER_CUBIC,
      BORDER_WRAP);
    faceImages.push_back(faceImage);
  }
  return faceImages;
}

void bicubicRemapToSpherical(
    Mat& dst,
    const Mat& src,
    const Camera& camera,
    const float leftAngle,
    const float rightAngle,
    const float topAngle,
    const float bottomAngle) {
  Mat warp(dst.size(), CV_32FC2);
  for (int x = 0; x < warp.cols; ++x) {
    // sweep xAngle from leftAngle to rightAngle
    const float xFrac = (x + 0.5f) / warp.cols;
    const float xAngle = (1 - xFrac) * leftAngle + xFrac * rightAngle;
    for (int y = 0; y < warp.rows; ++y) {
      // sweep yAngle from topAngle to bottomAngle
      const float yFrac = (y + 0.5f) / warp.rows;
      float yAngle = (1 - yFrac) * topAngle + yFrac * bottomAngle;
      const Camera::Vector3 unit(
        cos(yAngle) * cos(xAngle),
        cos(yAngle) * sin(xAngle),
        sin(yAngle));
      const Camera::Vector2 pixel =
        camera.pixel(unit * int(Camera::kNearInfinity));
      warp.at<Point2f>(y, x) = Point2f(pixel.x() - 0.5, pixel.y() - 0.5);
    }
  }
  Mat tmp = src;
  if (src.channels() == 3 && dst.channels() == 4) {
    cvtColor(src, tmp, CV_BGR2BGRA);
  }
  remap(tmp, dst, warp, Mat(), CV_INTER_CUBIC, BORDER_CONSTANT);
}

// srcTheta and srcPhi correspond to a pixel in an equirectangular projection.
// returns the point in the image of destCam corresponding to that pixel in the
// equirect, or (-1, -1) if the camera doesn't see that point.
Camera::Vector2 projectEquirectToCam(
    const float srcTheta,
    const float srcPhi,
    const Camera& destCam,
    const float depth) {

  const Camera::Vector3 sphericalDir(
    sinf(srcPhi) * cosf(srcTheta),
    sinf(srcPhi) * sinf(srcTheta),
    cosf(srcPhi));
  const Camera::Vector3 worldPoint = depth * sphericalDir;

  if (destCam.sees(worldPoint)) {
    return destCam.pixel(worldPoint);
  } else {
    return Camera::Vector2(-1, -1); // this will sample 0-alpha when cv::remap'd
  }
}

} // namespace warper
} // namespace surround360
