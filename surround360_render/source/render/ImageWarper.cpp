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

#include "CameraMetadata.h"
#include "MathUtil.h"

namespace surround360 {
namespace warper {

using namespace cv;
using namespace std;
using namespace surround360::math_util;
using namespace surround360::calibration;
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
    const int faceImageSize) {

  static const vector<CubemapFace> faces = {
    CUBEMAP_FACE_RIGHT,
    CUBEMAP_FACE_LEFT,
    CUBEMAP_FACE_TOP,
    CUBEMAP_FACE_BOTTOM,
    CUBEMAP_FACE_BACK,
    CUBEMAP_FACE_FRONT
  };

  const float dy = 1.0f / float(faceImageSize);
  const float dx = 1.0f / float(faceImageSize);

  int faceOffset = 0;
  vector<Mat> faceImages;
  for (const CubemapFace& face : faces) {
    Mat warpMat = Mat(Size(faceImageSize, faceImageSize), CV_32FC2);
    for (int i = 0; i < faceImageSize; ++i) {
      for (int j = 0; j < faceImageSize; ++j) {
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
    faceOffset += faceImageSize;
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

Mat bicubicRemapFisheyeToSpherical(
    const CameraMetadata& camModel,
    const Mat& fisheyeImage,
    const Size sphericalImageSize) {

  assert(camModel.isFisheye);

  Mat warpMat = Mat(sphericalImageSize.height, sphericalImageSize.width, CV_32FC2);
  const float dTheta = 2.0f * M_PI / float(sphericalImageSize.width);

  for (int i = 0; i < sphericalImageSize.width; ++i) {
    const float theta = -i * dTheta + M_PI
      + toRadians(camModel.fisheyeRotationDegrees);
    for (int j = 0; j < sphericalImageSize.height; ++j) {
      const float r = j / float(sphericalImageSize.height);
      const float rPix = camModel.usablePixelsRadius * r;
      const float srcX = camModel.imageCenterX + cosf(theta) * rPix;
      const float srcY = camModel.imageCenterY + sinf(theta) * rPix;
      warpMat.at<Point2f>(j, i) = Point2f(srcX, srcY);
    }
  }

  Mat sphericalImage(sphericalImageSize.height, sphericalImageSize.width, CV_8UC3);
  remap(
    fisheyeImage,
    sphericalImage,
    warpMat,
    Mat(),
    CV_INTER_CUBIC,
    BORDER_CONSTANT);
  return sphericalImage;
}

Mat sideFisheyeToSpherical(
    const Mat& src,
    const CameraMetadata& camModel,
    const int outWidth,
    const int outHeight) {

  Mat srcRGBA = src;
  if (src.type() == CV_8UC3) {
    cvtColor(src, srcRGBA, CV_BGR2BGRA);
  }
  Mat warpMat(Size(outWidth, outHeight), CV_32FC2);
  for (int y = 0; y < outHeight; ++y) {
    for (int x = 0; x < outWidth; ++x) {
      const float theta = M_PI * (1.0 - float(x) / float(outWidth));
      const float phi = M_PI * float(y) / float(outHeight);
      const float xSphere = cos(theta) * sin(phi);
      const float ySphere = sin(theta) * sin(phi);
      const float zSphere = cos(phi);
      const float xRot = xSphere;
      const float yRot = ySphere;
      const float zRot = zSphere;
      const float theta2 = atan2(-zRot, xRot);
      const float phi2 = acos(yRot);
      const float r = 2.0 * phi2 / M_PI;
      const float srcX = camModel.imageCenterX + camModel.usablePixelsRadius * r * cos(theta2);
      const float srcY = camModel.imageCenterY + camModel.usablePixelsRadius * r * sin(theta2);
      warpMat.at<Point2f>(y, x) = Point2f(srcX, srcY);
    }
  }
  Mat eqrImage(Size(outWidth, outHeight), CV_8UC4);
  remap(
    srcRGBA,
    eqrImage,
    warpMat,
    Mat(),
    CV_INTER_CUBIC,
    BORDER_CONSTANT,
    Scalar(0, 0, 0, 0));
  return eqrImage;
}

} // namespace warper
} // namespace surround360
