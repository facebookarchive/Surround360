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

// build a model for intrinsic calibration (fisheye correction) from a
// collection of images of checkerboards. note: for checkerSize, sensorWidth,
// and sensorHeight, it is OK to pass in values of 1.0 for many purposes (you
// will still get reasonable results even for quantities like FOV apparently).
void intrinsicCheckerboardCalibration(
  const double checkerSize, // checkerSize, sensorWidth/Height - real units
  const double sensorWidth,
  const double sensorHeight,
  const int checkerboardWidth,
  const int checkerboardHeight,
  const int resizeWidth,
  const int resizeHeight,
  const std::vector<std::string>& srcFilenames,
  const bool showUndistortedImagesInGUI,
  cv::Mat& intrinsic,
  cv::Mat& distCoeffs);

// uses the model built by intrinsicCheckerboardCalibration to
// remove fisheye lens distortion. Then resizes the image, and saves it. if a
// different file extension from the original is specified, it is converted
void undistortResizeConvert(
  const int resizeWidth,
  const int resizeHeight,
  const cv::Mat& intrinsic,
  const cv::Mat& distCoeffs,
  const std::string& inputFilename,
  const std::string& outputFilename);

// given a point in rectilinear, find its projection to spherical coordinates.
// for derivation, see http://mathinsight.org/spherical_coordinates
// and https://www.facebook.com/pxlcld/nGpT
cv::Point2f rectilinearToSpherical(
  const cv::Point2f& point,
  const cv::Size& imageSize,
  const CameraMetadata& camModel);

// given a rectilinear image, project it to spherical coordinates.
// for derivation, see https://www.facebook.com/pxlcld/nlFR
cv::Mat projectRectilinearToSpherical(
  const cv::Mat& srcRectilinear,
  const float fovHorizontalDeg,
  const float fovVerticalDeg,
  const int outputWidth,
  const int outputHeight);

// this is the same as cv::undistort, but uses bicubic instead of bilinear
void cvUndistortBicubic(
  InputArray _src,
  OutputArray _dst,
  InputArray _cameraMatrix,
  InputArray _distCoeffs,
  InputArray _newCameraMatrix);

// in eariler versions of the pipeline, we would correct for barrel distortion
// first, then later project to cylindrical or spherical coordinates. to
// preserve more image quality, it is better to combine these two warps
cv::Mat undistortToSpherical(
  const float fovHorizontalDeg,
  const float fovVerticalDeg,
  const int resizeWidth,
  const int resizeHeight,
  const cv::Mat& intrinsic,
  const cv::Mat& distCoeffs,
  const cv::Mat& perspectiveTransform,
  const cv::Mat& srcImage,
  const int alphaFeatherPix,
  const bool skipUndistort);

// the input image should be taken with a diffusing material over the lens
// (e.g., a tissue) and a bright lightsource behind. we threshold the image, then find
// the resulting blob's center of mass and area to estimate optical center and radius.
cv::Mat estimateOpticalCenterFromDiffusedImage(const Mat& image, const int t);

} // namespace calibration
} // namespace surround360
