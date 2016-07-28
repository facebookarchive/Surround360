/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <iostream>

#include "CvUtil.h"
#include "IntrinsicCalibration.h"
#include "MathUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace surround360 {
namespace calibration {

using namespace std;
using namespace cv;
using namespace cv::detail;
using namespace math_util;

void intrinsicCheckerboardCalibration(
    const double checkerSize,
    const double sensorWidth,
    const double sensorHeight,
    const int checkerboardWidth,
    const int checkerboardHeight,
    const int resizeWidth,
    const int resizeHeight,
    const vector<string>& srcFilenames,
    const bool showUndistortedImagesInGUI,
    Mat& intrinsic,
    Mat& distCoeffs) {

  Size boardSize = Size(checkerboardWidth, checkerboardHeight);
  vector<vector<Point3f>> objectPoints;
  vector<vector<Point2f>> imagePoints;

  vector<Point3f> checkerboardTemplate;
  for (int j = 0; j < checkerboardHeight; ++j) {
    for (int i = 0; i < checkerboardWidth; ++i) {
      checkerboardTemplate.push_back(Point3f(
        double(i) * checkerSize,
        double(j) * checkerSize,
        0));
    }
  }

  Size smallImageSize = Size(resizeWidth, resizeHeight);
  Size previewSize = Size(resizeWidth/2, resizeHeight/2);

  for (const string& filePath : srcFilenames) {
    if (!filePath.empty() && filePath[0] == '.') {
      continue;
    }

    Mat srcImage = imreadExceptionOnFail(filePath, CV_LOAD_IMAGE_GRAYSCALE);
    LOG(INFO) << "looking for checkerboard in " << filePath;
    Mat smallImage, previewImg;

    resize(srcImage, smallImage, smallImageSize);

    vector<Point2f> cornerPoints;
    bool foundCheckerboard = findChessboardCorners(
      smallImage,
      boardSize,
      cornerPoints,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    LOG(INFO) << "foundCheckerboard = " << foundCheckerboard;

    if (foundCheckerboard) {
      const static int kSubpixelCornerMaxItrs = 30;
      const static double kSubpixelCornerEpsilon = 0.1;
      cornerSubPix(
        smallImage,
        cornerPoints,
        boardSize,
        Size(-1, -1),
        TermCriteria(
          CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
          kSubpixelCornerMaxItrs,
          kSubpixelCornerEpsilon));

      static const int KVisualizationCircleRadius = 16;
      static const Scalar kVisualizationCircleColor = Scalar(0, 0, 255);
      static const int kVisualizationCircleThickness = 2;
      for (const Point2d& pt : cornerPoints) {
        circle(
          smallImage,
          pt,
          KVisualizationCircleRadius,
          kVisualizationCircleColor,
          kVisualizationCircleThickness,
          CV_AA);
      }

      // if we found the checkerboard, add the image data to the list
      imagePoints.push_back(cornerPoints);
      objectPoints.push_back(checkerboardTemplate);
    }
  }

  LOG(INFO) << "calibrating";
  LOG(INFO) << "imagePoints.size()=" << imagePoints.size();
  LOG(INFO) << "objectPoints()=" << objectPoints.size();

  vector<Mat> r, t;
  double reprojectionErr = calibrateCamera(
    objectPoints,
    imagePoints,
    smallImageSize,
    intrinsic,
    distCoeffs,
    r, t);
  LOG(INFO) << "reprojection error=" << reprojectionErr;

  static const double kAperatureWidth = 1.0;
  static const double kAperatureHeight = 1.0;
  double estimatedFovX;
  double estimatedFovY;
  double estimatedFocalLength;
  double estimatedAspectRatio;
  Point2d principalPoint;
  calibrationMatrixValues(
    intrinsic,
    smallImageSize,
    kAperatureWidth,
    kAperatureHeight,
    estimatedFovX,
    estimatedFovY,
    estimatedFocalLength,
    principalPoint,
    estimatedAspectRatio
  );
  LOG(INFO) << "estimatedFovX=" << estimatedFovX;
  LOG(INFO) << "estimatedFovY=" << estimatedFovY;
  LOG(INFO) << "estimatedFocalLength=" << estimatedFocalLength;
  LOG(INFO) << "estimatedAspectRatio=" << estimatedAspectRatio;
  LOG(INFO) << "principalPoint=" << principalPoint;

  if (showUndistortedImagesInGUI) {
    // show de-warped checkerboards
    for (const string& filePath : srcFilenames) {
      Mat srcImage = imreadExceptionOnFail(filePath, 1);
      Mat smallImage, previewImg;
      resize(srcImage, smallImage, smallImageSize);

      Mat imageUndistorted;
      undistort(smallImage, imageUndistorted, intrinsic, distCoeffs);

      resize(imageUndistorted, previewImg, previewSize);
      imshow("preview", previewImg);
      waitKey(0);
    }
  }
}

void cvUndistortBicubic(
    InputArray _src,
    OutputArray _dst,
    InputArray _cameraMatrix,
    InputArray _distCoeffs,
    InputArray _newCameraMatrix) {

  Mat src = _src.getMat();
  Mat cameraMatrix = _cameraMatrix.getMat();
  Mat distCoeffs = _distCoeffs.getMat();
  Mat newCameraMatrix = _newCameraMatrix.getMat();

  _dst.create(src.size(), src.type());
  Mat dst = _dst.getMat();

  CV_Assert(dst.data != src.data);

  int stripe_size0 =
    std::min(std::max(1, (1 << 12) / std::max(src.cols, 1)), src.rows);
  Mat map1(stripe_size0, src.cols, CV_16SC2);
  Mat map2(stripe_size0, src.cols, CV_16UC1);

  Mat_<double> A, Ar, I = Mat_<double>::eye(3,3);

  cameraMatrix.convertTo(A, CV_64F);
  if(!distCoeffs.empty()) {
      distCoeffs = Mat_<double>(distCoeffs);
  } else {
      distCoeffs.create(5, 1, CV_64F);
      distCoeffs = 0.;
  }

  if(!newCameraMatrix.empty()) {
    newCameraMatrix.convertTo(Ar, CV_64F);
  } else {
    A.copyTo(Ar);
  }

  double v0 = Ar(1, 2);
  for(int y = 0; y < src.rows; y += stripe_size0) {
      int stripe_size = std::min(stripe_size0, src.rows - y);
      Ar(1, 2) = v0 - y;
      Mat map1_part = map1.rowRange(0, stripe_size),
          map2_part = map2.rowRange(0, stripe_size),
          dst_part = dst.rowRange(y, y + stripe_size);

      initUndistortRectifyMap(
        A, distCoeffs, I, Ar, Size(src.cols, stripe_size),
        map1_part.type(), map1_part, map2_part);
      remap(src, dst_part, map1_part, map2_part, INTER_CUBIC, BORDER_CONSTANT);
  }
}

void undistortResizeConvert(
    const int resizeWidth,
    const int resizeHeight,
    const Mat& intrinsic,
    const Mat& distCoeffs,
    const string& inputFilename,
    const string& outputFilename) {

  Mat srcImage = imreadExceptionOnFail(inputFilename, 1);
  Mat smallImage, previewImg;
  Size smallImageSize = Size(resizeWidth, resizeHeight);
  resize(srcImage, smallImage, smallImageSize);

  Mat imageUndistorted;
  cvUndistortBicubic(
    smallImage, imageUndistorted, intrinsic, distCoeffs, intrinsic);

  imwriteExceptionOnFail(outputFilename, imageUndistorted);
}

Point2f rectilinearToSpherical(
  const Point2f& point,
  const cv::Size& imageSize,
  const CameraMetadata& camModel) {

  const float fovH = toRadians(camModel.fovHorizontal);
  const float fovV = toRadians(camModel.fovHorizontal / camModel.aspectRatioWH);

  const float f =  float(imageSize.width - 1) / (2.0f * tan(fovH / 2.0f)); // fu == fv

  const float u = float(point.x) - float(imageSize.width - 1) / 2.0f; // u in [-W/2, W/2]
  const float v = float(point.y) - float(imageSize.height - 1) / 2.0f; // v in [-H/2, H/2]
  const float r = sqrt(u*u + v*v + f*f);

  const float theta = atan(u / f);
  const float phi = acos(-v / r); // flip sign (y-coord in Mat goes down)
  const float thetaU = // theta in [-FOVh/2, FOVh/2]
    ((theta + fovH / 2.0f) / fovH) * float(imageSize.width - 1);
  const float phiV =  // phi in [pi/2 - FOVv/2, pi/2 + FOVv/2]
    ((phi - (M_PI / 2.0f - fovV / 2.0f)) / fovV) * float(imageSize.height - 1);

  return Point2f(thetaU, phiV);
}

Mat projectRectilinearToSpherical(
    const Mat& srcRectilinear,
    const float fovHorizontalDeg,
    const float fovVerticalDeg,
    const int outputWidth,
    const int outputHeight) {

  const float fovHorizontalRad = toRadians(fovHorizontalDeg);
  const float fovVerticalRad = toRadians(fovVerticalDeg);

  // do an oversized projection to make sure there is no loss of data, then scale down
  Mat warpMat(Size(srcRectilinear.cols, srcRectilinear.rows), CV_32FC2);
  // fu == fv
  const float f = float(warpMat.cols - 1) / (2.0f * tan(fovHorizontalRad / 2.0f));
  for (int y = 0; y < warpMat.rows; ++y) {
    for (int x = 0; x < warpMat.cols; ++x) {
      const float theta =
        (float(x) / float(warpMat.cols - 1)) * fovHorizontalRad - fovHorizontalRad / 2.0f;
      const float phi =
        -((float(y) / float(warpMat.rows - 1)) * fovVerticalRad - fovVerticalRad / 2.0f +
        M_PI / 2.0f);
      const float u = f * tan(theta); // u in [-W/2, W/2]
      const float v = f / (tan(phi) * cos(theta)); // v in [-V/2, V/2]
      const float srcX = u + float(srcRectilinear.cols - 1) / 2.0f;
      const float srcY = v + float(srcRectilinear.rows - 1) / 2.0f;
      warpMat.at<Point2f>(y, x) = Point2f(srcX, srcY);
    }
  }

  Mat sphericalImageLarge;
  remap(
    srcRectilinear,
    sphericalImageLarge,
    warpMat,
    Mat(),
    INTER_CUBIC, BORDER_CONSTANT,
    Scalar(0, 0, 0, 0));

  Mat sphericalImage;
  resize(
    sphericalImageLarge,
    sphericalImage,
    Size(outputWidth, outputHeight),
    0,
    0,
    INTER_AREA);
  return sphericalImage;
}

Mat undistortToSpherical(
    const float fovHorizontalDeg,
    const float fovVerticalDeg,
    const int resizeWidth,
    const int resizeHeight,
    const Mat& intrinsic,
    const Mat& distCoeffs,
    const Mat& perspectiveTransform,
    const Mat& srcImage,
    const int alphaFeatherPix,
    const bool skipUndistort) {

  Mat undistortedImage;
  if (skipUndistort) { // intrinsic mats are resolution specific, skip it for preview
    undistortedImage = srcImage.clone();
  } else {
    cvUndistortBicubic(
      srcImage, undistortedImage, intrinsic, distCoeffs, intrinsic);
  }

  Mat undistortedImageBGRA;
  cvtColor(undistortedImage, undistortedImageBGRA, CV_BGR2BGRA);

  // add alpha feathering helps optical flow/novel view synthesis handle boundaries
  if (alphaFeatherPix > 0) {
    if (alphaFeatherPix >= undistortedImageBGRA.rows) {
      throw VrCamException(
        "error in projecting camera images to spherical coordinates:"
        " alpha feather size is larger than image size");
    }
    for (int y = 0; y < alphaFeatherPix; ++y) {
      for (int x = 0; x < undistortedImageBGRA.cols; ++x) {
        const unsigned char alpha = 255.0f * float(y) / float(alphaFeatherPix);
        undistortedImageBGRA.at<Vec4b>(y, x)[3] = alpha;
        undistortedImageBGRA.at<Vec4b>(undistortedImageBGRA.rows - 1 - y, x)[3] = alpha;
      }
    }
  }

  // if the perspective transform matrix is non-empty, apply the transform
  if (perspectiveTransform.rows > 0) {
    warpPerspective(
      undistortedImageBGRA,
      undistortedImageBGRA,
      perspectiveTransform,
      undistortedImageBGRA.size(),
      CV_INTER_CUBIC);
  }

  return projectRectilinearToSpherical(
    undistortedImageBGRA, fovHorizontalDeg, fovVerticalDeg, resizeWidth, resizeHeight);

  // TODO: warper->wrap(), cvUndistortBicubic, and resize are all essentially
  // calling cv::remap(). for maximum image quality, all of those remaps would
  // be composed.
}

Mat estimateOpticalCenterFromDiffusedImage(const Mat& image, const int t) {
  Mat grey, mask;
  cvtColor(image, grey, CV_RGB2GRAY);
  cv::threshold(grey, mask, t, 255, THRESH_BINARY);
  long area = 0;
  double centerOfMassX = 0.0;
  double centerOfMassY = 0.0;
  for (int y = 0; y < mask.rows; ++y) {
    for (int x = 0; x < mask.cols; ++x) {
      if (mask.at<unsigned char>(y, x) > 0) {
        centerOfMassX += x;
        centerOfMassY += y;
        ++area;
      }
    }
  }
  centerOfMassX /= double(area);
  centerOfMassY /= double(area);
  const float radius = sqrt(double(area) / M_PI);
  LOG(INFO) << "radius = " << radius;
  LOG(INFO) << "centerOfMassX = " << centerOfMassX;
  LOG(INFO) << "centerOfMassY = " << centerOfMassY;
  return mask;
}

} // namespace calibration
} // namespace surround360
