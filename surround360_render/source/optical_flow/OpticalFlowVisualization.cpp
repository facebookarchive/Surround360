/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "OpticalFlowVisualization.h"

namespace surround360 {
namespace optical_flow {

using namespace std;
using namespace cv;
using namespace surround360::optical_flow;

Mat visualizeFlowAsGreyDisparity(const Mat& flow) {
  Mat disparity(flow.size(), CV_32F);
  for (int y = 0; y < flow.rows; ++y) {
    for (int x = 0; x < flow.cols; ++x) {
      disparity.at<float>(y, x) = flow.at<Vec2f>(y, x)[0];
    }
  }
  normalize(disparity, disparity, 0, 255, NORM_MINMAX, CV_32F);
  Mat disaprity8U;
  disparity.convertTo(disaprity8U, CV_8U);
  return disaprity8U;
}

Mat visualizeFlowAsVectorField(const Mat& flow, const Mat& image) {
  static const int kGridSpacing = 12;
  static const Scalar kGridColor = Scalar(0, 0, 0, 255);
  static const float kArrowLen = 7.0f;
  Mat imageWithFlowLines = image.clone();
  for (int y = kGridSpacing; y < image.rows - kGridSpacing; ++y) {
    for (int x = kGridSpacing; x < image.cols - kGridSpacing; ++x) {
      if (x % kGridSpacing == 0 && y % kGridSpacing == 0) {
        Point2f fxy = flow.at<Point2f>(y, x);
        const float mag = sqrt(fxy.x * fxy.x + fxy.y * fxy.y);
        const static float kEpsilon = 0.1f;
        fxy /= mag + kEpsilon;
        line(
          imageWithFlowLines,
          Point(x, y),
          Point(x + fxy.x * kArrowLen, y + fxy.y * kArrowLen),
          kGridColor,
          1,
          CV_AA);
      }
    }
  }
  return imageWithFlowLines;
}

Mat visualizeFlowColorWheel(const Mat& flow) {
  Mat rgbVis(flow.size(), CV_8UC3);
  const static float kDisplacementScale = 20.0f;
  const float maxExpectedDisplacement =
    float(max(flow.size().width, flow.size().height)) / kDisplacementScale;
  for (int y = 0; y < flow.rows; ++y) {
    for (int x = 0; x < flow.cols; ++x) {
      Point2f flowVec = flow.at<Point2f>(y, x);
      const float mag = sqrt(flowVec.x * flowVec.x + flowVec.y * flowVec.y);
      flowVec /= mag;
      const float brightness = .25f + .75f * min(1.0f, mag / maxExpectedDisplacement);
      const float hue = (atan2(flowVec.y, flowVec.x) + M_PI) / (2.0 * M_PI);
      rgbVis.at<Vec3b>(y, x)[0] = 180.0f * hue;
      rgbVis.at<Vec3b>(y, x)[1] = 255.0f * brightness;
      rgbVis.at<Vec3b>(y, x)[2] = 255.0f * brightness;
    }
  }
  cvtColor(rgbVis,  rgbVis, CV_HSV2BGR);
  return rgbVis;
}

Mat testColorWheel() {
  const static cv::Size kTestImageSize(256, 256);
  Mat flow(kTestImageSize, CV_32FC2);
  for (int y = 0; y < kTestImageSize.height; ++y) {
    for (int x = 0; x < kTestImageSize.width; ++x) {
      const float dx = float(kTestImageSize.width) / 2.0f - x;
      const float dy = float(kTestImageSize.height) / 2.0f - y;
      flow.at<Vec2f>(y, x) = Vec2f(dx, dy);
    }
  }
  return visualizeFlowColorWheel(flow);
}

} // namespace optical_flow
} // namespace surround360
