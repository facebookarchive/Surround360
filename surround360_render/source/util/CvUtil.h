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

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "CvUtil.h"

namespace surround360 {
namespace util {

using namespace std;
using namespace cv;

// 259: TIFF compression tag. 1: dump mode
static const vector<int> tiffParams = {259, 1};

// wrapper for cv::imread which throws an exception if loading fails
Mat imreadExceptionOnFail(const string& filename, const int flags = IMREAD_COLOR);

// wrapper for cv::imwrite which throws an exception if writing fails
void imwriteExceptionOnFail(
  const string& filename,
  const Mat& image,
  const vector<int>& params = vector<int>());

// Converts input 8-bit image to 16-bit
Mat convert8bitTo16bit(const Mat& inputImage);

// given a vector of images, stack them horizontally or vertically to form a larger image
Mat stackHorizontal(const std::vector<Mat>& images);
Mat stackVertical(const std::vector<Mat>& images);

// horizontally offset the pixels of an image, and wrap around at the sides
Mat offsetHorizontalWrap(const Mat& srcImage, const float offset);

// combines 6 cube faces into a cubemap, in either "photo" or "video" format
Mat stackOutputCubemapFaces(const string& format, const std::vector<Mat>& images);

// takes a 4 channel image where the alpha channel is a binary mask, and makes a new
// version where the alpha channel blends out smoothly. erodeSize must be odd.
Mat featherAlphaChannel(const Mat& src, int erodeSize);

// binary serialization for cv::Mats of type CV_32FC2
void saveFlowToFile(const Mat& flow, const string& filename);
Mat readFlowFromFile(const string& filename);

// given a 4-channel image, make a circle centered at the middle and set the alpha channel
// to 0 outside the circle.
void circleAlphaCut(Mat& imageBGRA, const float radius);

// for any pixel in redMaskBGR that is red=255, green=0, blue=0, set the alpha of
// destBGRA to 0.
void cutRedMaskOutOfAlphaChannel(Mat& destBGRA, Mat& redMaskBGR);

// flattens two layers, but if it looks like there will be ghosting, prefer the bottom
// (base) color to the top.
Mat flattenLayersDeghostPreferBase(
  const Mat& bottomLayer,
  const Mat& topLayer);

// build a linear regression model that maps colors in imageToAdjust to
// corresponding colors in targetImage. the model is of the form R^4->R^3.
vector<vector<float>> buildColorAdjustmentModel(
  const Mat& targetImage,
  const Mat& imageToAdjust);

// bottomLayer can be either 3 or 4 channel, and topLayer must be 4-channel
template <typename BasePixelType>
static Mat flattenLayers(
  const Mat& bottomLayer,
  const Mat& topLayer) {

  Mat mergedImage(bottomLayer.size(), CV_8UC4);
  for (int y = 0; y < bottomLayer.rows; ++y) {
    for (int x = 0; x < bottomLayer.cols; ++x) {
      const BasePixelType baseColor = bottomLayer.at<BasePixelType>(y, x);
      const Vec4b topColor = topLayer.at<Vec4b>(y, x);
      const float topA = float(topColor[3]) / 255.0f;
      const unsigned char blendR =
        float(topColor[0]) * topA + float(baseColor[0]) * (1.0 - topA);
      const unsigned char blendG =
        float(topColor[1]) * topA + float(baseColor[1]) * (1.0 - topA);
      const unsigned char blendB =
        float(topColor[2]) * topA + float(baseColor[2]) * (1.0 - topA);
      const unsigned char blendAlpha = max(topColor[3], baseColor[3]);
      mergedImage.at<Vec4b>(y, x) = Vec4b(blendR, blendG, blendB, blendAlpha);
    }
  }
  return mergedImage;
}

// Applies radial alpha mask to input image
void radialAlphaFade(Mat& img);

// Applies top-down alpha mask to input image
void topDownAlphaFade(Mat& img);

// Applies a softmax function to the input color layers
Mat flattenLayersAlphaSoftmax(
  const vector<Mat>& layers, const float softmaxCoef);

} // namespace util
} // namespace surround360
