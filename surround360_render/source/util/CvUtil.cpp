/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "CvUtil.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <random>

#include "MathUtil.h"
#include "LinearRegression.h"
#include "VrCamException.h"

namespace surround360 {
namespace util {

using namespace std;
using namespace cv;
using namespace math_util;
using namespace linear_regression;

Mat imreadExceptionOnFail(const string& filename, const int flags) {
  const Mat image = imread(filename, flags);
  if (image.empty()) {
    throw VrCamException("failed to load image: " + filename);
  }
  return image;
}

void imwriteExceptionOnFail(
    const string& filename,
    const Mat& image,
    const vector<int>& params) {

  if (!imwrite(filename, image, params)) {
    throw VrCamException("failed to write image: " + filename);
  }
}

Mat convert8bitTo16bit(const Mat& image8) {
  Mat image16(image8.rows, image8.cols, CV_16U);

  for (int y = 0; y < image8.rows; ++y) {
    for (int x = 0; x < image8.cols; ++x) {
      // Use the repeating high order bits in low order bits to
      // correctly fill 15 bits.
      image16.at<uint16_t>(y, x) =
        (uint16_t(image8.at<uint8_t>(y, x)) << 8) |
        (uint16_t(image8.at<uint8_t>(y, x)) & 0xff);
    }
  }

  return image16;
}

Mat stackHorizontal(const std::vector<Mat>& images) {
  assert(!images.empty());
  if (images.size() == 1) {
    return images[0];
  }
  Mat stacked = images[0].clone();
  for (int i = 1; i < images.size(); ++i) {
    hconcat(stacked, images[i], stacked);
  }
  return stacked;
}

Mat stackVertical(const std::vector<Mat>& images) {
  assert(!images.empty());
  if (images.size() == 1) {
    return images[0];
  }
  Mat stacked = images[0].clone();
  for (int i = 1; i < images.size(); ++i) {
    vconcat(stacked, images[i], stacked);
  }
  return stacked;
}

Mat offsetHorizontalWrap(
    const Mat& srcImage,
    const float offset) {

  Mat warpMat = Mat(srcImage.size(), CV_32FC2);
  for (int y = 0; y < srcImage.rows; ++y) {
    for (int x = 0; x < srcImage.cols; ++x) {
      float srcX = float(x) - offset;
      if (srcX < 0) { srcX += srcImage.cols; }
      if (srcX >= srcImage.cols) { srcX -= srcImage.cols; }
      warpMat.at<Point2f>(y, x) = Point2f(srcX, y);
    }
  }
  Mat warpedImage;
  remap(
    srcImage,
    warpedImage,
    warpMat,
    Mat(),
    INTER_NEAREST,
    BORDER_WRAP);
  return warpedImage;
}

Mat stackOutputCubemapFaces(const string& format, const std::vector<Mat>& images) {
  if (format == "video") {
    assert(images.size() == 6);
    const vector<Mat> first3 = {images[1], images[0], images[2]};
    const vector<Mat> second3 = {images[3], images[4], images[5]};
    flip(first3[0], first3[0], 1);
    flip(first3[1], first3[1], 1);
    flip(first3[2], first3[2], 1);
    flip(second3[0], second3[0], 1);
    flip(second3[1], second3[1], 1);
    flip(second3[2], second3[2], 1);
    const Mat first3h = stackHorizontal(first3);
    const Mat second3h = stackHorizontal(second3);
    const vector<Mat> stackV = {first3h, second3h};
    return stackVertical(stackV);
  } else if (format == "photo") {
    return stackVertical(images);
  } else {
    throw VrCamException(
      "unexpected cubemap format: " + format + ". valid formats are: video,photo");
  }
}

Mat featherAlphaChannel(const Mat& src, int erodeSize) {
  vector<Mat> channels;
  split(src, channels);

  // erode the alpha channel
  Mat erosionKernel = cv::getStructuringElement(
    cv::MORPH_CROSS,
    cv::Size(2 * erodeSize + 1, 2 * erodeSize + 1),
    cv::Point(erodeSize, erodeSize) );
  erode(channels[3], channels[3], erosionKernel);

  // blur the eroded alpha channel
  GaussianBlur(channels[3], channels[3], Size(erodeSize, erodeSize), erodeSize / 2.0f);

  Mat dest;
  merge(channels, dest);
  return dest;
}

void saveFlowToFile(const Mat& flow, const string& filename) {
  assert(flow.type() == CV_32FC2);
  int rows = flow.rows;
  int cols = flow.cols;
  FILE* file = fopen(filename.c_str(), "wb");
  if (file == NULL) {
    throw VrCamException("file not found: " + filename);
  }
  fwrite((void*)(&rows), sizeof(rows), 1, file);
  fwrite((void*)(&cols), sizeof(cols), 1, file);
  for (int y = 0; y < flow.rows; ++y) {
    for (int x = 0; x < flow.cols; ++x) {
      float fx = flow.at<Point2f>(y, x).x;
      float fy = flow.at<Point2f>(y, x).y;
      fwrite((void*)(&fx), sizeof(fx), 1, file);
      fwrite((void*)(&fy), sizeof(fy), 1, file);
    }
  }
  fclose(file);
}

Mat readFlowFromFile(const string& filename) {
  FILE* file = fopen(filename.c_str(), "rb");
  if (file == NULL) {
    throw VrCamException("file not found: " + filename);
  }
  int rows, cols;
  fread((void*)&rows, sizeof(rows), 1, file);
  fread((void*)&cols, sizeof(cols), 1, file);
  Mat flow(Size(cols, rows), CV_32FC2);
  for (int y = 0; y < flow.rows; ++y) {
    for (int x = 0; x < flow.cols; ++x) {
      float fx, fy;
      fread((void*)(&fx), sizeof(fx), 1, file);
      fread((void*)(&fy), sizeof(fy), 1, file);
      flow.at<Point2f>(y, x) = Point2f(fx, fy);
    }
  }
  fclose(file);
  return flow;
}

void circleAlphaCut(Mat& imageBGRA, const float radius) {
  for (int y = 0; y < imageBGRA.rows; ++y) {
    for (int x = 0; x < imageBGRA.cols; ++x) {
      const float dx = float(x) - float(imageBGRA.cols) / 2.0f;
      const float dy = float(y) - float(imageBGRA.rows) / 2.0f;
      const float r = sqrtf(dx * dx + dy * dy);
      const float alpha = r < radius ? 1.0f : 0.0f;
      imageBGRA.at<Vec4b>(y, x)[3] = (unsigned char)(alpha * 255.0f);
    }
  }
}

void cutRedMaskOutOfAlphaChannel(Mat& destBGRA, Mat& redMaskBGR) {
  assert(destBGRA.size() == redMaskBGR.size());
  for (int y = 0; y < redMaskBGR.rows; ++y) {
    for (int x = 0; x < redMaskBGR.cols; ++x) {
      if (redMaskBGR.at<Vec3b>(y, x) == Vec3b(0, 0, 255)) {
        destBGRA.at<Vec4b>(y, x)[3] = 0;
      }
    }
  }
}

Mat flattenLayersDeghostPreferBase(
    const Mat& bottomLayer,
    const Mat& topLayer) {

  Mat mergedImage(bottomLayer.size(), CV_8UC4);
  for (int y = 0; y < bottomLayer.rows; ++y) {
    for (int x = 0; x < bottomLayer.cols; ++x) {
      Vec4b baseColor = bottomLayer.at<Vec4b>(y, x);
      Vec4b topColor = topLayer.at<Vec4b>(y, x);

      const float colorDiff =
        (std::abs(baseColor[0] - topColor[0]) +
         std::abs(baseColor[1] - topColor[1]) +
         std::abs(baseColor[2] - topColor[2])) / 255.0f;

      static const float kColorDiffCoef = 5.0f;
      static const float kSoftmaxSharpness = 5.0f;
      static const float kBaseLayerBias = 2.0f;
      const float deghostCoef = tanhf(colorDiff * kColorDiffCoef);
      const float alphaR = topColor[3] / 255.0f;
      const float alphaL = 1.0f - alphaR;
      const double expL = exp(kSoftmaxSharpness * alphaL * kBaseLayerBias);
      const double expR = exp(kSoftmaxSharpness * alphaR);
      const double sumExp = expL + expR + 0.00001;
      const float softmaxL = float(expL / sumExp);
      const float softmaxR = 1.0f - softmaxL;

      const unsigned char outAlpha = max(topColor[3], baseColor[3]);
      mergedImage.at<Vec4b>(y, x) = Vec4b(
        float(baseColor[0]) * lerp(alphaL, softmaxL, deghostCoef) + float(topColor[0]) * lerp(alphaR, softmaxR, deghostCoef),
        float(baseColor[1]) * lerp(alphaL, softmaxL, deghostCoef) + float(topColor[1]) * lerp(alphaR, softmaxR, deghostCoef),
        float(baseColor[2]) * lerp(alphaL, softmaxL, deghostCoef) + float(topColor[2]) * lerp(alphaR, softmaxR, deghostCoef),
        outAlpha);
    }
  }
  return mergedImage;
}

vector<vector<float>> buildColorAdjustmentModel(
    const Mat& targetImage,
    const Mat& imageToAdjust) {

  static const int kSampleRate = 100;
  std::uniform_int_distribution<int> distribution(0, kSampleRate - 1);
  std::default_random_engine engine;
  LOG(INFO) << "building color adjustment model";
  vector<vector<float>> inputs;
  vector<vector<float>> outputs;
  for (int y = 0; y < targetImage.rows; ++y) {
    for (int x = 0; x < targetImage.cols; ++x) {
      Vec4b targetColor = targetImage.at<Vec4b>(y, x);
      Vec4b adjustColor = imageToAdjust.at<Vec4b>(y, x);
      static const int kAlphaThreshold = 250;
      if (targetColor[3] > kAlphaThreshold &&
          adjustColor[3] > kAlphaThreshold &&
          distribution(engine) == 0) {
        vector<float> feature;
        vector<float> target;
        const float dB = adjustColor[0] - targetColor[0];
        const float dG = adjustColor[1] - targetColor[1];
        const float dR = adjustColor[2] - targetColor[2];
        feature.push_back(1.0f);
        feature.push_back(adjustColor[0] / 255.0f);
        feature.push_back(adjustColor[1] / 255.0f);
        feature.push_back(adjustColor[2] / 255.0f);
        target.push_back(dB / 255.0f);
        target.push_back(dG / 255.0f);
        target.push_back(dR / 255.0f);
        inputs.push_back(feature);
        outputs.push_back(target);
      }
    }
  }

  static const int kInputDim = 4;
  static const int kOutputDim = 3;
  static const int kNumIterations = 1000;
  static const float kStepSize = 0.01f;
  static const bool kPrintObjective = false;
  return solveLinearRegressionRdToRk(
    kInputDim,
    kOutputDim,
    inputs,
    outputs,
    kNumIterations,
    kStepSize,kPrintObjective);
}

void radialAlphaFade(Mat& img) {
  CHECK_EQ(img.type(), CV_32FC4);
  for (int y = 0; y < img.rows; ++y) {
    for (int x = 0; x < img.cols; ++x) {
      const float dx = x - float(img.cols) / 2.0f;
      const float dy = y - float(img.rows) / 2.0f;
      const float r =
        sqrt(dx * dx + dy * dy) / float(std::min(img.rows, img.cols) / 2.0f);
      const float alpha = max(0.0f, 1.0f - r);
      img.at<Vec4f>(y, x)[3] *= alpha;
    }
  }
}

void topDownAlphaFade(Mat& img) {
  CHECK_EQ(img.type(), CV_32FC4);
  for (int y = 0; y < img.rows; ++y) {
    const float alpha = y / float(img.rows);
    for (int x = 0; x < img.cols; ++x) {
      img.at<Vec4f>(y, x)[3] *= alpha;
    }
  }
}

Mat flattenLayersAlphaSoftmax(
    const vector<Mat>& layers,
    const float softmaxCoef) {

  for (const auto& img : layers) {
    CHECK_EQ(img.type(), CV_32FC4);
  }

  Mat flattened(layers[0].size(), CV_32FC3);
  for (int y = 0; y < flattened.rows; ++y) {
    for (int x = 0; x < flattened.cols; ++x) {
      float sumAlpha = 0.0f;
      Vec3f sumColor(0.0f, 0.0f, 0.0f);
      for (int imgIdx = 0; imgIdx < layers.size(); ++imgIdx) {
        const Vec4f srcColor = layers[imgIdx].at<Vec4f>(y, x);
        const float alpha =
          expf(softmaxCoef * srcColor[3]) - 1.0f;
        sumColor += alpha * Vec3f(srcColor[0], srcColor[1], srcColor[2]);
        sumAlpha += alpha;
      }
      flattened.at<Vec3f>(y, x) = sumColor / sumAlpha;
    }
  }
  return flattened;
}

} // namespace util
} // namespace surround360
