/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "NovelView.h"

#include <string>
#include <vector>

#include "OpticalFlowInterface.h"
#include "OpticalFlowFactory.h"
#include "SystemUtil.h"
#include "CvUtil.h"

namespace surround360 {
namespace optical_flow {

using namespace std;
using namespace cv;
using namespace surround360::util;

Mat NovelViewUtil::generateNovelViewSimpleCvRemap(
    const Mat& srcImage,
    const Mat& flow,
    const double t) {

  const int w = srcImage.cols;
  const int h = srcImage.rows;
  Mat warpMap = Mat(Size(w, h), CV_32FC2);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      Point2f flowDir = flow.at<Point2f>(y, x);
      warpMap.at<Point2f>(y, x) =
        Point2f(x + flowDir.x * t, y + flowDir.y * t);
    }
  }
  Mat novelView;
  remap(srcImage, novelView, warpMap, Mat(), CV_INTER_CUBIC);
  return novelView;
}

Mat NovelViewUtil::combineNovelViews(
    const Mat& imageL,
    const float blendL,
    const Mat& imageR,
    const float blendR,
    const Mat& flowLtoR,
    const Mat& flowRtoL) {

  Mat blendImage(imageL.size(), CV_8UC4);
  for (int y = 0; y < imageL.rows; ++y) {
    for (int x = 0; x < imageL.cols; ++x) {
      const Vec4b colorL = imageL.at<Vec4b>(y, x);
      const Vec4b colorR = imageR.at<Vec4b>(y, x);
      Vec4b colorMixed;
      if (colorL[3] == 0 && colorR[3] == 0 ) {
        colorMixed = Vec4b(0, 0, 0, 0);
      } else if (colorL[3] > 0 && colorR[3] == 0) {
        colorMixed = Vec4b(colorL[0], colorL[1], colorL[2], 255);
      } else if (colorL[3] == 0 && colorR[3] > 0) {
        colorMixed = Vec4b(colorR[0], colorR[1], colorR[2], 255);
      } else {
        const Point2f fLR = flowLtoR.at<Point2f>(y, x);
        const Point2f fRL = flowRtoL.at<Point2f>(y, x);
        const float flowMagLR = sqrtf(fLR.x * fLR.x + fLR.y * fLR.y) / float(imageL.cols);
        const float flowMagRL = sqrtf(fRL.x * fRL.x + fRL.y * fRL.y) / float(imageL.cols);
        const float colorDiff =
          (std::abs(colorL[0] - colorR[0]) +
           std::abs(colorL[1] - colorR[1]) +
           std::abs(colorL[2] - colorR[2])) / 255.0f;
        static const float kColorDiffCoef = 10.0f;
        static const float kSoftmaxSharpness = 10.0f;
        static const float kFlowMagCoef = 100.0f; // determines how much we prefer larger flows
        const float deghostCoef = tanhf(colorDiff * kColorDiffCoef);
        const float alphaL = colorL[3] / 255.0f;
        const float alphaR = colorR[3] / 255.0f;
        const double expL =
          exp(kSoftmaxSharpness * blendL * alphaL * (1.0 + kFlowMagCoef * flowMagRL));
        const double expR =
          exp(kSoftmaxSharpness * blendR * alphaR * (1.0 + kFlowMagCoef * flowMagLR));
        const double sumExp = expL + expR + 0.00001;
        const float softmaxL = float(expL / sumExp);
        const float softmaxR = float(expR / sumExp);
        colorMixed = Vec4b(
          float(colorL[0]) * lerp(blendL, softmaxL, deghostCoef) + float(colorR[0]) * lerp(blendR, softmaxR, deghostCoef),
          float(colorL[1]) * lerp(blendL, softmaxL, deghostCoef) + float(colorR[1]) * lerp(blendR, softmaxR, deghostCoef),
          float(colorL[2]) * lerp(blendL, softmaxL, deghostCoef) + float(colorR[2]) * lerp(blendR, softmaxR, deghostCoef),
          255);
      }
      blendImage.at<Vec4b>(y, x) = colorMixed;
    }
  }
  return blendImage;
}

Mat NovelViewUtil::combineLazyViews(
    const Mat& imageL,
    const Mat& imageR,
    const Mat& flowMagL,
    const Mat& flowMagR) {

  Mat blendImage(imageL.size(), CV_8UC4);
  for (int y = 0; y < imageL.rows; ++y) {
    for (int x = 0; x < imageL.cols; ++x) {
      const Vec4b colorL = imageL.at<Vec4b>(y, x);
      const Vec4b colorR = imageR.at<Vec4b>(y, x);

      const unsigned char outAlpha =
        max(colorL[3], colorR[3]) / 255.0f > 0.1 ? 255 : 0;

      Vec4b colorMixed;
      if (colorL[3] == 0 && colorR[3] == 0) {
        colorMixed = Vec4b(0, 0, 0, outAlpha);
      } else if (colorL[3] == 0) {
        colorMixed = Vec4b(colorR[0], colorR[1], colorR[2], outAlpha);
      } else if (colorR[3] == 0) {
        colorMixed = Vec4b(colorL[0], colorL[1], colorL[2], outAlpha);
      } else {
        const float magL = flowMagL.at<float>(y,x) / float(imageL.cols);
        const float magR = flowMagR.at<float>(y,x) / float(imageL.cols);
        float blendL = float(colorL[3]);
        float blendR = float(colorR[3]);
        float norm = blendL + blendR;
        blendL /= norm;
        blendR /= norm;
        const float colorDiff =
          (std::abs(colorL[0] - colorR[0]) +
           std::abs(colorL[1] - colorR[1]) +
           std::abs(colorL[2] - colorR[2])) / 255.0f;
        static const float kColorDiffCoef = 10.0f;
        static const float kSoftmaxSharpness = 10.0f;
        static const float kFlowMagCoef = 20.0f; // NOTE: this is scaled differently than the test version due to normalizing magL & magR by imageL.cols
        const float deghostCoef = tanhf(colorDiff * kColorDiffCoef);
        const double expL = exp(kSoftmaxSharpness * blendL * (1.0 + kFlowMagCoef * magL));
        const double expR = exp(kSoftmaxSharpness * blendR * (1.0 + kFlowMagCoef * magR));
        const double sumExp = expL + expR + 0.00001;
        const float softmaxL = float(expL / sumExp);
        const float softmaxR = float(expR / sumExp);
        colorMixed = Vec4b(
          float(colorL[0]) * lerp(blendL, softmaxL, deghostCoef) + float(colorR[0]) * lerp(blendR, softmaxR, deghostCoef),
          float(colorL[1]) * lerp(blendL, softmaxL, deghostCoef) + float(colorR[1]) * lerp(blendR, softmaxR, deghostCoef),
          float(colorL[2]) * lerp(blendL, softmaxL, deghostCoef) + float(colorR[2]) * lerp(blendR, softmaxR, deghostCoef),
          255);
      }
      blendImage.at<Vec4b>(y, x) = colorMixed;
    }
  }
  return blendImage;
}

void NovelViewGeneratorLazyFlow::generateNovelView(
    const double shiftFromL,
    Mat& outNovelViewMerged,
    Mat& outNovelViewFromL,
    Mat& outNovelViewFromR) {

  outNovelViewFromL = NovelViewUtil::generateNovelViewSimpleCvRemap(
    imageL, flowRtoL, shiftFromL);

  outNovelViewFromR = NovelViewUtil::generateNovelViewSimpleCvRemap(
    imageR, flowLtoR, 1.0 - shiftFromL);

  outNovelViewMerged = NovelViewUtil::combineNovelViews(
    outNovelViewFromL, 1.0 - shiftFromL,
    outNovelViewFromR, shiftFromL,
    flowLtoR, flowRtoL);
}

pair<Mat, Mat> NovelViewGeneratorLazyFlow::renderLazyNovelView(
    const int width,
    const int height,
    const vector<vector<Point3f>>& novelViewWarpBuffer,
    const Mat& srcImage,
    const Mat& opticalFlow,
    const bool invertT) {

  // a composition of remap
  Mat warpOpticalFlow = Mat(Size(width, height), CV_32FC2);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Point3f lazyWarp = novelViewWarpBuffer[x][y];
      warpOpticalFlow.at<Point2f>(y, x) = Point2f(lazyWarp.x, lazyWarp.y);
    }
  }
  Mat remappedFlow;
  remap(opticalFlow, remappedFlow, warpOpticalFlow, Mat(), CV_INTER_CUBIC);

  Mat warpComposition = Mat(Size(width, height), CV_32FC2);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Point3f lazyWarp = novelViewWarpBuffer[x][y];
      Point2f flowDir = remappedFlow.at<Point2f>(y, x);
      // the 3rd coordinate (z) of novelViewWarpBuffer is shift/time value
      const float t = invertT ? (1.0f - lazyWarp.z) : lazyWarp.z;
      warpComposition.at<Point2f>(y, x) =
        Point2f(lazyWarp.x + flowDir.x * t, lazyWarp.y + flowDir.y * t);
    }
  }

  Mat novelView;
  remap(srcImage, novelView, warpComposition, Mat(), CV_INTER_CUBIC);
  Mat novelViewFlowMag(novelView.size(), CV_32F);
  // so far we haven't quite set things up to exactly match the original
  // O(n^3) algorithm. we need to blend the two novel views based on the
  // time shift value. we will pack that into the alpha channel here,
  // then use it to blend the two later.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Point3f lazyWarp = novelViewWarpBuffer[x][y];
      Point2f flowDir = remappedFlow.at<Point2f>(y, x);
      const float t = invertT ? (1.0f - lazyWarp.z) : lazyWarp.z;
      novelView.at<Vec4b>(y, x)[3] =
        int((1.0f - t) * novelView.at<Vec4b>(y, x)[3]);
      novelViewFlowMag.at<float>(y, x) =
        sqrtf(flowDir.x * flowDir.x + flowDir.y * flowDir.y);
    }
  }
  return make_pair(novelView, novelViewFlowMag);
}

pair<Mat, Mat> NovelViewGeneratorLazyFlow::combineLazyNovelViews(
    const LazyNovelViewBuffer& lazyBuffer) {

  // two images for the left eye (to be combined)
  pair<Mat, Mat> leftEyeFromLeft = renderLazyNovelView(
    lazyBuffer.width, lazyBuffer.height,
    lazyBuffer.warpL,
    imageL,
    flowRtoL,
    false);
  pair<Mat, Mat> leftEyeFromRight = renderLazyNovelView(
    lazyBuffer.width, lazyBuffer.height,
    lazyBuffer.warpL,
    imageR,
    flowLtoR,
    true);

  // two images for the right eye (to be combined)
  pair<Mat, Mat> rightEyeFromLeft = renderLazyNovelView(
    lazyBuffer.width, lazyBuffer.height,
    lazyBuffer.warpR,
    imageL,
    flowRtoL,
    false);
  pair<Mat, Mat> rightEyeFromRight = renderLazyNovelView(
    lazyBuffer.width, lazyBuffer.height,
    lazyBuffer.warpR,
    imageR,
    flowLtoR,
    true);

  Mat leftEyeCombined = NovelViewUtil::combineLazyViews(
    leftEyeFromLeft.first,
    leftEyeFromRight.first,
    leftEyeFromLeft.second,
    leftEyeFromRight.second);
  Mat rightEyeCombined = NovelViewUtil::combineLazyViews(
    rightEyeFromLeft.first,
    rightEyeFromRight.first,
    rightEyeFromLeft.second,
    rightEyeFromRight.second);
  return make_pair(leftEyeCombined, rightEyeCombined);
}

void NovelViewGeneratorAsymmetricFlow::prepare(
    const Mat& colorImageL,
    const Mat& colorImageR,
    const Mat& prevFlowLtoR,
    const Mat& prevFlowRtoL,
    const Mat& prevColorImageL,
    const Mat& prevColorImageR) {

  imageL = colorImageL.clone();
  imageR = colorImageR.clone();

  OpticalFlowInterface* flowAlg = makeOpticalFlowByName(flowAlgName);
  flowAlg->computeOpticalFlow(
    imageL, imageR, prevFlowLtoR, prevColorImageL, prevColorImageR, flowLtoR);
  flowAlg->computeOpticalFlow(
    imageR, imageL, prevFlowRtoL, prevColorImageR, prevColorImageL, flowRtoL);
  delete flowAlg;
}

} // namespace reprojection
} // namespace surround360
