/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <algorithm>
#include <cmath>

#include "KeypointMatchers.h"
#include "MathUtil.h"
#include "OpticalFlowInterface.h"
#include "OpticalFlowVisualization.h"
#include "CvUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace surround360 {
namespace optical_flow {

using namespace surround360::math_util;
using namespace cv;
using namespace cv::detail;

template <
  bool UseDirectionalRegularization,
  int MaxPercentage = 0 // how far to look when initializing flow
>
struct PixFlow : public OpticalFlowInterface {

  static constexpr int kPyrMinImageSize               = 24;
  static constexpr int kPyrMaxLevels                  = 1000;
  static constexpr float kGradEpsilon                 = 0.001f; // for finite differences
  static constexpr float kUpdateAlphaThreshold        = 0.9f;   // pixels with alpha below this aren't updated by proposals
  static constexpr int kMedianBlurSize                = 5;      // medianBlur max size is 5 pixels for CV_32FC2
  static constexpr int kPreBlurKernelWidth            = 5;
  static constexpr float kPreBlurSigma                = 0.25f;  // amount to blur images before pyramids
  static constexpr int kFinalFlowBlurKernelWidth      = 3;
  static constexpr float kFinalFlowBlurSigma          = 1.0f;   // blur that is applied to flow at the end after upscaling
  static constexpr int kGradientBlurKernelWidth       = 3;
  static constexpr float kGradientBlurSigma           = 0.5f;   // amount to blur image gradients
  static constexpr int kBlurredFlowKernelWidth        = 15;     // for regularization/smoothing/diffusion
  static constexpr float kBlurredFlowSigma            = 8.0f;

  const float pyrScaleFactor;
  const float smoothnessCoef;
  const float verticalRegularizationCoef;
  const float horizontalRegularizationCoef;
  const float gradientStepSize;
  const float downscaleFactor;
  const float directionalRegularizationCoef;

  PixFlow(
      const float pyrScaleFactor,
      const float smoothnessCoef,
      const float verticalRegularizationCoef,
      const float horizontalRegularizationCoef,
      const float gradientStepSize,
      const float downscaleFactor,
      const float directionalRegularizationCoef) :
  pyrScaleFactor(pyrScaleFactor),
  smoothnessCoef(smoothnessCoef),
  verticalRegularizationCoef(verticalRegularizationCoef),
  horizontalRegularizationCoef(horizontalRegularizationCoef),
  gradientStepSize(gradientStepSize),
  downscaleFactor(downscaleFactor),
  directionalRegularizationCoef(directionalRegularizationCoef) {}

  ~PixFlow() {}

  // these will be modified when running computeOpticalFlow. it becomes true if prevFlow
  // is non-empty.
  bool usePrevFlowTemporalRegularization = false;

  void computeOpticalFlow(
      const Mat& rgba0byte,
      const Mat& rgba1byte,
      const Mat& prevFlow,
      const Mat& prevI0BGRA,
      const Mat& prevI1BGRA,
      Mat& flow,
      DirectionHint hint) {

    assert(prevFlow.dims == 0 || prevFlow.size() == rgba0byte.size());

    // pre-scale everything to a smaller size. this should be faster + more stable
    Mat rgba0byteDownscaled, rgba1byteDownscaled, prevFlowDownscaled;
    Mat prevI0BGRADownscaled, prevI1BGRADownscaled;
    cv::Size originalSize = rgba0byte.size();
    cv::Size downscaleSize(
      rgba0byte.cols * downscaleFactor, rgba0byte.rows * downscaleFactor);
    resize(rgba0byte, rgba0byteDownscaled, downscaleSize, 0, 0, CV_INTER_CUBIC);
    resize(rgba1byte, rgba1byteDownscaled, downscaleSize, 0, 0, CV_INTER_CUBIC);
    Mat motion = Mat(downscaleSize, CV_32F);
    if (prevFlow.dims > 0) {
      usePrevFlowTemporalRegularization = true;
      resize(prevFlow, prevFlowDownscaled, downscaleSize, 0, 0, CV_INTER_CUBIC);
      prevFlowDownscaled *= float(prevFlowDownscaled.rows) / float(prevFlow.rows);

      resize(prevI0BGRA, prevI0BGRADownscaled, downscaleSize, 0, 0, CV_INTER_CUBIC);
      resize(prevI1BGRA, prevI1BGRADownscaled, downscaleSize, 0, 0, CV_INTER_CUBIC);

      // do motion detection vs. previous frame's images
      for (int y = 0; y < rgba0byteDownscaled.rows; ++y) {
        for (int x = 0; x < rgba0byteDownscaled.cols; ++x) {
          motion.at<float>(y, x) =
            (fabs(rgba1byteDownscaled.at<Vec4b>(y, x)[0] - prevI1BGRADownscaled.at<Vec4b>(y, x)[0]) +
             fabs(rgba1byteDownscaled.at<Vec4b>(y, x)[1] - prevI1BGRADownscaled.at<Vec4b>(y, x)[1]) +
             fabs(rgba1byteDownscaled.at<Vec4b>(y, x)[2] - prevI1BGRADownscaled.at<Vec4b>(y, x)[2])) / (255.0f * 3.0f);
        }
      }
    }

    // convert to various color spaces
    Mat I0Grey, I1Grey, I0, I1, alpha0, alpha1;
    vector<Mat> channels0, channels1;
    split(rgba0byteDownscaled, channels0);
    split(rgba1byteDownscaled, channels1);
    cvtColor(rgba0byteDownscaled, I0Grey,  CV_BGRA2GRAY);
    cvtColor(rgba1byteDownscaled, I1Grey,  CV_BGRA2GRAY);

    I0Grey.convertTo(I0, CV_32F);
    I1Grey.convertTo(I1, CV_32F);
    I0 /= 255.0f;
    I1 /= 255.0f;
    channels0[3].convertTo(alpha0, CV_32F);
    channels1[3].convertTo(alpha1, CV_32F);
    alpha0 /= 255.0f;
    alpha1 /= 255.0f;

    GaussianBlur(I0, I0, Size(kPreBlurKernelWidth, kPreBlurKernelWidth), kPreBlurSigma);
    GaussianBlur(I1, I1, Size(kPreBlurKernelWidth, kPreBlurKernelWidth), kPreBlurSigma);

    vector<Mat> pyramidI0       = buildPyramid(I0);
    vector<Mat> pyramidI1       = buildPyramid(I1);
    vector<Mat> pyramidAlpha0   = buildPyramid(alpha0);
    vector<Mat> pyramidAlpha1   = buildPyramid(alpha1);
    vector<Mat> prevFlowPyramid = buildPyramid(prevFlowDownscaled);
    vector<Mat> motionPyramid   = buildPyramid(motion);

    if (usePrevFlowTemporalRegularization) {
      // rescale the previous flow values at each level of the pyramid
      for (int level = 0; level < prevFlowPyramid.size(); ++level) {
        prevFlowPyramid[level] *=
          float(prevFlowPyramid[level].rows) / float(prevFlowPyramid[0].rows);
      }
    }
    flow = Mat();

    for (int level = pyramidI0.size() - 1; level >= 0; --level) {
      patchMatchPropagationAndSearch(
        pyramidI0[level],
        pyramidI1[level],
        pyramidAlpha0[level],
        pyramidAlpha1[level],
        flow,
        hint);

      if (usePrevFlowTemporalRegularization) {
        adjustFlowTowardPrevious(prevFlowPyramid[level], motionPyramid[level], flow);
      }

      if (level > 0) { // scale the flow up to the next size
        resize(flow, flow, pyramidI0[level - 1].size(), 0, 0, CV_INTER_CUBIC);
        flow *= (1.0f / pyrScaleFactor);
      }
    }

    // scale the flow result back to full size
    resize(flow, flow, originalSize, 0, 0, CV_INTER_LINEAR);
    flow *= (1.0f / downscaleFactor);
    GaussianBlur(
      flow,
      flow,
      Size(kFinalFlowBlurKernelWidth, kFinalFlowBlurKernelWidth),
      kFinalFlowBlurSigma);
  }

  void adjustFlowTowardPrevious(const Mat& prevFlow, const Mat& motion, Mat& flow) {
    for (int y = 0; y < flow.rows; ++y) {
      for (int x = 0; x < flow.cols; ++x) {
        const float w = 1.0f - motion.at<float>(y, x);
        flow.at<Point2f>(y, x) =
          flow.at<Point2f>(y, x) * (1.0f - w) + prevFlow.at<Point2f>(y, x) * w;
      }
    }
  }

  inline Point2f errorGradient(
      const Mat& I0,
      const Mat& I1,
      const Mat& alpha0,
      const Mat& alpha1,
      const Mat& I0x,
      const Mat& I0y,
      const Mat& I1x,
      const Mat& I1y,
      const int x,
      const int y,
      const Mat& flow,
      const Mat& blurredFlow,
      const float currErr) {

    const static Point2f dx(kGradEpsilon, 0.0f);
    const static Point2f dy(0.0f, kGradEpsilon);

    const float fx = errorFunction(I0, I1, alpha0, alpha1, I0x, I0y, I1x, I1y, x, y, flow, blurredFlow, flow.at<Point2f>(y, x) + dx);
    const float fy = errorFunction(I0, I1, alpha0, alpha1, I0x, I0y, I1x, I1y, x, y, flow, blurredFlow, flow.at<Point2f>(y, x) + dy);

    return Point2f((fx - currErr) / kGradEpsilon, (fy - currErr) / kGradEpsilon);
  }

  static inline int computeSearchDistance() {
    // we search a fraction of the coarsest pyramid level
    return (kPyrMinImageSize * MaxPercentage + 50) / 100;
  }

  // compare patch at (i0x,i0y) in i0 to patch at (i1x,i1y) in i1
  float computePatchError(
      const Mat& i0, const Mat& alpha0, int i0x, int i0y,
      const Mat& i1, const Mat& alpha1, int i1x, int i1y)
  {
    // compute sum-of-absolute-differences in 5x5 patch
    static const int kPatchRadius = 2;
    float sad = 0;
    float alpha = 0;
    for (int dy = -kPatchRadius; dy <= kPatchRadius; ++dy) {
      const int d0y = i0y + dy;
      if (0 <= d0y && d0y < i0.rows) {
        const int d1y = clamp(i1y + dy, 0, i1.rows - 1);
        for (int dx = -kPatchRadius; dx <= kPatchRadius; ++dx) {
          const int d0x = i0x + dx;
          if (0 <= d0x && d0x < i0.cols) {
            const int d1x = clamp(i1x + dx, 0, i1.cols - 1);
            const float difference =
              i0.at<float>(d0y, d0x) -
              i1.at<float>(d1y, d1x);
            sad += std::abs(difference);
            alpha +=
              alpha0.at<float>(d0y, d0x) *
              alpha1.at<float>(d1y, d1x);
          }
        }
      }
    }
    // normalize sad to sum of alphas (fine to go to infinity)
    sad /= alpha;
    // scale sad as flow vector length increases to favor short vectors
    const float length = norm(Point2f(i1x - i0x, i1y - i0y));
    sad *= 1 + length / computeSearchDistance();
    return sad;
  }

  // compute the average intensity ratio of lhs / rhs
  float computeIntensityRatio(
      const Mat& lhs, const Mat& lhsAlpha,
      const Mat& rhs, const Mat& rhsAlpha)
  {
    // just scale by the ratio between the sums, attenuated by alpha
    CHECK_EQ(lhs.size(), rhs.size());
    float sumLhs = 0;
    float sumRhs = 0;
    for (int y = 0; y < lhs.rows; ++y) {
      for (int x = 0; x < lhs.cols; ++x) {
        float alpha = lhsAlpha.at<float>(y, x) * rhsAlpha.at<float>(y, x);
        sumLhs += alpha * lhs.at<float>(y, x);
        sumRhs += alpha * rhs.at<float>(y, x);
      }
    }
    return sumLhs / sumRhs;
  }

  Rect computeSearchBox(DirectionHint hint) {
    // we search a rectangle that is a fraction of the coarsest pyramid level
    const int dist = computeSearchDistance();
    // the rectangle extends ortho to each side of the search direction
    static const int kRatio = 8; // aspect ratio of search box
    const int ortho = (dist + kRatio / 2) / kRatio;
    const int thickness = 2 * ortho + 1;
    switch (hint) {
      // opencv rectangles are left, top, width, height
      case DirectionHint::RIGHT: return Rect(0, -ortho, dist + 1, thickness);
      case DirectionHint::DOWN: return Rect(-ortho, 0, thickness, dist + 1);
      case DirectionHint::LEFT: return Rect(-dist, -ortho, dist + 1, thickness);
      case DirectionHint::UP: return Rect(-ortho, -dist, thickness, dist + 1);
      case DirectionHint::UNKNOWN: break; // silence warning
    }
    LOG(FATAL) << "unexpected direction " << int(hint);
    return Rect();
  }

  void adjustInitialFlow(
      const Mat& I0,
      const Mat& I1,
      const Mat& alpha0,
      const Mat& alpha1,
      Mat& flow,
      const DirectionHint hint) {
    // compute a version of I1 that matches I0's intensity
    // this is basically poor man's color correction
    Mat I1eq = I1 * computeIntensityRatio(I0, alpha0, I1, alpha1);

    // estimate the flow of each pixel in I0 by searching a rectangle
    Rect box = computeSearchBox(hint);
    for (int i0y = 0; i0y < I0.rows; ++i0y) {
      for (int i0x = 0; i0x < I0.cols; ++i0x) {
        if (alpha0.at<float>(i0y, i0x) > kUpdateAlphaThreshold) {
          // create affinity for (0,0) by using fraction of the actual error
          float kFraction = 0.8f; // lower the fraction to increase affinity
          float errorBest = kFraction * computePatchError(
            I0, alpha0, i0x, i0y,
            I1eq, alpha1, i0x, i0y);
          int i1xBest = i0x, i1yBest = i0y;
          // look for better patch in the box
          for (int dy = box.y; dy < box.y + box.height; ++dy) {
            for (int dx = box.x; dx < box.x + box.width; ++dx) {
              int i1x = i0x + dx;
              int i1y = i0y + dy;
              if (0 <= i1x && i1x < I1.cols && 0 <= i1y && i1y < I1.rows) {
                float error = computePatchError(
                  I0, alpha0, i0x, i0y,
                  I1eq, alpha1, i1x, i1y);
                if (errorBest > error) {
                    errorBest = error;
                    i1xBest = i1x;
                    i1yBest = i1y;
                }
              }
            }
          }
          // use the best match
          flow.at<Point2f>(i0y, i0x) = Point2f(i1xBest - i0x, i1yBest - i0y);
        }
      }
    }
  }

  void patchMatchPropagationAndSearch(
      const Mat& I0,
      const Mat& I1,
      const Mat& alpha0,
      const Mat& alpha1,
      Mat& flow,
      DirectionHint hint) {

    // image gradients
    Mat I0x, I0y, I1x, I1y;
    const int kSameDepth = -1; // same depth as source image
    const int kKernelSize = 1;
    Sobel(I0, I0x, kSameDepth, 1, 0, kKernelSize, 1, 0.0f, BORDER_REPLICATE);
    Sobel(I0, I0y, kSameDepth, 0, 1, kKernelSize, 1, 0.0f, BORDER_REPLICATE);
    Sobel(I1, I1x, kSameDepth, 1, 0, kKernelSize, 1, 0.0f, BORDER_REPLICATE);
    Sobel(I1, I1y, kSameDepth, 0, 1, kKernelSize, 1, 0.0f, BORDER_REPLICATE);

    // blur gradients
    const cv::Size kGradientBlurSize(kGradientBlurKernelWidth, kGradientBlurKernelWidth);
    GaussianBlur(I0x, I0x, kGradientBlurSize, kGradientBlurSigma);
    GaussianBlur(I0y, I0y, kGradientBlurSize, kGradientBlurSigma);
    GaussianBlur(I1x, I1x, kGradientBlurSize, kGradientBlurSigma);
    GaussianBlur(I1y, I1y, kGradientBlurSize, kGradientBlurSigma);

    if (flow.empty()) {
      // initialize to all zeros
      flow = Mat::zeros(I0.size(), CV_32FC2);
      // optionally look for a better flow
      if (MaxPercentage > 0 && hint != DirectionHint::UNKNOWN) {
        adjustInitialFlow(I0, I1, alpha0, alpha1, flow, hint);
      }
    }

    // blur flow. we will regularize against this
    Mat blurredFlow;
    GaussianBlur(
      flow,
      blurredFlow,
      cv::Size(kBlurredFlowKernelWidth, kBlurredFlowKernelWidth),
      kBlurredFlowSigma);

    const cv::Size imgSize = I0.size();

    // sweep from top/left
    for (int y = 0; y < imgSize.height; ++y) {
      for (int x = 0; x < imgSize.width; ++x) {
        if (alpha0.at<float>(y, x) > kUpdateAlphaThreshold && alpha1.at<float>(y, x) > kUpdateAlphaThreshold) {
          float currErr = errorFunction(I0, I1, alpha0, alpha1, I0x, I0y, I1x, I1y, x, y, flow, blurredFlow, flow.at<Point2f>(y, x));
          if (x > 0) { proposeFlowUpdate(alpha0, alpha1, I0, I1, I0x, I0y, I1x, I1y, flow, blurredFlow, currErr, x, y, flow.at<Point2f>(y, x - 1)); }
          if (y > 0) { proposeFlowUpdate(alpha0, alpha1, I0, I1, I0x, I0y, I1x, I1y, flow, blurredFlow, currErr, x, y, flow.at<Point2f>(y - 1, x)); }
          flow.at<Point2f>(y, x) -= gradientStepSize * errorGradient(I0, I1, alpha0, alpha1, I0x, I0y, I1x, I1y, x, y, flow, blurredFlow, currErr);
        }
      }
    }
    medianBlur(flow, flow, kMedianBlurSize);

    // sweep from bottom/right
    for (int y = imgSize.height - 1; y >= 0; --y) {
      for (int x = imgSize.width - 1; x >= 0; --x) {
        if (alpha0.at<float>(y, x) > kUpdateAlphaThreshold && alpha1.at<float>(y, x) > kUpdateAlphaThreshold) {
          float currErr = errorFunction(I0, I1, alpha0, alpha1, I0x, I0y, I1x, I1y, x, y, flow, blurredFlow, flow.at<Point2f>(y, x));
          if (x < imgSize.width - 1)  { proposeFlowUpdate(alpha0, alpha1, I0, I1, I0x, I0y, I1x, I1y, flow, blurredFlow, currErr, x, y, flow.at<Point2f>(y, x + 1)); }
          if (y < imgSize.height - 1) { proposeFlowUpdate(alpha0, alpha1, I0, I1, I0x, I0y, I1x, I1y, flow, blurredFlow, currErr, x, y, flow.at<Point2f>(y + 1, x)); }
          flow.at<Point2f>(y, x) -= gradientStepSize * errorGradient(I0, I1, alpha0, alpha1, I0x, I0y, I1x, I1y, x, y, flow, blurredFlow, currErr);
        }
      }
    }
    medianBlur(flow, flow, kMedianBlurSize);
    lowAlphaFlowDiffusion(alpha0, alpha1, flow);
  }

  inline void proposeFlowUpdate(
      const Mat& alpha0,
      const Mat& alpha1,
      const Mat& I0,
      const Mat& I1,
      const Mat& I0x,
      const Mat& I0y,
      const Mat& I1x,
      const Mat& I1y,
      Mat& flow,
      const Mat& blurredFlow,
      float& currErr,
      const int updateX, const int updateY,
      const Point2f& proposedFlow) {

    const float proposalErr = errorFunction(I0, I1, alpha0, alpha1, I0x, I0y, I1x, I1y, updateX, updateY, flow, blurredFlow, proposedFlow);
    if (proposalErr < currErr) {
      flow.at<Point2f>(updateY, updateX) = proposedFlow;
      currErr = proposalErr;
    }
  }

  void lowAlphaFlowDiffusion(const Mat& alpha0, const Mat& alpha1, Mat& flow) {
    Mat blurredFlow;
    GaussianBlur(
      flow,
      blurredFlow,
      Size(kBlurredFlowKernelWidth, kBlurredFlowKernelWidth),
      kBlurredFlowSigma);
    for (int y = 0; y < flow.rows; ++y) {
      for (int x = 0; x < flow.cols; ++x) {
        const float a0 = alpha0.at<float>(y, x);
        const float a1 = alpha1.at<float>(y, x);
        const float diffusionCoef = 1.0f - a0 * a1;
        flow.at<Point2f>(y, x) =
          diffusionCoef * blurredFlow.at<Point2f>(y, x)
          + (1.0f - diffusionCoef) * flow.at<Point2f>(y, x);
      }
    }
  }

  // use only with greyscale 32F Mat images
  static inline float getPixBilinear32FExtend(const Mat& img, float x, float y) {
    const cv::Size& imgSize = img.size();
    x                 = min(imgSize.width - 2.0f, max(0.0f, x));
    y                 = min(imgSize.height - 2.0f, max(0.0f, y));
    const int x0      = int(x);
    const int y0      = int(y);
    const float xR    = x - float(x0);
    const float yR    = y - float(y0);
    const float* p    = img.ptr<float>(y0);
    const float f00   = *(p + x0);
    const float f01   = *(p + x0 + img.cols);
    const float f10   = *(p + x0 + 1);
    const float f11   = *(p + x0 + img.cols + 1);
    const float a1    = f00;
    const float a2    = f10 - f00;
    const float a3    = f01 - f00;
    const float a4    = f00 + f11 - f10 - f01;
    return a1 + a2 * xR + a3 * yR + a4 * xR * yR;
  }

  vector<Mat> buildPyramid(const Mat& src) {
    vector<Mat> pyramid = {src};
    while (pyramid.size() < kPyrMaxLevels) {
      Size newSize(
        pyramid.back().cols * pyrScaleFactor + 0.5f,
        pyramid.back().rows * pyrScaleFactor + 0.5f);
      if (newSize.height <= kPyrMinImageSize || newSize.width <= kPyrMinImageSize) {
        break;
      }
      Mat scaledImage;
      resize(pyramid.back(), scaledImage, newSize, 0, 0, CV_INTER_LINEAR);
      pyramid.push_back(scaledImage);
    }
    return pyramid;
  }

  inline float errorFunction(
      const Mat& I0,
      const Mat& I1,
      const Mat& alpha0,
      const Mat& alpha1,
      const Mat& I0x,
      const Mat& I0y,
      const Mat& I1x,
      const Mat& I1y,
      const int x,
      const int y,
      const Mat& flow,
      const Mat& blurredFlow,
      const Point2f& flowDir) {

    const float matchX      = x + flowDir.x;
    const float matchY      = y + flowDir.y;
    const float i0x         = I0x.at<float>(y, x);
    const float i0y         = I0y.at<float>(y, x);
    const float i1x         = getPixBilinear32FExtend(I1x, matchX, matchY);
    const float i1y         = getPixBilinear32FExtend(I1y, matchX, matchY);
    const Point2f flowDiff  = blurredFlow.at<Point2f>(y, x) - flowDir;
    const float smoothness  = sqrtf(flowDiff.dot(flowDiff));

    float err = sqrtf((i0x - i1x) * (i0x - i1x) + (i0y - i1y) * (i0y - i1y))
      + smoothness * smoothnessCoef
      + verticalRegularizationCoef * fabsf(flowDir.y) / float(I0.cols)
      + horizontalRegularizationCoef * fabsf(flowDir.x) / float(I0.rows);

    if (UseDirectionalRegularization) {
      Point2f bf = blurredFlow.at<Point2f>(y, x);
      const float blurredFlowMag = sqrtf(bf.dot(bf));
      const static float kEpsilon = 0.001f;
      bf /= blurredFlowMag + kEpsilon;
      const float flowMag = sqrtf(flowDir.dot(flowDir));
      Point2f normalizedFlowDir = flowDir / (flowMag + kEpsilon);
      const float dot = bf.dot(normalizedFlowDir);
      err -= directionalRegularizationCoef * dot;
    }

    return err;
  }
};

using PixFlowWithDirectionalRegularization = PixFlow<true>;
using PixFlowWithoutDirectionalRegularization = PixFlow<false>;

} // namespace optical_flow
} // namespace surround360
