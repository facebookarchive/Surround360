/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include "MathUtil.h"

namespace surround360 {
namespace util {

using namespace cv;
using namespace std;
using namespace math_util;

// Boundary functor's
template <typename T>
struct WrapBoundary {
  WrapBoundary() {}
  inline T operator()(const T x, const T r) const {
    return wrap(x, r);
  }
};

template <typename T>
struct ReflectBoundary {
  ReflectBoundary() {}
  inline T operator()(const T x, const T r) const {
    return reflect(x, r);
  }
};

// Implements a two-tap IIR low pass filter
template <typename H, typename V, typename P>
void iirLowPass(
    const Mat inputImage,
    const float amount,
    Mat& lpImage,
    const H&  hBoundary,
    const V&  vBoundary,
    const float maxVal = 255.0f) {

  const float alpha = powf(amount, 1.0f/4.0f);

  const int width = inputImage.cols;
  const int height = inputImage.rows;
  Mat buffer(std::max(width, height), 1, CV_32FC3);
  const Vec3f zf(0, 0, 0);
  assert(width == lpImage.cols && height == lpImage.rows);

  // Horizontal pass
  for (int i = 0; i < height; ++i) {
    // Causal pass
    Vec3f v(inputImage.at<P>(i, 0));
    for (int j = 1; j <= width; ++j) {
      Vec3f ip(inputImage.at<P>(i, hBoundary(j, width)));
      v = lerp(ip, v, alpha);
      buffer.at<Vec3f>(hBoundary(j - 1, width)) = v;
    }

    // Anticausal pass
    for (int j = width - 2; j >= -1; --j) {
      Vec3f ip(buffer.at<Vec3f>(hBoundary(j, width)));
      v = lerp(ip, v, alpha);
      lpImage.at<P>(i, j + 1)[0] = clamp(v[0], 0.0f, maxVal);
      lpImage.at<P>(i, j + 1)[1] = clamp(v[1], 0.0f, maxVal);
      lpImage.at<P>(i, j + 1)[2] = clamp(v[2], 0.0f, maxVal);
    }
  }

  // Vertical pass
  for (int j = 0; j < width; ++j) {
    // Causal pass
    Vec3f v(lpImage.at<P>(0, j));
    for (int i = 1; i <= height; ++i) {
      Vec3f ip(lpImage.at<P>(vBoundary(i, height), j));
      v = lerp(ip, v, alpha);
      buffer.at<Vec3f>(vBoundary(i - 1, height)) = v;
    }
    // Anticausal pass
    for (int i = height - 2; i >= -1; --i) {
      Vec3f ip = buffer.at<Vec3f>(vBoundary(i, height));
      v = lerp(ip, v, alpha);
      lpImage.at<P>(i + 1, j)[0] = clamp(v[0], 0.0f, maxVal);
      lpImage.at<P>(i + 1, j)[1] = clamp(v[1], 0.0f, maxVal);
      lpImage.at<P>(i + 1, j)[2] = clamp(v[2], 0.0f, maxVal);
    }
  }
}

template <typename P>
void sharpenWithIirLowPass(
    Mat& inputImage,
    const Mat& lpImage,
    const float rAmount,
    const float gAmount,
    const float bAmount,
    const float noiseCore = 100.0f,
    const float maxVal = 255.0f) {
  // Iir unsharp mask with noise coring
  for (int i = 0; i < inputImage.rows; ++i) {
    for (int j = 0; j < inputImage.cols; ++j) {
      const Vec3f lp = lpImage.at<P>(i, j);
      P& p = inputImage.at<P>(i, j);
      // High pass signal - just the residual of the low pass
      // subtracted from the original signal.
      const Vec3f hp(
          p[0] - lp[0],
          p[1] - lp[1],
          p[2] - lp[2]);
      // Noise coring
      const Vec3f ng(
          1.0f - expf(-(square(hp[0]) * noiseCore)),
          1.0f - expf(-(square(hp[1]) * noiseCore)),
          1.0f - expf(-(square(hp[2]) * noiseCore)));
      // Unsharp mask with coring
      p[0] = clamp(lp[0] + hp[0] * ng[0] * rAmount,  0.0f, maxVal);
      p[1] = clamp(lp[1] + hp[1] * ng[1] * gAmount,  0.0f, maxVal);
      p[2] = clamp(lp[2] + hp[2] * ng[2] * bAmount,  0.0f, maxVal);
    }
  }
}

template <typename P>
inline void sharpenWithIirLowPass(
    Mat& inputImage,
    const Mat& lpImage,
    const float amount,
    const float noiseCore = 100.0f) {

  sharpenWithIirLowPass<P>(inputImage, lpImage, amount, amount, amount, noiseCore);
}

}
}
