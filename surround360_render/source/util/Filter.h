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

  Mat buffer(max(inputImage.rows, inputImage.cols), 1, CV_32FC3);

  // Horizontal pass
  for (int i = 0; i < lpImage.rows; ++i) {
    // Causal pass
    Vec3f v(inputImage.at<P>(i, lpImage.cols-1));
    for (int j = 0; j < lpImage.cols; j++) {
      Vec3f ip(inputImage.at<P>(i,j));
      v = lerp(ip, v, alpha);
      buffer.at<Vec3f>(hBoundary(j-1, lpImage.cols), 0) = v;
    }

    // Anticausal pass
    v = buffer.at<Vec3f>(i, 0);
    for (int j = lpImage.cols-1; j >= 0; j--) {
      Vec3f ip(buffer.at<Vec3f>(wrap(j, lpImage.cols), 0));
      v = lerp(ip, v, alpha);
      lpImage.at<P>(i, hBoundary(j+1, lpImage.cols)) = v;
    }
  }

  // Vertical pass
  for (int j = 0; j < lpImage.cols; j++) {
    // Causal pass
    Vec3f v(lpImage.at<P>(1,j));
    for (int i = 0; i < lpImage.rows; ++i) {
      Vec3f ip(lpImage.at<P>(i,j));
      v = lerp(ip, v, alpha);
      buffer.at<Vec3f>(vBoundary(i-1, lpImage.rows), 0) = v;
    }
    // Anticausal pass
    v = buffer.at<Vec3f>(lpImage.rows-2, 0);
    for (int i = lpImage.rows-1; i >= -1; i--) {
      Vec3f ip = buffer.at<Vec3f>(reflect(i, lpImage.rows), 0);
      v = lerp(ip, v, alpha);
      lpImage.at<P>(vBoundary(i+1, lpImage.rows),j) = v;
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
      P hp;
      hp[0] = p[0] - lp[0];
      hp[1] = p[1] - lp[1];
      hp[2] = p[2] - lp[2];
      // Noise coring
      P ng;
      ng[0] = 1.0f - expf(-(square(hp[0]) * noiseCore));
      ng[1] = 1.0f - expf(-(square(hp[1]) * noiseCore));
      ng[2] = 1.0f - expf(-(square(hp[2]) * noiseCore));

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
