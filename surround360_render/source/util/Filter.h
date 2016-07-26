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
      buffer.at<Vec3f>(hBoundary(j-1, lpImage.cols),0) = v;
    }

    // Anticausal pass
    v = buffer.at<Vec3f>(0, 0);
    for (int j = lpImage.cols-1; j >= 0; j--) {
      Vec3f ip(buffer.at<Vec3f>(wrap(j, lpImage.cols), 0));
      v = lerp(ip, v, alpha);
      lpImage.at<P>(i, hBoundary(j+1, lpImage.cols))[0] = clamp(v[0], 0.0f, maxVal);
      lpImage.at<P>(i, hBoundary(j+1, lpImage.cols))[1] = clamp(v[1], 0.0f, maxVal);
      lpImage.at<P>(i, hBoundary(j+1, lpImage.cols))[2] = clamp(v[2], 0.0f, maxVal);
    }
  }

  // Vertical pass
  for (int j = 0; j < lpImage.cols; j++) {
    // Causal pass
    Vec3f v(lpImage.at<P>(1,j));
    for (int i = 0; i < lpImage.rows; ++i) {
      Vec3f ip(lpImage.at<P>(i,j));
      v = lerp(ip, v, alpha);
      buffer.at<Vec3f>(vBoundary(i-1, lpImage.rows),0) = v;
    }
    // Anticausal pass
    v = buffer.at<Vec3f>(lpImage.rows-2,0);
    for (int i = lpImage.rows-1; i >= -1; i--) {
      Vec3f ip = buffer.at<Vec3f>(reflect(i, lpImage.rows),0);
      v = lerp(ip, v, alpha);
      lpImage.at<P>(vBoundary(i+1, lpImage.rows),j)[0] = clamp(v[0], 0.0f, maxVal);
      lpImage.at<P>(vBoundary(i+1, lpImage.rows),j)[1] = clamp(v[1], 0.0f, maxVal);
      lpImage.at<P>(vBoundary(i+1, lpImage.rows),j)[2] = clamp(v[2], 0.0f, maxVal);
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
    const float maxVal = 255.0f) {

  for (int i = 0; i < inputImage.rows; ++i) {
    for (int j = 0; j < inputImage.cols; ++j) {
      const Vec3f lp = lpImage.at<P>(i, j);
      P& p = inputImage.at<P>(i, j);
      p[0] = clamp(lp[0] + (float(p[0]) - lp[0]) * rAmount,  0.0f, maxVal);
      p[1] = clamp(lp[1] + (float(p[1]) - lp[1]) * gAmount,  0.0f, maxVal);
      p[2] = clamp(lp[2] + (float(p[2]) - lp[2]) * bAmount,  0.0f, maxVal);
    }
  }
}

template <typename P>
inline void sharpenWithIirLowPass(
    Mat& inputImage,
    const Mat& lpImage,
    const float amount) {

  sharpenWithIirLowPass<P>(inputImage, lpImage, amount, amount, amount);
}

}
}

