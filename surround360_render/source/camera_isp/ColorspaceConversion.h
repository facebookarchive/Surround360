/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/
#pragma once

#include "opencv2/imgproc.hpp"

#include "MathUtil.h"

namespace surround360 {
namespace color {

using namespace cv;
using namespace std;
using namespace surround360::math_util;

static float kRgb2yuvData[9] = {
  0.299f,     0.587f,     0.114f,
  -0.14713f,  -0.28886f,  0.436f,
  0.615f,     -0.51499f,  -0.10001f
};

static Mat rgb2yuv(3, 3, CV_32FC1, kRgb2yuvData);

static float kYuv2rgbData[9] = {
  1.0f,     0.0f,         1.13983f,
  1.0f,     -0.39465f,    -0.58060f,
  1.0f,     2.03211f,     0.0f
};

static Mat yuv2rgb(3, 3, CV_32FC1, kYuv2rgbData);

// "dec" is a "decorrelated" color space.  This matrix uses a DCT decomposition
// which is a good approximation to a optimum principle component decorrelation
// basis.

static float kRgb2decData[9] = {
  1.0f/sqrtf(3.0f),  1.0f/sqrtf(2.0f), 1.0f/sqrtf(6.0f),
  1.0f/sqrtf(3.0f),  0.0f,            -2.0f/sqrtf(6.0f),
  1.0f/sqrtf(3.0f), -1.0f/sqrtf(2.0f), 1.0f/sqrtf(6.0f)
};

static Mat rgb2dec(3, 3, CV_32FC1, kRgb2decData);


// Lab power curve
static double pCurve(const double x) {
  return x > 0.008856f ? pow(x , 1.0 / 3.0) : 7.787 * x + 16.0/116.0;
}

void toLab(
    const double r,
    const double g,
    const double b,
    double& L,
    double& A,
    double& B) {
  static bool firstTimeThru = true;
  static const double kD65White[3] = {0.950456, 1, 1.088754};
  double rgbToXyz[3][3] = {
    { 0.412453, 0.357580, 0.180423 },
    { 0.212671, 0.715160, 0.072169 },
    { 0.019334, 0.119193, 0.950227 } };

  if (firstTimeThru) {
    firstTimeThru = false;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rgbToXyz[i][j] = rgbToXyz[i][j] / kD65White[i];
      }
    }
  }

  const double x = pCurve(rgbToXyz[0][0] * r + rgbToXyz[0][1] * g  + rgbToXyz[0][2] * b);
  const double y = pCurve(rgbToXyz[1][0] * r + rgbToXyz[1][1] * g  + rgbToXyz[1][2] * b);
  const double z = pCurve(rgbToXyz[2][0] * r + rgbToXyz[2][1] * g  + rgbToXyz[2][2] * b);

  L = 116.0 * y - 16.0;
  A = 500.0 * (x - y);
  B = 200.0 * (y - z);
}
} // end namespace color
} // end namespace surround360
