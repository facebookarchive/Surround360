/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <mutex>
#include <vector>

#include "CvUtil.h"
#include "ColorAdjustmentSampleLogger.h"

namespace surround360 {
namespace color_adjust {

using namespace cv;
using namespace std;

vector<double> computeBrightnessAdjustmentsForSideCameras(
    const int numSideCams,
    const vector<ColorSample>& colorSamples) {

  vector<double> brightnessAdjustments(numSideCams, 0.0);
  vector<int> numValsInAvg(numSideCams, 0);

  for (const ColorSample& cs : colorSamples) {
    const double leftImageBrightness =
      (cs.leftImageColor[0] +
      cs.leftImageColor[1] +
      cs.leftImageColor[2]) / 3.0;
    const double rightImageBrightness =
      (cs.rightImageColor[0] +
      cs.rightImageColor[1] +
      cs.rightImageColor[2]) / 3.0;
    const double midBrightness =
      (leftImageBrightness + rightImageBrightness) / 2.0;

    const double adjustLeft  = midBrightness - leftImageBrightness;
    const double adjustRight = midBrightness - rightImageBrightness;

    brightnessAdjustments[cs.leftImageIdx]  += adjustLeft;
    brightnessAdjustments[cs.rightImageIdx] += adjustRight;
    ++numValsInAvg[cs.leftImageIdx];
    ++numValsInAvg[cs.rightImageIdx];
  }

  for (int i = 0; i < numSideCams; ++i) {
    brightnessAdjustments[i] /= double(numValsInAvg[i]);
  }

  return brightnessAdjustments;
}

} // namespace color_adjust
} // namespace surround360
