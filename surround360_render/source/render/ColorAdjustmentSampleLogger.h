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

namespace surround360 {
namespace color_adjust {

using namespace cv;
using namespace std;

struct ColorSample {
  int leftImageIdx, rightImageIdx;
  Vec4b leftImageColor, rightImageColor;
};


// the purpose of this class is to log samples of colors that will be blended
// during novel view synthesis. the blending happens in multiple threads, so
// we need to do the logging in a thread-safe way. it is OK to subsample.
class ColorAdjustmentSampleLogger {
 public:
  mutex samplesMutex;
  vector<ColorSample> samples;
  bool enabled;

  static ColorAdjustmentSampleLogger& instance() {
    static ColorAdjustmentSampleLogger* inst =
      new ColorAdjustmentSampleLogger();
    return *inst;
  }

  inline void addSample(
      const int& leftImageIdx,
      const int& rightImageIdx,
      const Vec4b& leftImageColor,
      const Vec4b& rightImageColor) {

    if (!enabled) {
      return;
    }

    static const int kSampleRate = 1000; // keep only one over this # samples
    if (rand() % kSampleRate == 0) {
      ColorSample sample;
      sample.leftImageIdx = leftImageIdx;
      sample.rightImageIdx = rightImageIdx;
      sample.leftImageColor = leftImageColor;
      sample.rightImageColor = rightImageColor;

      samplesMutex.lock();
      samples.push_back(sample);
      samplesMutex.unlock();
    }
  }

 private:
  ColorAdjustmentSampleLogger() {
    enabled = true;
  }
};

} // namespace color_adjust
} // namespace surround360
