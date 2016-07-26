/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <string>

#include "PixFlow.h"
#include "VrCamException.h"

namespace surround360 {
namespace optical_flow {

using namespace std;
using namespace cv;

static OpticalFlowInterface* makeOpticalFlowByName(const string flowAlgName) {

  if (flowAlgName == "pixflow_low") {
    static const float kPyrScaleFactor                  = 0.9f;
    static const float kSmoothnessCoef                  = 0.001f;
    static const float kVerticalRegularizationCoef      = 0.01f;
    static const float kHorizontalRegularizationCoef    = 0.01f;
    static const float kGradientStepSize                = 0.5f;
    static const float kDownscaleFactor                 = 0.5f;
    static const float kDirectionalRegularizationCoef   = 0.0f;
    return new PixFlowWithoutDirectionalRegularization(
      kPyrScaleFactor,
      kSmoothnessCoef,
      kVerticalRegularizationCoef,
      kHorizontalRegularizationCoef,
      kGradientStepSize,
      kDownscaleFactor,
      kDirectionalRegularizationCoef
    );
  }

  if (flowAlgName == "pixflow_med") {
    static const float kPyrScaleFactor                  = 0.95f;
    static const float kSmoothnessCoef                  = 0.0001f;
    static const float kVerticalRegularizationCoef      = 0.03f;
    static const float kHorizontalRegularizationCoef    = 0.03f;
    static const float kGradientStepSize                = 0.95f;
    static const float kDownscaleFactor                 = 0.5f;
    static const float kDirectionalRegularizationCoef   = 0.1f;
    return new PixFlowWithDirectionalRegularization(
      kPyrScaleFactor,
      kSmoothnessCoef,
      kVerticalRegularizationCoef,
      kHorizontalRegularizationCoef,
      kGradientStepSize,
      kDownscaleFactor,
      kDirectionalRegularizationCoef
    );
  }

  if (flowAlgName == "pixflow_ultra") {
    static const float kPyrScaleFactor                  = 0.97f;
    static const float kSmoothnessCoef                  = 0.00005f;
    static const float kVerticalRegularizationCoef      = 0.02f;
    static const float kHorizontalRegularizationCoef    = 0.02f;
    static const float kGradientStepSize                = 1.0f;
    static const float kDownscaleFactor                 = 1.0f;
    static const float kDirectionalRegularizationCoef   = 0.1f;
    return new PixFlowWithDirectionalRegularization(
      kPyrScaleFactor,
      kSmoothnessCoef,
      kVerticalRegularizationCoef,
      kHorizontalRegularizationCoef,
      kGradientStepSize,
      kDownscaleFactor,
      kDirectionalRegularizationCoef
    );
  }

  throw VrCamException("unrecognized flow algorithm name: " + flowAlgName);
}

} // namespace optical_flow
} // namespace surround360
