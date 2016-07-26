/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include "CvUtil.h"

namespace surround360 {
namespace optical_flow {

using namespace cv;

// generate a greyscale visualization of a flow field by interpreting
// just the x-coordinate of the flow as disparity.
Mat visualizeFlowAsGreyDisparity(const Mat& flow);

// generate a visualization of a flow field by overlaying arrows on top
Mat visualizeFlowAsVectorField(const Mat& flow, const Mat& image);

// generate a colored visualization of a flow field where the color indicates
// the direction and magnitude of flow. black for 0 magnitude, increasing brightness
// proportional to magnnitude. uses HSV color space; direction is encoded with hue.
Mat visualizeFlowColorWheel(const Mat& flow);

// generate a legend for the color scheme used in visualizeFlowColorWheel
Mat testColorWheel();

} // namespace optical_flow
} // namespace surround360
