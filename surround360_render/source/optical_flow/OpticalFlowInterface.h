/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include "opencv2/core.hpp"

namespace surround360 {
namespace optical_flow {

using namespace cv;

// interface for an optical flow algorithm
class OpticalFlowInterface {
public:
  virtual ~OpticalFlowInterface() {};

  // compute the flow field that warps image I1 so that it becomes like image I0.
  // I0 and I1 are 1 byte/channel BGRA format, i.e. they have an alpha channel.
  // it may be the case that I0 and I1 are frames in a video sequence, and some form
  // of temporal regularization may be applied to the flow. in this case, prevFlow stores
  // the last frame's flow for the same camera pair, and prevI0BGRA and prevI1BGRA store
  // the previous frame pixel data. note however that all of the prev Mats may be empty,
  // e.g., if this is the first frame of a sequence, or if we are just rendering a photo.
  // implementations may also chose to ignore previous data regardless.
  virtual void computeOpticalFlow(
    const Mat& I0BGRA,
    const Mat& I1BGRA,
    const Mat& prevFlow,
    const Mat& prevI0BGRA,
    const Mat& prevI1BGRA,
    Mat& flow) = 0;
};

} // namespace optical_flow
} // namespace surround360
