/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <vector>

#include "CvUtil.h"

namespace surround360 {

using namespace cv;
using namespace std;

// loads the two bottom camera images. generates an alpha channel corresponding to pole
// masks. does optical flow to merge the two images. results are saved to bottomImage.
void combineBottomImagesWithPoleRemoval(
  const string& imagesDir,
  const string& frameNumber,
  const string& poleMaskDir,
  const string& prevFrameDataDir,
  const string& outputDataDir,
  const bool saveDebugImages,
  const bool saveFlowDataForNextFrame,
  const string& flowAlgName,
  const int alphaFeatherSize,
  const string& bottomCamId,
  const string& bottomCam2Id,
  const float bottomCamUsablePixelsRadius,
  const float bottomCam2UsablePixelsRadius,
  const bool flip180,
  Mat& bottomImage);

} // namespace surround360
