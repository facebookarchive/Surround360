/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "PoleRemoval.h"

#include <string>
#include <vector>

#include "CvUtil.h"
#include "MathUtil.h"
#include "OpticalFlowFactory.h"
#include "OpticalFlowInterface.h"
#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace surround360 {

using namespace cv;
using namespace std;
using namespace surround360::calibration;
using namespace surround360::optical_flow;
using namespace surround360::util;

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
    Mat& bottomImage) {

  const string cameraDir = imagesDir + "/" + bottomCamId;
  const string camera2Dir = imagesDir + "/" + bottomCam2Id;
  const string bottomImageFilename = frameNumber + ".png";
  const string bottomImagePath = cameraDir + "/" + bottomImageFilename;
  const string bottomImagePath2 = camera2Dir + "/" + bottomImageFilename;
  bottomImage = imreadExceptionOnFail(bottomImagePath, CV_LOAD_IMAGE_COLOR);
  Mat bottomImage2 = imreadExceptionOnFail(bottomImagePath2, CV_LOAD_IMAGE_COLOR);
  const string poleMaskPath = poleMaskDir + "/" + bottomCamId + ".png";
  const string poleMaskPath2 = poleMaskDir + "/" + bottomCam2Id + ".png";
  Mat bottomRedMask = imreadExceptionOnFail(poleMaskPath, 1);
  Mat bottomRedMask2 = imreadExceptionOnFail(poleMaskPath2, 1);
  if (bottomRedMask.rows == 0 ||
      bottomRedMask.cols == 0 ||
      bottomRedMask2.rows == 0 ||
      bottomRedMask2.cols == 0) {
    throw VrCamException(
      "missing or bad pole mask:" + poleMaskPath + "," + poleMaskPath2);
  }

  // make alpha channels from usable radius
  cvtColor(bottomImage, bottomImage, CV_BGR2BGRA);
  cvtColor(bottomImage2, bottomImage2, CV_BGR2BGRA);
  circleAlphaCut(bottomImage, bottomCamUsablePixelsRadius);
  circleAlphaCut(bottomImage2, bottomCam2UsablePixelsRadius);

  // cut out red masks from alpha channel
  cutRedMaskOutOfAlphaChannel(bottomImage, bottomRedMask);
  cutRedMaskOutOfAlphaChannel(bottomImage2, bottomRedMask2);

  // feather the alpha channel to make it a smoother transition (helps flow)
  bottomImage = featherAlphaChannel(bottomImage, alphaFeatherSize);
  bottomImage2 = featherAlphaChannel(bottomImage2, alphaFeatherSize);

  // rotate the second bottom camera's image 180 degree
  if (flip180) {
    flip(bottomImage2, bottomImage2, -1);
  }

  // do optical flow
  VLOG(1) << "Doing optical flow to merge bottom camera images";
  Mat prevFrameBottomPoleRemovalFlow = Mat();
  Mat prevBottomImage = Mat();
  Mat prevBottomImage2 = Mat();
  if (prevFrameDataDir != "NONE") {
    VLOG(1) << "Reading previous frame flow for bottom-secondary camera: "
      << prevFrameDataDir;

    const string flowPrevDir = outputDataDir + "/flow/" + prevFrameDataDir;
    const string flowImagesPrevDir =
      outputDataDir + "/debug/" + prevFrameDataDir + "/flow_images";

    prevFrameBottomPoleRemovalFlow = readFlowFromFile(
      flowPrevDir + "/flow_bottom_secondary.bin");
    prevBottomImage = imreadExceptionOnFail(
      flowImagesPrevDir + "/bottomImage.png", -1);
    prevBottomImage2 = imreadExceptionOnFail(
      flowImagesPrevDir + "/bottomImage2.png", -1);
  }

  OpticalFlowInterface* flowAlg = makeOpticalFlowByName(flowAlgName);
  Mat flow;
  flowAlg->computeOpticalFlow(
    bottomImage,
    bottomImage2,
    prevFrameBottomPoleRemovalFlow,
    prevBottomImage,
    prevBottomImage2,
    flow,
    OpticalFlowInterface::DirectionHint::DOWN);
  delete flowAlg;

  const string flowDir = outputDataDir + "/flow/" + frameNumber;
  const string flowImagesDir =
    outputDataDir + "/debug/" + frameNumber + "/flow_images";
  if (saveFlowDataForNextFrame) {
    VLOG(1) << "Serializing bottom-secondary flow and images";
    saveFlowToFile(flow, flowDir + "/flow_bottom_secondary.bin");
    imwriteExceptionOnFail(flowImagesDir + "/bottomImage.png", bottomImage);
    imwriteExceptionOnFail(flowImagesDir + "/bottomImage2.png", bottomImage2);
  }

  VLOG(1) << "Warping secondary bottom camera to align with primary bottom camera";
  assert(bottomImage.size() == bottomImage2.size());
  Mat warpMat(bottomImage.size(), CV_32FC2);
  for (int y = 0; y < warpMat.rows; ++y) {
    for (int x = 0; x < warpMat.cols; ++x) {
      warpMat.at<Point2f>(y, x) = Point2f(x, y) + flow.at<Point2f>(y, x);
    }
  }
  Mat warpedBottomImage2;
  Mat warpedTopImage;
  remap(
    bottomImage2,
    warpedBottomImage2,
    warpMat,
    Mat(),
    CV_INTER_CUBIC,
    BORDER_CONSTANT);

  const string debugDir = outputDataDir + "/debug/" + frameNumber;
  if (saveDebugImages) {
    imwriteExceptionOnFail(debugDir + "/bottomImage.png", bottomImage);
    imwriteExceptionOnFail(debugDir + "/bottomImage2.png", bottomImage2);
    imwriteExceptionOnFail(debugDir + "/bottomWarp2.png", warpedBottomImage2);
  }

  VLOG(1) << "Combining the primary bottom image and the secondary warped image";
  for (int y = 0; y < bottomImage.rows; ++y) {
    for (int x = 0; x < bottomImage.cols; ++x) {
      const float alpha = bottomImage.at<Vec4b>(y, x)[3] / 255.0f;
      const float alpha2 =
        warpedBottomImage2.at<Vec4b>(y, x)[3] / 255.0f;
      // if we don't have full alpha from the primary image, and we have some data from
      // the secondary image, use a weighted combination. otherwise leave it unchanged.
      if (alpha < 1.0f && alpha2 > 0.0f) {
        const float a1 = alpha;
        const float a2 = 1.0f - alpha;
        const float r1 = bottomImage.at<Vec4b>(y, x)[2];
        const float g1 = bottomImage.at<Vec4b>(y, x)[1];
        const float b1 = bottomImage.at<Vec4b>(y, x)[0];
        const float r2 = warpedBottomImage2.at<Vec4b>(y, x)[2];
        const float g2 = warpedBottomImage2.at<Vec4b>(y, x)[1];
        const float b2 = warpedBottomImage2.at<Vec4b>(y, x)[0];
        bottomImage.at<Vec4b>(y, x) = Vec4b(
          a1 * b1 + a2 * b2,
          a1 * g1 + a2 * g2,
          a1 * r1 + a2 * r2,
          255);
      }
    }
  }
  // redo the alpha channel.. this is to remove an alpha-channel hole where
  // pole masks overlap at the very bottom.
  circleAlphaCut(bottomImage, bottomCamUsablePixelsRadius);
  bottomImage = featherAlphaChannel(bottomImage, alphaFeatherSize);

  if (saveDebugImages) {
    imwriteExceptionOnFail(debugDir + "/_bottomCombined.png", bottomImage);
  }
}

} // namespace surround360
