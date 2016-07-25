/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "CameraMetadata.h"
#include "CvUtil.h"
#include "IntrinsicCalibration.h"
#include "MathUtil.h"
#include "NovelView.h"
#include "OpticalFlowVisualization.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;
using namespace surround360::calibration;
using namespace surround360::optical_flow;
using namespace surround360::math_util;

DEFINE_string(rig_json_file,        "",   "path to json file drescribing camera array");
DEFINE_string(images_dir,           "",   "path to dir with camera images (isp_out)");
DEFINE_string(vis_output_dir,       "",   "path to write output visualizations");

// baseImage should be a fisheye image. draws a crosshair located at the optical center
// specified by camModel.
void drawCrosshair(Mat& baseImage, const CameraMetadata& camModel) {
  const static Scalar kCrosshairColor = Scalar(0, 255, 0, 255);
  line(
    baseImage,
    Point2f(0, camModel.imageCenterY),
    Point2f(baseImage.cols, camModel.imageCenterY),
    kCrosshairColor,
    1,
    CV_AA);

  line(
    baseImage,
    Point2f(camModel.imageCenterX, 0),
    Point2f(camModel.imageCenterX, baseImage.rows),
    kCrosshairColor,
    1, // thickness
    CV_AA);

  // draw a thicker line at the usablePixelRadius
  circle(
    baseImage,
    Point2f(camModel.imageCenterX, camModel.imageCenterY),
    camModel.usablePixelsRadius,
    kCrosshairColor,
    2, // thickness
    CV_AA);

  // draw thinner concentric lines
  const static int kCircleRange = 300;
  const static int kCircleStep = 150;
  const static Scalar kCircleColor = Scalar(255, 255, 255, 255);
  for (int r = -kCircleRange; r <= kCircleRange; r += kCircleStep) {
    circle(
      baseImage,
      Point2f(camModel.imageCenterX, camModel.imageCenterY),
      camModel.usablePixelsRadius + r,
      kCircleColor,
      1, // thickness
      CV_AA);
  }
}

// reads a camera array description from --rig_json_file. checks if it has top, bottom,
// and secondary bottom cameras, and generates a cross-hair visualization for optical
// centering for cameras which exist, using the images in --images_dir as background.
// results are saved to --vis_output_dir
int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_vis_output_dir, "vis_output_dir");
  requireArg(FLAGS_images_dir, "images_dir");
  requireArg(FLAGS_rig_json_file, "rig_json_file");

  vector<CameraMetadata> camModelArrayWithTop =
      readCameraProjectionModelArrayFromJSON(FLAGS_rig_json_file);

  try {
    CameraMetadata topCamModel = getTopCamModel(camModelArrayWithTop);
    LOG(INFO) << "got top camera";
    Mat topImage = imreadExceptionOnFail(
      FLAGS_images_dir + "/" + topCamModel.cameraId + ".png", 1);
    drawCrosshair(topImage, topCamModel);
    imwriteExceptionOnFail(FLAGS_vis_output_dir + "/top_vis.png", topImage);
  } catch (const VrCamException& e) {}

  try {
    CameraMetadata bottomCamModel = getBottomCamModel(camModelArrayWithTop);
    LOG(INFO) << "got bottom camera";
    Mat bottomImage = imreadExceptionOnFail(
      FLAGS_images_dir + "/" + bottomCamModel.cameraId + ".png", 1);
    drawCrosshair(bottomImage, bottomCamModel);
    imwriteExceptionOnFail(FLAGS_vis_output_dir + "/bottom_vis.png", bottomImage);
  } catch (const VrCamException& e) {}

  try {
    CameraMetadata bottomCamModel2 = getBottomCamModel2(camModelArrayWithTop);
    LOG(INFO) << "got bottom2 camera";
    Mat bottomImage2 = imreadExceptionOnFail(
      FLAGS_images_dir + "/" + bottomCamModel2.cameraId + ".png", 1);
    drawCrosshair(bottomImage2, bottomCamModel2);
    imwriteExceptionOnFail(FLAGS_vis_output_dir + "/bottom2_vis.png", bottomImage2);
  } catch (const VrCamException& e) {}

  return EXIT_SUCCESS;
}
