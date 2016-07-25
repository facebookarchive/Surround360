/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "CameraMetadata.h"
#include "CvUtil.h"
#include "Filter.h"
#include "ImageWarper.h"
#include "IntrinsicCalibration.h"
#include "MathUtil.h"
#include "MonotonicTable.h"
#include "NovelView.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace cv;
using namespace std;
using namespace surround360;
using namespace surround360::calibration;
using namespace surround360::math_util;
using namespace surround360::optical_flow;
using namespace surround360::util;
using namespace surround360::warper;

DEFINE_string(rig_json_file,          "",       "path to json file drescribing camera array");
DEFINE_string(imgs_dir,               "",       "path to folder of images with names matching cameras in the rig file");
DEFINE_string(output_data_dir,        "",       "path to write spherical projections for debugging");
DEFINE_string(output_equirect_path,   "",       "path to write output oculus 360 cubemap");
DEFINE_bool(enable_top,               false,    "is there a top camera?");
DEFINE_bool(enable_bottom,            false,    "are there two bottom cameras?");
DEFINE_int32(eqr_width,               1024,     "height of spherical projection image (0 to 2pi)");
DEFINE_int32(eqr_height,              1024,     "height of spherical projection image (0 to pi)");

// reads a camera array description from --rig_json_file, and camera images from
// --imgs_dir, then extracts just the top and primary bottom cameras (assuming fisheye).
// these two images are reprojected to form a full equirect panorama (no attempt at
// stitching or blending). the merged equirect image from both cameras is saved to
// --output_equirect_path, and separate equirect projections of the images are saved to
// --output_data_dir
int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_imgs_dir, "imgs_dir");
  requireArg(FLAGS_output_data_dir, "output_data_dir");
  requireArg(FLAGS_output_equirect_path, "output_equirect_path");

  LOG(INFO) << "starting up. imgs_dir=" << FLAGS_imgs_dir;

  // load camera meta data and source images
  LOG(INFO) << "reading camera model json";
  vector<CameraMetadata> camModelArrayWithTop =
    readCameraProjectionModelArrayFromJSON(FLAGS_rig_json_file);

  LOG(INFO) << "verifying image filenames";
  verifyImageDirFilenamesMatchCameraArray(camModelArrayWithTop, FLAGS_imgs_dir);

  Mat equirectImage(FLAGS_eqr_height, FLAGS_eqr_width, CV_8UC4);

  CameraMetadata bottomCamModel, topCamModel;
  Mat bottomImage, topImage, bottomSpherical, topSpherical;

  if (FLAGS_enable_top) {
    topCamModel = getTopCamModel(camModelArrayWithTop);
    const string topImagePath = FLAGS_imgs_dir + "/" + topCamModel.cameraId + ".png";
    topImage = imreadExceptionOnFail(topImagePath, CV_LOAD_IMAGE_COLOR);

    topSpherical = bicubicRemapFisheyeToSpherical(
      topCamModel,
      topImage,
      Size(
        FLAGS_eqr_width,
        FLAGS_eqr_height * (topCamModel.fisheyeFovDegrees / 2.0f) / 180.0f));
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/topSpherical.png", topSpherical);

    cvtColor(topSpherical, topSpherical, CV_BGR2BGRA);
    copyMakeBorder(
      topSpherical,
      topSpherical,
      0,
      FLAGS_eqr_height - topSpherical.rows,
      0,
      0,
      BORDER_CONSTANT);
    equirectImage = flattenLayers<Vec4b>(equirectImage, topSpherical);
  }

  if (FLAGS_enable_bottom) {
    bottomCamModel = getBottomCamModel(camModelArrayWithTop);
    const string bottomImagePath =
      FLAGS_imgs_dir + "/" + bottomCamModel.cameraId + ".png";
    bottomImage = imreadExceptionOnFail(bottomImagePath, CV_LOAD_IMAGE_COLOR);

    bottomSpherical = bicubicRemapFisheyeToSpherical(
      bottomCamModel,
      bottomImage,
      Size(
        FLAGS_eqr_width,
        FLAGS_eqr_height * (bottomCamModel.fisheyeFovDegrees / 2.0f) / 180.0f));
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/bottomSpherical.png", bottomSpherical);

    flip(equirectImage, equirectImage, -1);
    cvtColor(bottomSpherical, bottomSpherical, CV_BGR2BGRA);
    copyMakeBorder(
      bottomSpherical,
      bottomSpherical,
      0,
      FLAGS_eqr_height - bottomSpherical.rows,
      0,
      0,
      BORDER_CONSTANT);
    equirectImage = flattenLayers<Vec4b>(equirectImage, bottomSpherical);
    flip(equirectImage, equirectImage, -1);
  }

  imwriteExceptionOnFail(FLAGS_output_equirect_path, equirectImage);

  return EXIT_SUCCESS;
}
