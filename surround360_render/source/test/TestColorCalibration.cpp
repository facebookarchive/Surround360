/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "CameraIsp.h"
#include "ColorCalibration.h"
#include "CvUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;
using namespace surround360::color_calibration;

DEFINE_string(image_path,           "",     "path to RAW image containing color chart");
DEFINE_string(isp_passthrough_path, "",     "passthrough ISP configuration file path");
DEFINE_int32(num_squares_w,         6,      "number of squares horizontally");
DEFINE_int32(num_squares_h,         4,      "number of squares vertically");
DEFINE_string(output_data_dir,      ".",    "path to write data for debugging");
DEFINE_double(clamp_min,            0.0,    "min clamping threshold");
DEFINE_double(clamp_max,            1.0,    "max clamping threshold");
DEFINE_bool(just_intercepts,        false,  "just compute RGB X-intercepts");
DEFINE_string(black_level,          "",     "space-separated manual black level (8-bit)");
DEFINE_bool(black_level_darkest,    false,  "if true, assume black level is darkest point");
DEFINE_bool(save_debug_images,      false,  "save intermediate images");

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_image_path, "image_path");
  requireArg(FLAGS_isp_passthrough_path, "isp_passthrough_path");

  if (FLAGS_num_squares_w < FLAGS_num_squares_h) {
    LOG(ERROR) << "Only supporting num_squares_w > num_squares_h";
    return EXIT_FAILURE;
  }

  if (FLAGS_output_data_dir != "") {
    system(string("mkdir -p " + FLAGS_output_data_dir).c_str());
  }

  int stepDebugImages = 0;

  Mat raw = imreadExceptionOnFail(FLAGS_image_path, CV_LOAD_IMAGE_GRAYSCALE);

  // Working with 16-bit images
  Mat raw16 = convert8bitTo16bit(raw);

  if (FLAGS_save_debug_images) {
    const string rawImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) + "_raw.png";
    imwriteExceptionOnFail(rawImageFilename, raw16);
  }

  LOG(INFO) << "Detecting clamped pixels...";

  Mat rawClamped = findClampedPixels(raw);

  if (FLAGS_save_debug_images) {
    const string rawClampedImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_raw_clamped_pixels.png";
    imwriteExceptionOnFail(rawClampedImageFilename, rawClamped);
  }
  LOG(INFO) << "Detecting color chart...";

  vector<ColorPatch> colorPatches = detectColorChart(
    raw,
    FLAGS_num_squares_w,
    FLAGS_num_squares_h,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages);

  const int numPatchesExpected = FLAGS_num_squares_w * FLAGS_num_squares_h;

  if (colorPatches.size() != numPatchesExpected) {
    LOG(ERROR) << "Number of patches found (" << colorPatches.size()
               << ") different than expected (" << numPatchesExpected << ")";
    return EXIT_FAILURE;
  }

  Mat raw16isp = getRaw(FLAGS_isp_passthrough_path, raw16);

  ColorResponse colorResponse = computeRGBResponse(
    raw16isp.clone(),
    true,
    colorPatches,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    "raw");

  // Save X-intercepts
  saveXIntercepts(colorResponse, FLAGS_output_data_dir);

  if (FLAGS_just_intercepts) {
    return EXIT_SUCCESS;
  }

  Vec3f blackLevel = Vec3f(0.0f, 0.0f, 0.0f);
  if (FLAGS_black_level != "") {
    std::istringstream blIs(FLAGS_black_level);
    vector<float> blVec;
    blVec.assign(std::istream_iterator<int>(blIs), std::istream_iterator<int>());
    blackLevel = Vec3f(blVec[0] / 255.0f, blVec[1] / 255.0f, blVec[2] / 255.0f);
  } else if (FLAGS_black_level_darkest) {
    // Find black level on original raw
    blackLevel = findBlackLevel(
      raw16,
      FLAGS_isp_passthrough_path,
      FLAGS_save_debug_images,
      FLAGS_output_data_dir,
      stepDebugImages);
  } else {
    blackLevel = colorResponse.rgbInterceptY;
  }

  LOG(INFO) << "Adjusting black level: " << blackLevel;

  Mat rawBlackAdjusted = adjustBlackLevel(
    FLAGS_isp_passthrough_path,
    raw16,
    raw16isp.clone(),
    blackLevel);

  if (FLAGS_save_debug_images) {
    const string noBlackImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_black_level_adjusted.png";
    imwriteExceptionOnFail(noBlackImageFilename, 255.0f * rawBlackAdjusted);
  }

  colorResponse = computeRGBResponse(
    rawBlackAdjusted,
    true,
    colorPatches,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    "raw_black_adjusted");

  LOG(INFO) << "Computing white balance gains...";

  Vec3f wbGains = computeWhiteBalanceGains(colorResponse);

  LOG(INFO) << "White balance gains: " << wbGains;

  // Apply white point ratio to entire image using ISP
  Mat rawWB = whiteBalance(
    FLAGS_isp_passthrough_path,
    raw16,
    rawBlackAdjusted.clone(),
    wbGains);

  if (FLAGS_save_debug_images) {
    const string rawWBImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_white_balance_raw.png";
    imwriteExceptionOnFail(rawWBImageFilename, 255.0f * rawWB);
  }

  colorResponse = computeRGBResponse(
    rawWB,
    true,
    colorPatches,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    "wb_raw");

  const float clampMin = float(FLAGS_clamp_min);
  const float clampMax = float(FLAGS_clamp_max);

  LOG(INFO) << "Clamping at " << clampMin << ", " << clampMax;

  Vec3f rgbClampMin = {clampMin, clampMin, clampMin};
  Vec3f rgbClampMax = {clampMax, clampMax, clampMax};
  Mat rawWBClamped = clampAndStretch(
    FLAGS_isp_passthrough_path,
    raw16,
    rawWB.clone(),
    colorResponse,
    rgbClampMin,
    rgbClampMax);

  if (FLAGS_save_debug_images) {
    const string rawWBClampedImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_white_balance_raw_clamped.png";
    imwriteExceptionOnFail(rawWBClampedImageFilename, 255.0f * rawWBClamped);
  }

  colorResponse = computeRGBResponse(
    rawWBClamped,
    true,
    colorPatches,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    "wb_raw_clamped");

  LOG(INFO) << "Demosaicing...";

  Mat rgbWB = demosaic(FLAGS_isp_passthrough_path, raw, rawWBClamped.clone());

  if (FLAGS_save_debug_images) {
    Mat bgrWB;
    cvtColor(rgbWB, bgrWB, CV_RGB2BGR);
    const string rgbWBImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_white_balance_rgb.png";
    imwriteExceptionOnFail(rgbWBImageFilename, 255.0f * bgrWB);
  }

  colorResponse = computeRGBResponse(
    rgbWB,
    false,
    colorPatches,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    "wb_rgb");

  LOG(INFO) << "Computing CCM...";

  Mat ccm = computeCCM(colorPatches);

  LOG(INFO) << "ISP: Black level: " << blackLevel;
  LOG(INFO) << "ISP: White balance gains (RGB): " << wbGains;
  LOG(INFO) << "ISP: CCM: " << ccm.t();

  LOG(INFO) << "Applying color correction...";

  Mat rgbCCM = colorCorrect(FLAGS_isp_passthrough_path, raw, rgbWB.clone(), ccm.t());

  if (FLAGS_save_debug_images) {
    Mat bgrCCM;
    cvtColor(rgbCCM, bgrCCM, CV_RGB2BGR);
    const string rgbCCMImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) + "_ccm.png";
    imwriteExceptionOnFail(rgbCCMImageFilename, 255.0f * bgrCCM);
  }

  colorResponse = computeRGBResponse(
    rgbCCM,
    false,
    colorPatches,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    "rgb_ccm");

  LOG(INFO) << "Applying gamma correction...";

  static const Point3f kGamma = Point3f(0.4545, 0.4545, 0.4545);
  Mat rgbGamma = colorCorrect(
    FLAGS_isp_passthrough_path, raw, rgbWB.clone(), ccm.t(), kGamma);

  if (FLAGS_save_debug_images) {
    Mat bgrGamma;
    cvtColor(rgbGamma, bgrGamma, CV_RGB2BGR);
    const string rgbGammaImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_gamma_corrected.png";
    imwriteExceptionOnFail(rgbGammaImageFilename, 255.0f * bgrGamma);
  }

  LOG(INFO) << "Computing errors...";

  pair<Vec4f, Vec4f> errors =
    computeColorPatchErrors(rgbCCM, rgbCCM.clone(), colorPatches);

  LOG(INFO) << "RGB, R, G, B errors (before): " << errors.first;
  LOG(INFO) << "RGB, R, G, B errors (after): " << errors.second;

  LOG(INFO) << "Generating ISP config file...";

  string ispConfigPathOut = FLAGS_output_data_dir + "/isp_out.json";
  writeIspConfigFile(
    FLAGS_isp_passthrough_path,
    ispConfigPathOut,
    raw16,
    blackLevel,
    wbGains,
    rgbClampMin,
    rgbClampMax,
    ccm.t(),
    kGamma);

  return EXIT_SUCCESS;
}
