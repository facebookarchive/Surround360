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
#ifdef USE_HALIDE
#include "CameraIspPipe.h"
#endif
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

DEFINE_string(image_path,             "",       "path to RAW image containing color chart");
DEFINE_string(illuminant,             "D65",    "illuminant (D50 or D65)");
DEFINE_string(isp_passthrough_path,   "",       "passthrough ISP configuration file path");
DEFINE_int32(num_squares_w,           6,        "number of squares horizontally");
DEFINE_int32(num_squares_h,           4,        "number of squares vertically");
DEFINE_double(min_area_chart_perc,    0.5,      "expected min chart area (% of entire image)");
DEFINE_double(max_area_chart_perc,    40.0,     "expected max chart area (% of entire image)");
DEFINE_string(output_data_dir,        ".",      "path to write data for debugging");
DEFINE_bool(update_clamps,            false,    "just update passthrough ISP with given clamps");
DEFINE_double(clamp_min,              0.0,      "min clamping threshold");
DEFINE_double(clamp_max,              1.0,      "max clamping threshold");
DEFINE_string(black_level,            "",       "space-separated manual black level (values in range [0, 1])");
DEFINE_bool(black_level_hole,         false,    "if true, get black from black hole in image");
DEFINE_int32(black_level_hole_pixels, 500,      "estimated size of black hole (pixels)");
DEFINE_bool(black_level_y_intercept,  false,    "if true, get black level from Y-intercept of RGB response");
DEFINE_bool(save_debug_images,        false,    "save intermediate images");

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_image_path, "image_path");
  requireArg(FLAGS_isp_passthrough_path, "isp_passthrough_path");

  if (labMacbeth.find(FLAGS_illuminant) == labMacbeth.end()) {
    LOG(ERROR) << "Illuminant " << FLAGS_illuminant << " not supported";
    return EXIT_FAILURE;
  }

  if (FLAGS_num_squares_w < FLAGS_num_squares_h) {
    LOG(ERROR) << "Only supporting num_squares_w > num_squares_h";
    return EXIT_FAILURE;
  }

  if (FLAGS_output_data_dir != "") {
    system(string("mkdir -p \"" + FLAGS_output_data_dir + "\"").c_str());
  }

  int stepDebugImages = 0;

  Mat raw = imreadExceptionOnFail(
    FLAGS_image_path, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);

  Mat raw8;
  Mat raw16;
  const uint8_t rawDepth = raw.type() & CV_MAT_DEPTH_MASK;
  if (rawDepth == CV_8U) {
    raw8 = raw;
    raw16 = convert8bitTo16bit(raw.clone());
  } else if (rawDepth == CV_16U) {
    raw8 = imreadExceptionOnFail(FLAGS_image_path, CV_LOAD_IMAGE_GRAYSCALE);
    raw16 = raw;
  } else {
    throw VrCamException("Input image is not 8-bit or 16-bit");
  }

  if (FLAGS_update_clamps) {
    const float clampMin = static_cast<float>(FLAGS_clamp_min);
    const float clampMax = static_cast<float>(FLAGS_clamp_max);
    const Vec3f rgbClampMin = {clampMin, clampMin, clampMin};
    const Vec3f rgbClampMax = {clampMax, clampMax, clampMax};
    const int bitsPerPixel = getBitsPerPixel(raw16);
    updateIspWithClamps(
      FLAGS_isp_passthrough_path,
      bitsPerPixel,
      rgbClampMin,
      rgbClampMax);
    return EXIT_SUCCESS;
  }

  if (FLAGS_save_debug_images) {
    const string rawImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) + "_" +
        FLAGS_illuminant + "_raw.png";
    imwriteExceptionOnFail(rawImageFilename, raw16);
  }

  LOG(INFO) << "Detecting clamped pixels...";

  Mat rawClamped = findClampedPixels(raw8);

  if (FLAGS_save_debug_images) {
    const string rawClampedImageFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_raw_clamped_pixels.png";
    imwriteExceptionOnFail(rawClampedImageFilename, rawClamped);
  }
  LOG(INFO) << "Detecting color chart...";

  const float imageSize = raw8.rows * raw8.cols;
  const float minAreaChart = (FLAGS_min_area_chart_perc / 100.0f) * imageSize;
  const float maxAreaChart = (FLAGS_max_area_chart_perc / 100.0f) * imageSize;
  vector<ColorPatch> colorPatches = detectColorChart(
    raw8,
    FLAGS_num_squares_w,
    FLAGS_num_squares_h,
    minAreaChart,
    maxAreaChart,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages);

  const int numPatchesExpected = FLAGS_num_squares_w * FLAGS_num_squares_h;

  if (colorPatches.size() != numPatchesExpected) {
    LOG(ERROR) << "Number of patches found (" << colorPatches.size()
               << ") different than expected (" << numPatchesExpected << ")";
    return EXIT_FAILURE;
  }

  const Mat rawNormalized = getRaw(FLAGS_isp_passthrough_path, raw16.clone());

  ColorResponse colorResponse = computeRGBResponse(
    rawNormalized.clone(),
    true,
    colorPatches,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    "raw");

  // Save X-intercepts
  saveXIntercepts(colorResponse, FLAGS_output_data_dir);

  Vec3f blackLevel = Vec3f(0.0f, 0.0f, 0.0f);
  bool isBlackLevelSet = false;
  if (FLAGS_black_level != "") {
    LOG(INFO) << "Manually set black level...";
    std::istringstream blIs(FLAGS_black_level);
    vector<float> blVec;
    blVec.assign(
      std::istream_iterator<float>(blIs),
      std::istream_iterator<float>());
    blackLevel = Vec3f(blVec[0], blVec[1], blVec[2]);
    isBlackLevelSet = true;
  } else if (FLAGS_black_level_hole) {
    LOG(INFO) << "Finding black hole...";
    blackLevel = findBlackLevel(
      raw16,
      FLAGS_black_level_hole_pixels,
      FLAGS_isp_passthrough_path,
      FLAGS_save_debug_images,
      FLAGS_output_data_dir,
      stepDebugImages);
    isBlackLevelSet = true;
  } else if (FLAGS_black_level_y_intercept) {
    LOG(INFO) << "Black level from Y-intercept...";
    blackLevel = colorResponse.rgbInterceptY;
    isBlackLevelSet = true;
  }

  LOG(INFO) << "Generating ISP parameters...";

  Vec3f whiteBalance;
  Mat ccm;
  obtainIspParams(
    colorPatches,
    FLAGS_illuminant,
    raw16.size(),
    isBlackLevelSet,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages,
    blackLevel,
    whiteBalance,
    ccm);

  saveBlackLevel(blackLevel, FLAGS_output_data_dir);

  LOG(INFO) << "Generating ISP config file...";

  const int bitsPerPixel = getBitsPerPixel(raw16);
  const float maxPixelValue = static_cast<float>((1 << bitsPerPixel) - 1);
  static const Point3f kGamma = Point3f(0.4545, 0.4545, 0.4545);
  string ispConfigPathOut = FLAGS_output_data_dir + "/isp_out.json";
  CameraIsp cameraIsp(getJson(FLAGS_isp_passthrough_path), bitsPerPixel);
  writeIspConfigFile(
    ispConfigPathOut,
    cameraIsp,
    blackLevel * maxPixelValue,
    whiteBalance,
    ccm.t(),
    kGamma);

  LOG(INFO) << "Applying ISP parameters to input image...";

  static const int kOutputBpp = 16;
  Mat rgbOut(raw16.size(), CV_16UC3);

  #ifdef USE_HALIDE
    static const bool kFast = false;
    CameraIspPipe cameraIspTest(getJson(ispConfigPathOut), kFast, kOutputBpp);
  #else
    CameraIsp cameraIspTest(getJson(ispConfigPathOut), kOutputBpp);
  #endif

  cameraIspTest.setBitsPerPixel(kOutputBpp);
  cameraIspTest.loadImage(raw16);
  cameraIspTest.setup();

  #ifdef USE_HALIDE
    cameraIspTest.initPipe();
  #endif

  cameraIspTest.getImage(rgbOut);

  if (FLAGS_save_debug_images) {
    const string rgbOutFilename =
      FLAGS_output_data_dir + "/" + to_string(++stepDebugImages) +
      "_isp_out.png";
    imwriteExceptionOnFail(rgbOutFilename, rgbOut);
  }

  return EXIT_SUCCESS;
}
