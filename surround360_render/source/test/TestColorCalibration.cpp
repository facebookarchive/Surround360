/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <iostream>
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

  LOG(INFO) << "Finding black point...";

  const Point3f blackLevel = findBlackPoint(
    raw,
    FLAGS_isp_passthrough_path,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages);

  // Convert to RGB using passthrough IPS, applying the black level offset
  // Output image in [0..1]
  Mat rgb = raw2rgb(FLAGS_isp_passthrough_path, raw, blackLevel);

  if (FLAGS_save_debug_images) {
    Mat bgr;
    cvtColor(rgb, bgr, CV_RGB2BGR);
    const string rgbPassthroughImageFilename = FLAGS_output_data_dir +
          "/" + to_string(++stepDebugImages) + "_passthrough.png";
    imwriteExceptionOnFail(rgbPassthroughImageFilename, 255.0f * bgr);
  }

  // Resize image with Gaussian kernel. Low pass filter + speed-up
  Mat rgbSmall;
  pyrDown(rgb, rgbSmall, Size(rgb.cols / 2, rgb.rows / 2));

  LOG(INFO) << "Detecting color chart...";

  Mat rawSmall;
  pyrDown(raw, rawSmall, Size(raw.cols / 2, raw.rows / 2));
  vector<ColorPatch> colorPatches = detectColorChart(
    rawSmall,
    FLAGS_num_squares_w,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages);

  LOG(INFO) << "Computing patches RGB medians..";

  computeRGBMedians(
    colorPatches,
    rgbSmall,
    FLAGS_save_debug_images,
    FLAGS_output_data_dir,
    stepDebugImages);

  const int numPatchesExpected = FLAGS_num_squares_w * FLAGS_num_squares_h;

  if (colorPatches.size() != numPatchesExpected) {
    LOG(ERROR) << "Number of patches found (" << colorPatches.size()
               << ") different than expected (" << numPatchesExpected << ")";
    return EXIT_FAILURE;
  }

  LOG(INFO) << "Computing white balance gains...";

  const Vec3f channelRatios = computeChannelRatios(
    colorPatches,
    FLAGS_num_squares_w);

  if (FLAGS_save_debug_images) {
    plotWhiteBalanceHistogram(
      colorPatches,
      channelRatios,
      FLAGS_num_squares_w,
      FLAGS_output_data_dir,
      stepDebugImages);
  }

  // Apply white point ratio to entire image using our ISP
  // Output image in [0..1]
  Mat rgbWB = raw2rgb(FLAGS_isp_passthrough_path, raw, blackLevel, channelRatios);
  if (FLAGS_save_debug_images) {
    Mat bgrWB;
    cvtColor(rgbWB, bgrWB, CV_RGB2BGR);
    const string rgbWBImageFilename = FLAGS_output_data_dir +
          "/" + to_string(++stepDebugImages) + "_white_balance.png";
    imwriteExceptionOnFail(rgbWBImageFilename, 255.0f * bgrWB);
  }

  // Update medians to reflect white balance (centroids were found on scaled
  // down version)
  for (int i = 0; i < colorPatches.size(); ++i) {
    Point2f centroid = colorPatches[i].centroid * 2.0f;
    colorPatches[i].rgbMedian = rgbWB.at<Vec3f>(centroid);
  }

  LOG(INFO) << "Computing CCM...";

  Mat ccm = computeCCM(colorPatches);

  LOG(INFO) << "ISP: Black level: " << blackLevel;
  LOG(INFO) << "ISP: White balance gains (RGB): " << channelRatios;
  LOG(INFO) << "ISP: CCM: " << ccm.t();

  // Apply CCM to entire image
  Mat rgbCCM = raw2rgb(
    FLAGS_isp_passthrough_path,
    raw,
    blackLevel,
    channelRatios,
    ccm.t());

  if (FLAGS_save_debug_images) {
    Mat bgrCCM;
    cvtColor(rgbCCM, bgrCCM, CV_RGB2BGR);
    const string rgbCCMImageFilename = FLAGS_output_data_dir +
          "/" + to_string(++stepDebugImages) + "_ccm.png";
    imwriteExceptionOnFail(rgbCCMImageFilename, 255.0f * bgrCCM);
  }

  LOG(INFO) << "Computing errors...";

  Mat rgbCCMSmall;
  pyrDown(rgbCCM, rgbCCMSmall, Size(rgbCCM.cols / 2, rgbCCM.rows / 2));
  pair<Vec4f, Vec4f> errors = computeColorPatchErrors(
    rgbSmall,
    rgbCCMSmall,
    colorPatches);

  LOG(INFO) << "RGB, R, G, B errors (before): " << errors.first;
  LOG(INFO) << "RGB, R, G, B errors (after): " << errors.second;

  LOG(INFO) << "Applying gamma correction...";

  string ispConfigPathOut = FLAGS_output_data_dir + "/isp_out.json";
  Point3f gamma {0.4545, 0.4545, 0.4545};
  Mat rgbGamma = raw2rgb(
    FLAGS_isp_passthrough_path,
    raw,
    blackLevel,
    channelRatios,
    ccm.t(),
    gamma,
    ispConfigPathOut);

  if (FLAGS_save_debug_images) {
    Mat bgrGamma;
    cvtColor(rgbGamma, bgrGamma, CV_RGB2BGR);
    const string rgbGammaImageFilename = FLAGS_output_data_dir +
          "/" + to_string(++stepDebugImages) + "_gamma_corrected.png";
    imwriteExceptionOnFail(rgbGammaImageFilename, 255.0f * bgrGamma);
  }

  return EXIT_SUCCESS;
}
