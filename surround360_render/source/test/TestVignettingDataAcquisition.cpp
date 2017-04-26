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
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <folly/FileUtil.h>
#include <folly/json.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;
using namespace surround360::color_calibration;

DEFINE_string(input_dir,              "",     "path to directory of RAW images containing color chart");
DEFINE_string(output_dir,             "",     "path to write data for debugging");
DEFINE_string(isp_json,               "",     "ISP configuration file");
DEFINE_double(min_area_chart_perc,    0.1,    "expected min chart area (% of entire image)");
DEFINE_double(max_area_chart_perc,    10.0,   "expected max chart area (% of entire image)");
DEFINE_bool(save_debug_images,        false,  "save intermediate images");

void saveToJson(
    const vector<string>& imageIds,
    const vector<Point2f>& locations,
    const vector<Vec3f>& medians,
    const string& filename) {

  folly::dynamic serialized = folly::dynamic::array;
  for (int i = 0; i < locations.size(); ++i) {
    const Point2f& loc = locations[i];
    const Vec3f& median = medians[i];
    folly::dynamic properties =
      folly::dynamic::object
        ("image_id", imageIds[i])
        ("location", folly::dynamic::array(loc.x, loc.y))
        ("rgbmedian", folly::dynamic::array(median[0], median[1], median[2]));
    serialized.push_back(properties);
  }
  folly::writeFile(folly::toPrettyJson(serialized), filename.c_str());
}

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_input_dir, "input_dir");
  requireArg(FLAGS_isp_json, "isp_json");

  const string outputDir =
    FLAGS_output_dir.empty() ? FLAGS_input_dir + "/output" : FLAGS_output_dir;
  system(string("mkdir -p \"" + outputDir + "\"").c_str());

  Mat vignettePatchMask;

  vector<string> imageIds;
  vector<string> invalidIds;
  vector<Point2f> vignetteMapLocs;
  vector<Vec3f> vignetteMapRGBValues;

  bool firstPass = true;
  int stepDebugImages = 0;
  unsigned int currentCount = 0;
  unsigned int countValidCharts = 0;

  const string chartsDir = FLAGS_input_dir + "/charts";
  const vector<string> imageFilenames = getFilesInDir(chartsDir, false);
  for (const string& imageFilename : imageFilenames) {
    LOG(INFO) << "Images left: " << imageFilenames.size() - currentCount;

    ++currentCount;

    LOG(INFO) << "Loading " + imageFilename + "...";

    const string imageId = stringSplit(imageFilename, '.')[0];

    const string imageDir = outputDir + "/" + imageId;
    system(string("mkdir -p \"" + imageDir + "\"").c_str());

    const string imagePath = chartsDir + "/" + imageFilename;
    Mat raw = imreadExceptionOnFail(
      imagePath, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);

    Mat raw8;
    Mat raw16;
    const uint8_t rawDepth = raw.type() & CV_MAT_DEPTH_MASK;
    if (rawDepth == CV_8U) {
      raw8 = raw;
      raw16 = convert8bitTo16bit(raw.clone());
    } else if (rawDepth == CV_16U) {
      raw8 = imreadExceptionOnFail(imagePath, CV_LOAD_IMAGE_GRAYSCALE);
      raw16 = raw;
    } else {
      throw VrCamException("Input image is not 8-bit or 16-bit");
    }

    static const int kNumSquaresW = 1;
    static const int kNumSquaresH = 1;
    const float imageSize = raw8.rows * raw8.cols;
    const float minAreaChart = (FLAGS_min_area_chart_perc / 100.0f) * imageSize;
    const float maxAreaChart = (FLAGS_max_area_chart_perc / 100.0f) * imageSize;
    static const bool kSaveChartDebugImages = false;
    vector<ColorPatch> colorPatches = detectColorChart(
      raw8,
      kNumSquaresW,
      kNumSquaresH,
      minAreaChart,
      maxAreaChart,
      kSaveChartDebugImages,
      imageDir,
      stepDebugImages);

    if (colorPatches.size() == 0) {
      invalidIds.push_back(imageId);
      continue;
    }

    // There may be two overlapping masks (inner and outer contours of chart)
    // Pick smallest one
    int countMaskMin = INT_MAX;
    int kPatchRefId = 0;
    for (int i = 0; i < colorPatches.size(); ++i) {
      Mat mask = colorPatches[i].mask;
      if (countNonZero(mask) < countMaskMin) {
        countMaskMin = countNonZero(mask);
        kPatchRefId = i;
      }
    }

    imageIds.push_back(imageId);

    ColorPatch colorPatch = colorPatches[kPatchRefId];
    vignetteMapLocs.push_back(colorPatch.centroid);

    // Need to apply black level before getting patch medians
    CameraIsp cameraIsp(getJson(FLAGS_isp_json), getBitsPerPixel(raw16));
    cameraIsp.setup();
    cameraIsp.loadImage(raw16);
    cameraIsp.blackLevelAdjust();
    Mat rawNormalized = cameraIsp.getRawImage();

    static const bool kIsRaw = true;
    colorPatch.rgbMedian =
      getRgbMedianMask(rawNormalized, colorPatch.mask, FLAGS_isp_json, kIsRaw);
    vignetteMapRGBValues.push_back(colorPatch.rgbMedian);

    if (FLAGS_save_debug_images) {
      const string patchMaskFilename = imageDir + "/patch_mask.png";
      imwriteExceptionOnFail(patchMaskFilename, 255.0f * colorPatch.mask);
    }

    if (firstPass) {
      vignettePatchMask =
        Mat::zeros(colorPatch.mask.size(), colorPatch.mask.type());
      firstPass = false;
    }

    bitwise_or(vignettePatchMask, colorPatch.mask, vignettePatchMask);

    LOG(INFO) << "Valid charts: " << ++countValidCharts << "/" << currentCount;
  }

  for (int i = 0; i < vignetteMapLocs.size(); ++i) {
    static const double kTextFontScale = 0.8;
    const string text = imageIds[i];
    putText(
      vignettePatchMask,
      text,
      vignetteMapLocs[i],
      FONT_HERSHEY_SIMPLEX,
      kTextFontScale,
      Scalar(0));
  }

  if (invalidIds.size() > 0) {
    LOG(INFO) << "Invalid images: " << invalidIds.size() << "/" << currentCount;
    for (const string& imageId : invalidIds) {
      LOG(INFO) << imageId;
    }
  }

  if (countValidCharts == 0) {
    throw VrCamException("No samples found...");
  }

  if (countValidCharts > 0 && FLAGS_save_debug_images) {
    const string vignetteMaskFilename = outputDir + "/vignette_patch_mask.png";
    imwriteExceptionOnFail(vignetteMaskFilename, 255.0f * vignettePatchMask);
  }

  // Save locations and medians to txt file for future use
  const string dataPath = outputDir + "/data.json";
  saveToJson(imageIds, vignetteMapLocs, vignetteMapRGBValues, dataPath);

  return EXIT_SUCCESS;
}
