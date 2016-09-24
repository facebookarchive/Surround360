/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <string>

#include "CameraIsp.h"
#ifdef USE_HALIDE
#include "CameraIspPipe.h"
#endif
#include "CvUtil.h"
#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace cv;
using namespace std;
using namespace surround360;
using namespace surround360::math_util;
using namespace surround360::util;

DEFINE_string(input_image_path,     "",                     "input image path");
DEFINE_string(output_image_path,    "",                     "output image path");
DEFINE_string(isp_config_path,      "",                     "ISP configuration file path");
DEFINE_int32(black_level_offset,    0,                      "amount to add to the blacklevel in the config file");
DEFINE_int32(demosaic_filter,       EDGE_AWARE_DM_FILTER,   "Demosaic filter type: 0=Bilinear(fast), 1=Frequency, 2=Edge aware");
DEFINE_int32(resize,                1,                      "Amount to \"bin-down\" the input. Legal values are 1, 2, 4, and 8");
DEFINE_int32(output_bpp,            8,                      "output image bits per pixel, either 8 or 16");
#ifdef USE_HALIDE
DEFINE_bool(accelerate,             false,                  "Use halide accelerated version");
DEFINE_bool(fast,                   false,                  "Use fastest halide for realtime apps or previews");
#endif

Mat readRaw(
    const string& filename,
    const int width,
    const int height,
    const int bitsPerPixel) {

  ifstream inputRawImageFile(filename, ios::in | ios::binary);
  if (!inputRawImageFile) {
    throw VrCamException("file read failed: " + filename);
  }

  int bytesLeft = width * height * 2;

  Mat rawImage(height, width, CV_16UC1);
  rawImage = 0;

  VLOG(1) << "Reading raw";
  if (inputRawImageFile) {
    char* dataPtr = reinterpret_cast<char*>(rawImage.data);
    inputRawImageFile.read(dataPtr, bytesLeft);
    if (inputRawImageFile.gcount() != bytesLeft) {
      LOG(WARNING) << "Warning: expected " << bytesLeft
                   << " but only read " << inputRawImageFile.gcount();
    }
    inputRawImageFile.close();
  }
  imwriteExceptionOnFail("raw.tif", rawImage);
  return rawImage;
}

void runPipeline(
  CameraIsp* cameraIsp,
  Mat& inputImage,
  Mat& outputImage,
  string outputImagePath) {
  cameraIsp->addBlackLevelOffset(FLAGS_black_level_offset);
  cameraIsp->loadImage(inputImage);

  double startTime = getCurrTimeSec();
  cameraIsp->getImage(outputImage);
  double endTime = getCurrTimeSec();
  LOG(INFO) << "Runtime = " << (endTime - startTime) * 1000.0 << "ms" << endl;

  imwriteExceptionOnFail(outputImagePath, outputImage);
}

int main(int argc, char* argv[]) {
  initSurround360(argc, argv);
  requireArg(FLAGS_input_image_path, "input_image_path");
  requireArg(FLAGS_output_image_path, "output_image_path");
  requireArg(FLAGS_isp_config_path, "isp_config_path");

  // Load the json camera ISP configuration
  ifstream ifs(FLAGS_isp_config_path, std::ios::in);
  if (!ifs) {
    throw VrCamException("file read failed: " + FLAGS_isp_config_path);
  }
  std::string json(
    (std::istreambuf_iterator<char>(ifs)),
    std::istreambuf_iterator<char>());

  const json::Object config = json::Deserialize(json);
  const bool isRaw = string(FLAGS_input_image_path).find(string(".raw")) != -1;

  Mat inputImage =
    isRaw
    ? readRaw(
      argv[1],
      getInteger(config, "CameraIsp", "width"),
      getInteger(config, "CameraIsp", "height"),
      getInteger(config, "CameraIsp", "bitsPerPixel"))
    : imreadExceptionOnFail(FLAGS_input_image_path, CV_LOAD_IMAGE_GRAYSCALE);

  if (inputImage.cols > 2 && inputImage.rows > 2) {
    const uint8_t depth = inputImage.type() & CV_MAT_DEPTH_MASK;

    // Make all the input data S16
    Mat inputImage16(inputImage.rows, inputImage.cols, CV_16U);

    if (depth == CV_8U) {
      for (int i = 0; i < inputImage.rows; ++i) {
        for (int j = 0; j < inputImage.cols; ++j) {
          // Use the repeating high order bits in low order bits to
          // correctly fill 15 bits.
          inputImage16.at<uint16_t>(i, j) =
            (uint16_t(inputImage.at<uint8_t>(i, j)) << 8) |
            (uint16_t(inputImage.at<uint8_t>(i, j)) & 0xff);
        }
      }
    } else if (depth == CV_16U) {
      inputImage16 = inputImage;
    } else {
      throw VrCamException("input is larger that 16 bits per pixel");
    }

    Mat outputImage(inputImage.rows, inputImage.cols, FLAGS_output_bpp == 8 ? CV_8UC3 : CV_16UC3);

#ifdef USE_HALIDE
    if (FLAGS_accelerate) {
      CameraIspPipe cameraIsp(json, FLAGS_fast, FLAGS_output_bpp);
      runPipeline(&cameraIsp, inputImage16, outputImage, FLAGS_output_image_path);
    } else {
#else
    {
#endif
      CameraIsp cameraIsp(json, FLAGS_output_bpp);
      cameraIsp.setDemosaicFilter(FLAGS_demosaic_filter);
      cameraIsp.setResize(FLAGS_resize);
      runPipeline(&cameraIsp, inputImage16, outputImage, FLAGS_output_image_path);
    }
  } else {
    throw VrCamException("Unable to open " + FLAGS_input_image_path);
  }
}
