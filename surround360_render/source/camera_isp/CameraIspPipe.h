/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include "CameraIsp.h"
#include "CameraIspGen8.h"
#include "CameraIspGenFast8.h"
#include "CameraIspGen16.h"
#include "CameraIspGenFast16.h"
#include "Halide.h"

namespace surround360 {

using namespace std;
using namespace cv;
using namespace surround360::color;
using namespace surround360::util;
using namespace Halide;

// Halide accelerated version of the ISP
class CameraIspPipe : public CameraIsp {
 protected:
  Mat inputImage;
  buffer_t inputBufferBp;
  buffer_t outputBufferBp;

  Mat toneCurveTable;
  Mat vignetteCurveTableH;
  Mat vignetteCurveTableV;


  buffer_t ccMatBp;
  buffer_t toneTableBp;
  buffer_t vignetteTableHBp;
  buffer_t vignetteTableVBp;

  const bool fast;

 public:
  CameraIspPipe(string jsonInput,
      const bool fast = false,
      const int outputBpp = 8) :
      CameraIsp(jsonInput, outputBpp),
      fast(fast) {
    memset(&inputBufferBp, 0, sizeof(buffer_t));
    memset(&outputBufferBp, 0, sizeof(buffer_t));
    memset(&ccMatBp, 0, sizeof(buffer_t));
    memset(&toneTableBp, 0, sizeof(buffer_t));
    memset(&vignetteTableHBp, 0, sizeof(buffer_t));
    memset(&vignetteTableVBp, 0, sizeof(buffer_t));

    initPipe();
  }

  // Resets the pipeline's lookup tables used for updating interactive
  // tonemapping, vignetting, and color matrix settings.
  void initPipe() {
    toneCurveTable = Mat(kToneCurveLutSize, 3, outputBpp == 8 ? CV_8U : CV_16U);

    // Convert the tone curve to a 8 bit output table
    for (int i = 0; i < kToneCurveLutSize; ++i) {
      const int r = toneCurveLut[i][0];
      const int g = toneCurveLut[i][1];
      const int b = toneCurveLut[i][2];

      if (outputBpp == 8) {
        toneCurveTable.at<uint8_t>(i, 0) = r;
        toneCurveTable.at<uint8_t>(i, 1) = g;
        toneCurveTable.at<uint8_t>(i, 2) = b;
      } else {
        toneCurveTable.at<uint16_t>(i, 0) = r;
        toneCurveTable.at<uint16_t>(i, 1) = g;
        toneCurveTable.at<uint16_t>(i, 2) = b;
      }
    }

    // Convert the vignetting curves into tables
    vignetteCurveTableH = Mat(width, 3, CV_32F);
    vignetteCurveTableV = Mat(height, 3, CV_32F);

    for (int j = 0;  j < width; ++j) {
      const Vec3f v = curveHAtPixel(j);
      vignetteCurveTableH.at<float>(j, 0) = v[0];
      vignetteCurveTableH.at<float>(j, 1) = v[2];
      vignetteCurveTableH.at<float>(j, 2) = v[1];
    }

    for (int i = 0; i < height; ++i) {
      const Vec3f v = curveVAtPixel(i);
      vignetteCurveTableV.at<float>(i, 0) = v[0];
      vignetteCurveTableV.at<float>(i, 1) = v[1];
      vignetteCurveTableV.at<float>(i, 2) = v[2];
    }

    // Marshal these tables into Halide buffers
    ccMatBp.host = compositeCCM.data;
    ccMatBp.extent[0] = 3;
    ccMatBp.extent[1] = 3;
    ccMatBp.stride[0] = 1;
    ccMatBp.stride[1] = 3;
    ccMatBp.elem_size = sizeof(float);

    toneTableBp.host = toneCurveTable.data;
    toneTableBp.extent[0] = 3;
    toneTableBp.extent[1] = kToneCurveLutSize;
    toneTableBp.stride[0] = 1;
    toneTableBp.stride[1] = 3;
    toneTableBp.elem_size = outputBpp / 8;

    vignetteTableHBp.host = vignetteCurveTableH.data;
    vignetteTableHBp.extent[0] = 3;
    vignetteTableHBp.extent[1] = width;
    vignetteTableHBp.stride[0] = 1;
    vignetteTableHBp.stride[1] = 3;
    vignetteTableHBp.elem_size = sizeof(float);

    vignetteTableVBp.host = vignetteCurveTableV.data;
    vignetteTableVBp.extent[0] = 3;
    vignetteTableVBp.extent[1] = height;
    vignetteTableVBp.stride[0] = 1;
    vignetteTableVBp.stride[1] = 3;
    vignetteTableVBp.elem_size = sizeof(float);
  }

  void runPipe(const bool swizzle) {
    // Call apropos the Halide generated ISP pipeline
    int pattern = 0;
    if (bayerPattern.find("GBRG") != std::string::npos) {
        pattern = 0;
    } else if (bayerPattern.find("RGGB") != std::string::npos) {
        pattern = 1;
    } else {
    }

    if (outputBpp == 8) {
      if (fast) {
        CameraIspGenFast8(
            &inputBufferBp, width, height, &vignetteTableHBp, &vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            clampMin.x, clampMin.y, clampMin.z, clampMax.x, clampMax.y, clampMax.z,
            sharpening.x, sharpening.y, sharpening.z, sharpeningSupport, noiseCore,
            &ccMatBp, &toneTableBp, swizzle, pattern, &outputBufferBp);
      } else {
        CameraIspGen8(
            &inputBufferBp, width, height, &vignetteTableHBp, &vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            clampMin.x, clampMin.y, clampMin.z, clampMax.x, clampMax.y, clampMax.z,
            sharpening.x, sharpening.y, sharpening.z,  sharpeningSupport, noiseCore,
            &ccMatBp, &toneTableBp, swizzle, pattern, &outputBufferBp);
      }
    } else {
      if (fast) {
        CameraIspGenFast16(
            &inputBufferBp, width, height, &vignetteTableHBp, &vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            clampMin.x, clampMin.y, clampMin.z, clampMax.x, clampMax.y, clampMax.z,
            sharpening.x, sharpening.y, sharpening.z,  sharpeningSupport, noiseCore,
            &ccMatBp, &toneTableBp, swizzle, pattern, &outputBufferBp);
      } else {
        CameraIspGen16(
            &inputBufferBp, width, height, &vignetteTableHBp, &vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            clampMin.x, clampMin.y, clampMin.z, clampMax.x, clampMax.y, clampMax.z,
            sharpening.x, sharpening.y, sharpening.z,  sharpeningSupport, noiseCore,
            &ccMatBp, &toneTableBp, swizzle, pattern, &outputBufferBp);
      }
    }
  }

  // Used for streaming of in place updates of input and output images
  // as in live-preview.
  void runPipe(void* inputImageData, void* outputImageData, const bool swizzle) {
    inputBufferBp.host = reinterpret_cast<uint8_t*>(inputImageData);
    outputBufferBp.host = reinterpret_cast<uint8_t*>(outputImageData);
    runPipe(swizzle);
  }

  // Called once to initialize the input image size and data pointer.
  void loadImage(const Mat& inputImage) {
    this->inputImage = inputImage;
    loadImage(inputImage.data, inputImage.cols, inputImage.rows);
  }

  void loadImage(uint8_t* inputImageData, const int xRes, const int yRes) {
    *const_cast<int*>(&width) = xRes;
    *const_cast<int*>(&height) = yRes;
    *const_cast<int*>(&maxDimension) = std::max(width, height);
    *const_cast<float*>(&maxD) = square(width) + square(width);
    *const_cast<float*>(&sqrtMaxD) = sqrt(maxD);
    inputBufferBp.host = inputImageData;
    inputBufferBp.extent[0] = width;
    inputBufferBp.extent[1] = height;
    inputBufferBp.stride[0] = 1;
    inputBufferBp.stride[1] = width;
    inputBufferBp.elem_size = 2;
  }

  // Called at least one to setup the output image size and process
  // the first input image.
  void getImage(uint8_t* outputImageData, const bool swizzle = true) {
    outputBufferBp.host = outputImageData;
    outputBufferBp.elem_size = outputBpp / 8;
    outputBufferBp.extent[0] = width;
    outputBufferBp.extent[1] = height;
    outputBufferBp.extent[2] = 3;
    outputBufferBp.stride[0] = 3;
    outputBufferBp.stride[1] = 3 * width;
    outputBufferBp.stride[2] = 1;

    // Pull the first image through the pipe
    runPipe(swizzle);
  }

  void getImage(Mat& outputImage, const bool swizzle = true) {
    getImage(outputImage.data, swizzle);
  }
};
}
