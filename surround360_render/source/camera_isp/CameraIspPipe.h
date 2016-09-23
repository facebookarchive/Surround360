#pragma once

#include "CameraIsp.h"
#include "CameraIspGen.h"
#include "CameraIspGenFast.h"
#include "Halide.h"
#include "Image.h"

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
  const bool fast_;
  void loadImage(const Mat& inputImage) {
    *const_cast<int*>(&width) = inputImage.cols / resize;
    *const_cast<int*>(&height) = inputImage.rows / resize;

    *const_cast<float*>(&halfWidth) = width / 2.0f;
    *const_cast<float*>(&halfHeight) = height / 2.0f;
    *const_cast<float*>(&maxD) = square(width) + square(width);
    *const_cast<float*>(&sqrtMaxD) = sqrt(maxD);
    this->inputImage = inputImage;
  }

 public:
  CameraIspPipe(string jsonInput,
      const bool fast = false) :
      CameraIsp(jsonInput),
      fast_(fast) {
  }

  void getImage(Mat& outputImage, const bool swizzle = true) {
    Buffer inputBuffer(UInt(16), inputImage.cols, inputImage.rows, 1, 1, inputImage.data);
    buffer_t* inputBufferBp = inputBuffer.raw_buffer();

    const int width = outputImage.cols;
    const int height = outputImage.rows;
    Buffer outputBuffer(UInt(ISP_OBUFFER_BPP), width, height, 3, 1, outputImage.data);
    buffer_t* outputBufferBp = outputBuffer.raw_buffer();
    outputBufferBp->elem_size = ISP_OBUFFER_BPP / 8;
    outputBufferBp->extent[0] = width;
    outputBufferBp->extent[1] = height;
    outputBufferBp->extent[2] = 3;
    outputBufferBp->stride[0] = 3;
    outputBufferBp->stride[1] = 3*width;
    outputBufferBp->stride[2] = 1;

    // The stage following the CCM maps tone curve lut to 256 so we
    // scale the pixel by the lut size here once instead of doing it
    // for every pixel.
    compositeCCM *= float(kToneCurveLutSize);

    // Convert the tone curve to a 8 bit output table
#if ISP_OBUFFER_BPP == 8
    Mat toneCurveTable(kToneCurveLutSize, 3, CV_8U);
#else
    Mat toneCurveTable(kToneCurveLutSize, 3, CV_16U);
#endif

    const float range = float((1 << ISP_OBUFFER_BPP) - 1);
    for (int i = 0; i < kToneCurveLutSize; ++i) {
      for (int j = 0; j < 3; ++j) {
#if ISP_OBUFFER_BPP == 8
        toneCurveTable.at<uint8_t>(i, j) = clamp(toneCurveLut[i][j] * range, 0.0f, range);
#else
        toneCurveTable.at<uint16_t>(i, j) = clamp(toneCurveLut[i][j] * range, 0.0f, range);
#endif
      }
    }

    // Convert the vignetting curves into tables
    Mat vignetteCurveTableH(width, 3, CV_32F);
    Mat vignetteCurveTableV(height, 3, CV_32F);

    for (int j = 0;  j < width; ++j) {
      Point3f v = (*vignetteCurve)(std::abs(j - halfWidth)  / halfWidth);

      vignetteCurveTableH.at<float>(j, 0) = v.x;
      vignetteCurveTableH.at<float>(j, 1) = v.y;
      vignetteCurveTableH.at<float>(j, 2) = v.z;
    }

    for (int i = 0; i < height; ++i) {
      const Point3f v = (*vignetteCurve)(std::abs(i - halfHeight) / halfHeight);

      vignetteCurveTableV.at<float>(i, 0) = v.x;
      vignetteCurveTableV.at<float>(i, 1) = v.y;
      vignetteCurveTableV.at<float>(i, 2) = v.z;
    }

    // Marshal these tables into Halide buffers
    Buffer ccMat(Float(32), 3, 3, 1, 1, compositeCCM.data);
    Buffer toneTable(UInt(ISP_OBUFFER_BPP), 3, kToneCurveLutSize, 1, 1, toneCurveTable.data);
    Buffer vignetteTableH(Float(32), 3, width, 1, 1, vignetteCurveTableH.data);
    Buffer vignetteTableV(Float(32), 3, height, 1, 1, vignetteCurveTableV.data);

    buffer_t* ccMatBp = ccMat.raw_buffer();
    buffer_t* toneTableBp = toneTable.raw_buffer();
    buffer_t* vignetteTableHBp = vignetteTableH.raw_buffer();
    buffer_t* vignetteTableVBp = vignetteTableV.raw_buffer();

    // Call apropos the Halide generated ISP pipeline
    if (fast_) {
      CameraIspGenFast(
          inputBufferBp, width, height, denoise, denoiseRadius, vignetteTableHBp, vignetteTableVBp,
          blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
          sharpenning.x, sharpenning.y, sharpenning.z, ccMatBp, toneTableBp, swizzle, outputBufferBp);
    } else {
      CameraIspGen(
          inputBufferBp, width, height, denoise, denoiseRadius, vignetteTableHBp, vignetteTableVBp,
          blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
          sharpenning.x, sharpenning.y, sharpenning.z, ccMatBp, toneTableBp, swizzle, outputBufferBp);
    }
  }
};
}
