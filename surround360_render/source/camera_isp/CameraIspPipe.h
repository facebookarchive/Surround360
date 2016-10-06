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
  shared_ptr<Buffer> inputBuffer;
  buffer_t* inputBufferBp;
  shared_ptr<Buffer> outputBuffer;
  buffer_t* outputBufferBp;
  Mat toneCurveTable;
  Mat vignetteCurveTableH;
  Mat vignetteCurveTableV;

  shared_ptr<Buffer> ccMat;
  shared_ptr<Buffer> toneTable;
  shared_ptr<Buffer> vignetteTableH;
  shared_ptr<Buffer> vignetteTableV;

  buffer_t* ccMatBp;
  buffer_t* toneTableBp;
  buffer_t* vignetteTableHBp;
  buffer_t* vignetteTableVBp;

  const bool fast;

  void loadImage(const Mat& inputImage) {
    *const_cast<int*>(&width) = inputImage.cols / resize;
    *const_cast<int*>(&height) = inputImage.rows / resize;

    *const_cast<float*>(&halfWidth) = width / 2.0f;
    *const_cast<float*>(&halfHeight) = height / 2.0f;
    *const_cast<float*>(&maxD) = square(width) + square(width);
    *const_cast<float*>(&sqrtMaxD) = sqrt(maxD);
    this->inputImage = inputImage;

    inputBuffer = shared_ptr<Buffer>(new Buffer(UInt(16), width, height, 1, 1, inputImage.data));
    inputBufferBp = inputBuffer->raw_buffer();
  }

  void loadImage(uint8_t* inputImageData, const int xRes, const int yRes) {
    *const_cast<int*>(&width) = xRes;
    *const_cast<int*>(&height) = yRes;

    inputBuffer = shared_ptr<Buffer>(new Buffer(UInt(16), width, height, 1, 1, inputImageData));
    inputBufferBp = inputBuffer->raw_buffer();
    *const_cast<float*>(&halfWidth) = width / 2.0f;
    *const_cast<float*>(&halfHeight) = height / 2.0f;
    *const_cast<float*>(&maxD) = square(width) + square(width);
    *const_cast<float*>(&sqrtMaxD) = sqrt(maxD);
  }

 public:
  CameraIspPipe(string jsonInput,
      const bool fast = false,
      const int outputBpp = 8) :
      CameraIsp(jsonInput, outputBpp),
      fast(fast) {
  }

  void runPipe(const bool swizzle) {
    // Call apropos the Halide generated ISP pipeline
    if (outputBpp == 8) {
      if (fast) {
        CameraIspGenFast8(
            inputBufferBp, width, height, vignetteTableHBp, vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            sharpenning.x, sharpenning.y, sharpenning.z, ccMatBp, toneTableBp, swizzle, outputBufferBp);
      } else {
        CameraIspGen8(
            inputBufferBp, width, height, vignetteTableHBp, vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            sharpenning.x, sharpenning.y, sharpenning.z, ccMatBp, toneTableBp, swizzle, outputBufferBp);
      }
    } else {
      if (fast) {
        CameraIspGenFast16(
            inputBufferBp, width, height, vignetteTableHBp, vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            sharpenning.x, sharpenning.y, sharpenning.z, ccMatBp, toneTableBp, swizzle, outputBufferBp);
      } else {
        CameraIspGen16(
            inputBufferBp, width, height, vignetteTableHBp, vignetteTableVBp,
            blackLevel.x, blackLevel.y, blackLevel.z, whiteBalanceGain.x, whiteBalanceGain.y, whiteBalanceGain.z,
            sharpenning.x, sharpenning.y, sharpenning.z, ccMatBp, toneTableBp, swizzle, outputBufferBp);
      }
    }
  }

  void runPipe(void* inputImageData, void* outputImageData, const bool swizzle) {
    inputBufferBp->host = reinterpret_cast<uint8_t*>(inputImageData);
    outputBufferBp->host = reinterpret_cast<uint8_t*>(outputImageData);
    runPipe(swizzle);
  }

  void getImage(uint8_t* outputImageData, const bool swizzle = true) {
    outputBuffer = shared_ptr<Buffer>(new Buffer(UInt(outputBpp), width, height, 3, 1, outputImageData));
    outputBufferBp = outputBuffer->raw_buffer();
    outputBufferBp->elem_size = outputBpp / 8;
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

    toneCurveTable = Mat(kToneCurveLutSize, 3, outputBpp == 8 ? CV_8U : CV_16U);

    // Convert the tone curve to a 8 bit output table
    const float range = float((1 << outputBpp) - 1);

    for (int i = 0; i < kToneCurveLutSize; ++i) {
      const int r = int(clamp(toneCurveLut[i][0] * range, 0.0f, range));
      const int g = int(clamp(toneCurveLut[i][1] * range, 0.0f, range));
      const int b = int(clamp(toneCurveLut[i][1] * range, 0.0f, range));

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
    Mat vignetteCurveTableH = Mat(width, 3, CV_32F);
    Mat vignetteCurveTableV = Mat(height, 3, CV_32F);

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
    ccMat = shared_ptr<Buffer>(new Buffer(Float(32), 3, 3, 1, 1, compositeCCM.data));
    toneTable = shared_ptr<Buffer>(new Buffer(UInt(outputBpp), 3, kToneCurveLutSize, 1, 1, toneCurveTable.data));
    vignetteTableH = shared_ptr<Buffer>(new Buffer(Float(32), 3, width, 1, 1, vignetteCurveTableH.data));
    vignetteTableV = shared_ptr<Buffer>(new Buffer(Float(32), 3, height, 1, 1, vignetteCurveTableV.data));

    ccMatBp = ccMat->raw_buffer();
    toneTableBp = toneTable->raw_buffer();
    vignetteTableHBp = vignetteTableH->raw_buffer();
    vignetteTableVBp = vignetteTableV->raw_buffer();

    // Pull the first image through the pipe
    runPipe(swizzle);
  }

  void getImage(Mat& outputImage, const bool swizzle = true) {
    getImage(outputImage.data, swizzle);
  }
};
}
