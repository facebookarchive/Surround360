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
#include "DngTags.h"
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
DEFINE_string(output_dng_path,      "",                     "output a DNG version of the raw file.");
DEFINE_string(isp_config_path,      "",                     "ISP configuration file path");
DEFINE_int32(black_level_offset,    0,                      "amount to add to the blacklevel in the config file");
DEFINE_int32(demosaic_filter,       EDGE_AWARE_DM_FILTER,   "Demosaic filter type: 0=Bilinear(fast), 1=Frequency, 2=Edge aware");
DEFINE_int32(resize,                1,                      "Amount to \"bin-down\" the input. Legal values are 1, 2, 4, and 8");
DEFINE_int32(output_bpp,            8,                      "output image bits per pixel, either 8 or 16");
#ifdef USE_HALIDE
DEFINE_bool(accelerate,             false,                  "Use halide accelerated version");
DEFINE_bool(fast,                   false,                  "Use fastest halide for realtime apps or previews");
#endif
DEFINE_bool(disable_tone_curve,     false,                  "By default tone curve is enabled");

// We really want all ISP input bits to fill 16 bits
const int kIspInputBitsPerPixel = 16;

void writeIfd(
    const uint16_t tag,
    const uint16_t type,
    const uint32_t count,
    const uint32_t offset,
    const uint32_t offsetInc,
    uint32_t &dOffset,
    FILE* fDng) {
  // Write out an tiff directory entry aka an ifd
  uint32_t writeStatus;

  fwrite(&tag, sizeof(tag), 1, fDng);
  fwrite(&type, sizeof(type), 1, fDng);
  fwrite(&count, sizeof(count), 1, fDng);
  fwrite(&offset, sizeof(offset), 1, fDng);

  dOffset += offsetInc;
}

void writeDng(
    CameraIsp& cameraIsp,
    const string& filename,
    Mat inputImage,
    Mat outputImage) {
  // Write out a dng if one was specified
  if (filename.size() > 0) {
    LOG(INFO) << "Writing: " << filename << endl;

    FILE *fDng = fopen(filename.c_str(),"w");
    if (fDng == NULL)    {
      return;
    }

    uint32_t width = outputImage.cols;
    uint32_t height = outputImage.rows;

    // TIFF data layout calculations (64k strip for 16bit data)
    uint16_t rowsPerStrip = (32 * 1024) / width;
    uint16_t stripsPerImg = (height / rowsPerStrip);
    if (height % rowsPerStrip != 0) {
      stripsPerImg++;
    }

    const std::string cameraManufacturer("Facebook");
    const std::string cameraModel("Surround 360");
    const std::string cameraSoftware("Raw2Rgb");

    // Write the TIFF file header
    const char byteOrder[3] = "II";
    fwrite(byteOrder, sizeof(char), 2, fDng);

    const uint16_t version = 42;
    fwrite(&version, sizeof(uint16_t), 1, fDng);

    const uint32_t Idf0Offset = 0x00000008;
    fwrite(&Idf0Offset, sizeof(unsigned), 1, fDng);

    const uint16_t ifdCount = 42;
    fwrite(&ifdCount, sizeof(uint16_t), 1, fDng);

    // Map Surround's ISP cfa pattern code to DNG's
    uint32_t cfaFilter;

    switch(cameraIsp.getFilters()) {
      case 0x94949494:
        cfaFilter = 0x02010100;
        break;
      case 0x16161616:
        cfaFilter = 0x00010102;
        break;
      case 0x49494949:
        cfaFilter = 0x01000201;
        break;
      case 0x61616161:
        cfaFilter = 0x01020001;
        break;
      default:
        LOG(ERROR) << "Unknown bayer-pattern found while writing DNG file" << endl;
        fclose(fDng);
        return;
    }

    // Write the tags
    const uint32_t kIfdEntrySize = sizeof(uint16_t) * 2 + sizeof(uint32_t) * 2;
    uint32_t dOffset = 10 + ifdCount * kIfdEntrySize + 4;

    writeIfd(kTiffTagNewSubFileType,            kTiffTypeLONG,      1, 0, 0, dOffset, fDng);
    writeIfd(kTiffTagImageWidth,                kTiffTypeLONG,      1, width, 0, dOffset, fDng);
    writeIfd(kTiffTagImageLength,               kTiffTypeLONG,      1, height, 0, dOffset, fDng);
    writeIfd(kTiffTagBitsPerSample,             kTiffTypeSHORT,     1, cameraIsp.getBitsPerPixel(), 0, dOffset, fDng);
    writeIfd(kTiffTagCompression,               kTiffTypeSHORT,     1, 1, 0, dOffset, fDng);
    writeIfd(kTiffTagPhotometricInterpretation, kTiffTypeSHORT,     1, 32803, 0, dOffset, fDng);
    writeIfd(kTiffTagMake,                      kTiffTypeASCII,     cameraManufacturer.size() + 1, dOffset, cameraManufacturer.size() + 1, dOffset, fDng);
    writeIfd(kTiffTagModel,                     kTiffTypeASCII,     cameraModel.size() + 1, dOffset, cameraModel.size() + 1, dOffset, fDng);
    writeIfd(kTiffTagStripOffsets,              kTiffTypeLONG,      stripsPerImg, dOffset, stripsPerImg * 4, dOffset, fDng);
    writeIfd(kTiffTagOrientation,               kTiffTypeSHORT,     1, 1, 0, dOffset, fDng);
    writeIfd(kTiffTagSamplesPerPixel,           kTiffTypeSHORT,     1, 1, 0, dOffset, fDng);
    writeIfd(kTiffTagRowsPerStrip,              kTiffTypeSHORT,     1, rowsPerStrip, 0, dOffset, fDng);
    writeIfd(kTiffTagStripByteCounts,           kTiffTypeSHORT,     stripsPerImg, dOffset, stripsPerImg * 2, dOffset, fDng);
    writeIfd(kTiffTagPlanarConfiguration,       kTiffTypeSHORT,     1, 1, 0, dOffset, fDng);
    writeIfd(kTiffTagResolutionUnit,            kTiffTypeSHORT,     1, 2, 0, dOffset, fDng);
    writeIfd(kTiffTagSoftware,                  kTiffTypeASCII,     cameraSoftware.size()+1, dOffset, cameraSoftware.size()+1, dOffset, fDng);
    writeIfd(kTiffTagDateTime,                  kTiffTypeASCII,     20, dOffset, 20, dOffset, fDng);
    writeIfd(kTiffEpTagCFARepeatPatternDim,     kTiffTypeSHORT,     2, 0x00020002, 0, dOffset, fDng);
    writeIfd(kTiffEpTagCFAPattern,              kTiffTypeBYTE,      4, cfaFilter, 0, dOffset, fDng);
    writeIfd(kDngTagDNGVersion,                 kTiffTypeBYTE,      4, 0x00000301, 0, dOffset, fDng);
    writeIfd(kDngTagDNGBackwardVersion,         kTiffTypeBYTE,      4, 0x00000101, 0, dOffset, fDng);
    writeIfd(kDngTagUniqueCameraModel,          kTiffTypeASCII,     cameraModel.size()+1, dOffset, cameraModel.size()+1, dOffset, fDng);
    writeIfd(kDngTagLocalizedCameraModel,       kTiffTypeASCII,     cameraModel.size()+1, dOffset, cameraModel.size()+1, dOffset, fDng);
    writeIfd(kDngTagCFAPlaneColor,              kTiffTypeBYTE,      3, 0x00020100, 0, dOffset, fDng);
    writeIfd(kDngTagCFALayout,                  kTiffTypeSHORT,     1, 1, 0, dOffset, fDng);
    writeIfd(kDngTagBlackLevelRepeatDim,        kTiffTypeSHORT,     2, 0x00020002, 0, dOffset, fDng);
    writeIfd(kDngTagBlackLevel,                 kTiffTypeSHORT,     4, dOffset, 8, dOffset, fDng);
    writeIfd(kDngTagWhiteLevel,                 kTiffTypeLONG,      1, (1 << cameraIsp.getBitsPerPixel()) - 1, 0, dOffset, fDng);
    writeIfd(kDngTagDefaultScale,               kTiffTypeRATIONAL,  2, dOffset, 16, dOffset, fDng);
    writeIfd(kDngTagDefaultCropOrigin,          kTiffTypeSHORT,     2, 0, 0, dOffset, fDng);
    writeIfd(kDngTagDefaultCropSize,            kTiffTypeSHORT,     2, (height << 16) | width, 0, dOffset, fDng);
    writeIfd(kDngTagColorMatrix1,               kTiffTypeSRATIONAL, 9, dOffset, 9 * 8, dOffset, fDng);
    writeIfd(kDngTagAnalogBalance,              kTiffTypeRATIONAL,  3, dOffset, 3 * 8, dOffset, fDng);
    writeIfd(kDngTagAsShotNeutral,              kTiffTypeRATIONAL,  3, dOffset, 3 * 8, dOffset, fDng);
    writeIfd(kDngTagBaselineExposure,           kTiffTypeSRATIONAL, 1, dOffset, 8, dOffset, fDng);
    writeIfd(kDngTagBaselineSharpness,          kTiffTypeRATIONAL,  1, dOffset, 8, dOffset, fDng);
    writeIfd(kDngTagBayerGreenSplit,            kTiffTypeLONG,      1, 0, 0, dOffset, fDng);
    writeIfd(kDngTagLinearResponseLimit,        kTiffTypeRATIONAL,  1, dOffset, 8, dOffset, fDng);
    writeIfd(kDngTagLensInfo,                   kTiffTypeRATIONAL,  4, dOffset, 32, dOffset, fDng);
    writeIfd(kDngTagAntiAliasStrength,          kTiffTypeRATIONAL,  1, dOffset, 8, dOffset, fDng);
    writeIfd(kDngTagCalibrationIlluminant1,     kTiffTypeSHORT,     1, 23, 0, dOffset, fDng);
    writeIfd(kDngTagBestQualityScale,           kTiffTypeRATIONAL,  1, dOffset, 8, dOffset, fDng);

    // Now write data fields
    uint32_t firstIFD = 0x00000000;

    fwrite(&firstIFD, sizeof(uint32_t), 1, fDng);
    fwrite(cameraManufacturer.c_str(), sizeof(char), cameraManufacturer.size()+1, fDng);
    fwrite(cameraModel.c_str(), sizeof(char), cameraModel.size()+1, fDng);

    vector<uint32_t> stripOff;
    stripOff.push_back(dOffset);      // where image data will be written...
    for (int16_t s = 1; s < stripsPerImg; ++s) {
      stripOff.push_back(stripOff[s-1] + (rowsPerStrip * width)*2);
    }

    fwrite(&stripOff[0], sizeof(unsigned), stripsPerImg, fDng);

    vector<uint16_t> stripCnt;
    int nRowsLeft = height;
    uint32_t t = 0;
    while (nRowsLeft > 0) {
      if ((unsigned)nRowsLeft > rowsPerStrip) {
        stripCnt.push_back(rowsPerStrip * width*2);
      } else {
         stripCnt.push_back(nRowsLeft * width*2);
      }
      t++;
      nRowsLeft -= rowsPerStrip;
    }

    fwrite(&stripCnt[0], sizeof(uint16_t), stripsPerImg, fDng);
    fwrite(cameraSoftware.c_str(), sizeof(char), cameraSoftware.size()+1, fDng);

    char szDateTime[20];
    time_t time = 0; // Need to get this from the camera meta data.
    struct tm *tlocal = localtime(&time);
    snprintf( szDateTime,
        20,
        "%04d-%02d-%02d %02d:%02d:%02d",
        tlocal->tm_year+1900,
        tlocal->tm_mon,
        tlocal->tm_mday,
        tlocal->tm_hour,
        tlocal->tm_min,
        tlocal->tm_sec);
    szDateTime[19] = '\0';

    fwrite(szDateTime, sizeof(char), 20, fDng);
    fwrite(cameraModel.c_str(), sizeof(char), cameraModel.size()+1, fDng);
    fwrite(cameraModel.c_str(), sizeof(char), cameraModel.size()+1, fDng);

    // Conversion to XYZ - Bradford adapted using D50 reference white point
    Mat sRgbToXyzD50(3, 3, CV_32F);
    sRgbToXyzD50.at<float>(0, 0) = 0.4360747;
    sRgbToXyzD50.at<float>(0, 1) = 0.3850649;
    sRgbToXyzD50.at<float>(0, 2) = 0.1430804;

    sRgbToXyzD50.at<float>(1, 0) = 0.2225045;
    sRgbToXyzD50.at<float>(1, 1) = 0.7168786;
    sRgbToXyzD50.at<float>(1, 2) = 0.0606169;

    sRgbToXyzD50.at<float>(2, 0) = 0.0139322;
    sRgbToXyzD50.at<float>(2, 1) = 0.0971045;
    sRgbToXyzD50.at<float>(2, 2) = 0.7141733;

    Mat ccm = cameraIsp.getCCM();
    Mat camToXyz = ccm * sRgbToXyzD50;
    Mat xyzToCam;
    invert(camToXyz, xyzToCam);

    uint16_t uBlackLevel[4]; // {G, R, B, G}
    Point3f bl = cameraIsp.getBlackLevel();
    uBlackLevel[0] = bl.y;
    uBlackLevel[3] = bl.y;
    uBlackLevel[1] = bl.x;
    uBlackLevel[2] = bl.z;
    fwrite(uBlackLevel, sizeof(uint16_t), 4, fDng);

    uint32_t defaultScale[4];
    defaultScale[0] = 1;  defaultScale[2] = 1;
    defaultScale[1] = 1;  defaultScale[3] = 1;

    fwrite(defaultScale, sizeof(unsigned), 4, fDng);

    int colorMatrix[18];
    for (int i = 0; i < 9; i++) {
      const int x = i % 3;
      const int y = i / 3;
      colorMatrix[2*i] = xyzToCam.at<float>(y, x) * (1 << 28);
      colorMatrix[2*i+1] = (1 << 28);
    }
    fwrite(colorMatrix, sizeof(int), 18, fDng);

    uint32_t analogBalance[6];
    analogBalance[0] = 256; analogBalance[1] = 256;
    analogBalance[2] = 256; analogBalance[3] = 256;
    analogBalance[4] = 256; analogBalance[5] = 256;

    fwrite(analogBalance, sizeof(unsigned), 6, fDng);

    // kDngTagAsShotNeutral
    Point3f whitePoint = cameraIsp.getWhiteBalanceGain();
    uint32_t asShotNeutral[6];
    float minChannel = min(min(whitePoint.x, whitePoint.y), whitePoint.z);
    asShotNeutral[0] = (minChannel / whitePoint.x) * float(1 << 28); asShotNeutral[1] = 1 << 28;
    asShotNeutral[2] = (minChannel / whitePoint.y) * float(1 << 28); asShotNeutral[3] = 1 << 28;
    asShotNeutral[4] = (minChannel / whitePoint.z) * float(1 << 28); asShotNeutral[5] = 1 << 28;

    fwrite(asShotNeutral, sizeof(uint32_t), 6, fDng);

    int32_t baseExposure[2];

    baseExposure[0] = -log2f(1.0f / minChannel) * (1 << 28);
    baseExposure[1] = (1 << 28);

    fwrite(baseExposure, sizeof(unsigned), 2, fDng);

    uint32_t baseSharp[2];

    baseSharp[0] = 1;
    baseSharp[1] = 1;

    fwrite(baseSharp, sizeof(unsigned), 2, fDng);

    uint32_t linearLimit[2];
    linearLimit[0] = 1;
    linearLimit[1] = 1;      // 1.0 = sensor is linear

    fwrite(linearLimit, sizeof(unsigned), 2, fDng);

    uint32_t lensInfo[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    fwrite(lensInfo, sizeof(unsigned), 8, fDng);

    uint32_t antiAlias[2];
    antiAlias[0] = 0;
    antiAlias[1] = 1;        // Turn off antiAliasStrength

    fwrite(antiAlias, sizeof(unsigned), 2, fDng);

    uint32_t bestScale[2];
    bestScale[0] = 1;
    bestScale[1] = 1;        // use 1:1 scaling

    fwrite(bestScale, sizeof(unsigned), 2, fDng);

    // Raw image data
    const size_t status = fwrite(inputImage.data, sizeof(uint16_t), width*height, fDng);

    if (status != width*height) {
      LOG(ERROR) << "DNG write error: image data" << endl;
    }

    fclose(fDng);
  }
}

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
    : imreadExceptionOnFail(
        FLAGS_input_image_path,
        CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);

  if (inputImage.cols > 2 && inputImage.rows > 2) {
    const uint8_t depth = inputImage.type() & CV_MAT_DEPTH_MASK;

    // Make all the input data S16
    Mat inputImage16(inputImage.rows, inputImage.cols, CV_16U);

    if (depth == CV_8U) {
      LOG(INFO) << "8 bit raw" << endl;
      inputImage16 = convert8bitTo16bit(inputImage);
    } else if (depth == CV_16U) {
      LOG(INFO) << "16 bit raw" << endl;
      inputImage16 = inputImage;
    } else {
      throw VrCamException("input is larger that 16 bits per pixel");
    }

    const int width = inputImage.cols / FLAGS_resize;
    const int height = inputImage.rows / FLAGS_resize;
    Mat outputImage(height, width, FLAGS_output_bpp == 8 ? CV_8UC3 : CV_16UC3);

#ifdef USE_HALIDE
    if (FLAGS_accelerate) {
      CameraIspPipe cameraIsp(json, FLAGS_fast, FLAGS_output_bpp);
      cameraIsp.setBitsPerPixel(kIspInputBitsPerPixel);
      if (FLAGS_disable_tone_curve) {
        cameraIsp.disableToneMap();
      } else {
        cameraIsp.enableToneMap();
      }

      cameraIsp.addBlackLevelOffset(FLAGS_black_level_offset);
      cameraIsp.loadImage(inputImage16);
      cameraIsp.initPipe();
      runPipeline(&cameraIsp, inputImage16, outputImage, FLAGS_output_image_path);
      writeDng(cameraIsp, FLAGS_output_dng_path, inputImage16, outputImage);
    } else {
#else
      {
#endif
        CameraIsp cameraIsp(json, FLAGS_output_bpp);
        cameraIsp.setBitsPerPixel(kIspInputBitsPerPixel);
        cameraIsp.setDemosaicFilter(FLAGS_demosaic_filter);
        cameraIsp.setResize(FLAGS_resize);
        if (FLAGS_disable_tone_curve) {
          cameraIsp.disableToneMap();
        } else {
          cameraIsp.enableToneMap();
        }
        cameraIsp.addBlackLevelOffset(FLAGS_black_level_offset);
        cameraIsp.loadImage(inputImage16);
        runPipeline(&cameraIsp, inputImage16, outputImage, FLAGS_output_image_path);
        writeDng(cameraIsp, FLAGS_output_dng_path, inputImage16, outputImage);
      }
    } else {
      throw VrCamException("Unable to open " + FLAGS_input_image_path);
    }
}
