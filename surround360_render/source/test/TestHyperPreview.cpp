/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "CameraMetadata.h"
#include "CvUtil.h"
#include "ImageWarper.h"
#include "MathUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace cv;
using namespace std;
using namespace surround360;
using namespace surround360::calibration;
using namespace surround360::math_util;
using namespace surround360::util;
using namespace surround360::warper;

DEFINE_int32(image_width,           2048,       "expected image width");
DEFINE_int32(image_height,          2048,       "expected image height");
DEFINE_string(binary_prefix,        "",         "path to binary image disk up to timestamp_ (i.e. before 0,1)");
DEFINE_int32(disk_count,            2,          "number of consumer threads");
DEFINE_int32(start_frame,           0,          "start frame (per camera)");
DEFINE_int32(frame_count,           0,          "number of frames to unpack (per camera)");
DEFINE_string(rig_json_file,        "",         "path to json file drescribing camera array");
DEFINE_int32(eqr_width,             2048,       "height of spherical projection image (0 to 2pi)");
DEFINE_int32(eqr_height,            1024,        "height of spherical projection image (0 to pi)");
DEFINE_string(preview_dest,         "",         "path to write equirect preview frames");
DEFINE_double(gamma,                1.0,        "gamma correction exponent");
DEFINE_int32(top_cam_index,         0,          "index of the top camera");
DEFINE_int32(bottom_cam_index,      15,         "index of the primary bottom camera");
DEFINE_int32(bottom_cam2_index,     16,         "index of the secondary bottom camera");
DEFINE_int32(enable_pole_removal,   true,       "if true, the secondary bottom camera is used to remove the pole in the primary bottom camera image");

struct PreviewRenderer {

  Mat topImage, bottomImage, bottomImage2;
  CameraMetadata topCamModel, bottomCamModel;
  Mat fisheyeWarpMatTop, fisheyeWarpMatBottom;
  cv::Size outputSize;

  PreviewRenderer() {

    LOG(INFO) << "reading camera model json";
    float cameraRingRadius;
    vector<CameraMetadata> camModelArrayWithTop =
      readCameraProjectionModelArrayFromJSON(
        FLAGS_rig_json_file,
        cameraRingRadius);
    topCamModel = getTopCamModel(camModelArrayWithTop);
    bottomCamModel = getBottomCamModel(camModelArrayWithTop);

    // the simple/fast ISP we use for preview reduces image size by half,
    // so we will need to account for that when projecting from fisheye to
    // equirectangular.
    topCamModel.usablePixelsRadius      /= 2;
    topCamModel.imageCenterX            /= 2;
    topCamModel.imageCenterY            /= 2;
    bottomCamModel.usablePixelsRadius   /= 2;
    bottomCamModel.imageCenterX         /= 2;
    bottomCamModel.imageCenterY         /= 2;

    outputSize = cv::Size(
      FLAGS_eqr_width,
      FLAGS_eqr_height * (topCamModel.fisheyeFovDegrees / 2.0f) / 180.0f);
    fisheyeWarpMatTop = precomputeBicubicRemapFisheyeToSpherical(
      topCamModel, outputSize);
    fisheyeWarpMatBottom = precomputeBicubicRemapFisheyeToSpherical(
      bottomCamModel, outputSize);
  }

  void addTopImage(const Mat& image) {
    topImage = image.clone();
  }

  void addBottomImage(const Mat& image) {
    bottomImage = image.clone();
  }

  void addBottomImage2(const Mat& image) {
    bottomImage2 = image.clone();
  }

  Mat makePaddedEquirect(
      const Mat& warpMat,
      const Mat& fisheyeImage,
      const bool isBottom) {

    Mat eqrImage(outputSize, CV_8UC3);
    remap(
      fisheyeImage,
      eqrImage,
      warpMat,
      Mat(),
      CV_INTER_CUBIC,
      BORDER_CONSTANT);

    cvtColor(eqrImage, eqrImage, CV_BGR2BGRA);
    if (isBottom) {
      flip(eqrImage, eqrImage, -1);

      // do an alpha ramp to blend the bottom camera image against the top
      const int halfEqrHeight = FLAGS_eqr_height / 2;
      const int alphaRampHeight = eqrImage.rows - halfEqrHeight;
      for (int y = 0; y < alphaRampHeight; ++y) {
        for (int x = 0; x < eqrImage.cols; ++x) {
          const float alpha = float(y) / float(alphaRampHeight);
          eqrImage.at<Vec4b>(y, x)[3] = alpha * 255.0f;
        }
      }
    }

    copyMakeBorder(
      eqrImage,
      eqrImage,
      isBottom ? FLAGS_eqr_height - eqrImage.rows : 0,
      isBottom ? 0 : FLAGS_eqr_height - eqrImage.rows,
      0,
      0,
      BORDER_CONSTANT);
    return eqrImage;
  }

  void render(int frameNumber) {
    const Mat topBGR = simpleDemosaic(topImage);
    const Mat bottomBGR = simpleDemosaic(bottomImage);
    const Mat bottomBGR2 = simpleDemosaic(bottomImage2);

    Mat topEqr = makePaddedEquirect(fisheyeWarpMatTop, topBGR, false);
    Mat bottomEqr = makePaddedEquirect(fisheyeWarpMatBottom, bottomBGR, true);

    if (FLAGS_enable_pole_removal) {
      // the secondary bottom camera is rotated 180 degrees. we need to fix
      // that, plus only composite in the right half of its image to cover the
      // pole in the primary bottom camera image.
      Mat bottomEqr2 = makePaddedEquirect(
        fisheyeWarpMatBottom, bottomBGR2, true);
      const static float kHorizontalRampFrac = 0.05f;
      const int horizontalRampSize = bottomEqr.cols * kHorizontalRampFrac;
      for (int y = 0; y < bottomEqr.rows; ++y) {
        for (int x = bottomEqr.cols / 2; x < bottomEqr.cols; ++x) {
          const Vec4b baseColor = bottomEqr.at<Vec4b>(y, x);
          const Vec4b bottomColor2 =
            bottomEqr2.at<Vec4b>(y, x - bottomEqr.cols / 2);
          const float alphaRamp =
            rampf(x - bottomEqr.cols / 2, 0, horizontalRampSize);
          bottomEqr.at<Vec4b>(y, x) = Vec4b(
            lerp(baseColor[0], bottomColor2[0], alphaRamp),
            lerp(baseColor[1], bottomColor2[1], alphaRamp),
            lerp(baseColor[2], bottomColor2[2], alphaRamp),
            bottomColor2[3]);
        }
        for (int x = 0; x < horizontalRampSize; ++x) {
          const Vec4b baseColor = bottomEqr.at<Vec4b>(y, x);
          const Vec4b bottomColor2 =
            bottomEqr2.at<Vec4b>(y, x + bottomEqr.cols / 2);
          const float alphaRamp = 1.0f - rampf(x, 0, horizontalRampSize);
          bottomEqr.at<Vec4b>(y, x) = Vec4b(
            lerp(baseColor[0], bottomColor2[0], alphaRamp),
            lerp(baseColor[1], bottomColor2[1], alphaRamp),
            lerp(baseColor[2], bottomColor2[2], alphaRamp),
            bottomColor2[3]);
        }
      }
    }

    Mat eqrImage = flattenLayers<Vec4b>(topEqr, bottomEqr);

    stringstream ss;
    ss << std::setw(6) << std::setfill('0') << frameNumber;
    const string outFilename = FLAGS_preview_dest + "/" + ss.str() + ".jpg";
    imwriteExceptionOnFail(outFilename, eqrImage);
  }

  static Mat simpleDemosaic(const Mat& src) {
    Mat dest(src.size() / 2, CV_8UC3);
    for (int y = 0; y < dest.rows; ++y) {
      for (int x = 0; x < dest.cols; ++x) {
        float g1 = src.at<unsigned char>(y * 2, x * 2) / 255.0f;
        float g2 = src.at<unsigned char>(y * 2 + 1, x * 2 + 1) / 255.0f;
        float b = src.at<unsigned char>(y * 2, x * 2 + 1) / 255.0f;
        float r = src.at<unsigned char>(y * 2 + 1, x * 2) / 255.0f;
        float g = (g1 + g2) / 2.0f;
        r = powf(r, FLAGS_gamma);
        g = powf(g, FLAGS_gamma);
        b = powf(b, FLAGS_gamma);
        dest.at<Vec3b>(y, x) = Vec3b(b * 255.0f, g * 255.0f, r * 255.0f);
      }
    }
    return dest;
  }
};

vector<string> readCameraNamesFile() {
  const string camera_names_path = FLAGS_binary_prefix + "cameranames.txt";
  ifstream cameraNamesFile(camera_names_path);
  if (!cameraNamesFile) {
    throw VrCamException("file read failed:" + camera_names_path);
  }

  string line;
  vector<string> cameraNames;
  while (std::getline(cameraNamesFile, line)) {
    cameraNames.push_back(line);
  }
  cameraNamesFile.close();
  return cameraNames;
}

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_binary_prefix, "binary_prefix");
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_rig_json_file, "preview_dest");

  vector<string> cameraNames = readCameraNamesFile();
  vector<string> sortedCameraNames(cameraNames);
  sort(sortedCameraNames.begin(), sortedCameraNames.end());

  const size_t imageSizeBytes = FLAGS_image_width * FLAGS_image_height;
  const int cameraCount = cameraNames.size();
  int totalImageCount = FLAGS_frame_count * cameraCount;

  int binaryFileHandles[FLAGS_disk_count];
  int imageCountForFile[FLAGS_disk_count];
  size_t readCount[FLAGS_disk_count];

  LOG(INFO) << "Opening binary files and seeking to start frame...";
  for (int diskIdx = 0; diskIdx < FLAGS_disk_count; ++diskIdx) {
    string fileName(FLAGS_binary_prefix + to_string(diskIdx) + ".bin");
    binaryFileHandles[diskIdx] = open(fileName.c_str(), O_RDONLY);
    if (binaryFileHandles[diskIdx] < 0) {
      throw VrCamException(
        "error opening binary file. err: " + string(strerror(errno)) +
        " filename: " + fileName);
    }

    if (FLAGS_frame_count == 0) {
      // update frame count to all the available frames
      const size_t bytesInFile = lseek(
        binaryFileHandles[diskIdx], 0, SEEK_END) + 1;
      imageCountForFile[diskIdx] = bytesInFile / imageSizeBytes;
      lseek(binaryFileHandles[diskIdx], 0, SEEK_SET);

      VLOG(1) << "Total frame count binary " << diskIdx << ": "
        << imageCountForFile[diskIdx];
    }

    // posix_fadvices speeds things up in Linux, but doesn't work in Darwin (Mac OS X)
#if _XOPEN_SOURCE >= 600 || _POSIX_C_SOURCE >= 200112L
    const size_t fileSizeStart = FLAGS_start_frame * cameraCount * imageSizeBytes;
    const int numFrames = FLAGS_frame_count == 0 ? imageCountForFile[diskIdx] : (FLAGS_frame_count * cameraCount);
    const int fileSize = numFrames * imageSizeBytes;
    posix_fadvise(binaryFileHandles[diskIdx], fileSizeStart, fileSize, POSIX_FADV_DONTNEED);
    posix_fadvise(binaryFileHandles[diskIdx], fileSizeStart, fileSize, POSIX_FADV_SEQUENTIAL);
#endif

    // move pointer to start frame
    int cameraCountOnDisk = cameraCount / FLAGS_disk_count;
    if (diskIdx < cameraCount % FLAGS_disk_count) {
      ++cameraCountOnDisk;
    }
    const int numImagesSkip = FLAGS_start_frame * cameraCountOnDisk;
    lseek(binaryFileHandles[diskIdx], numImagesSkip * imageSizeBytes, SEEK_SET);

    if (FLAGS_frame_count == 0) {
      totalImageCount += imageCountForFile[diskIdx];
    }

    readCount[diskIdx] = -1;
  }

  LOG(INFO) << "Generating previews...";
  PreviewRenderer previewRenderer;
  Mat outImage(FLAGS_image_height, FLAGS_image_width, CV_8U);
  void* outputPtr = outImage.ptr(0);
  const int lastFrame = FLAGS_start_frame * cameraCount + totalImageCount - 1;
  int percentDonePrev = 0;
  bool isDone = false;
  for (int frameNumber = FLAGS_start_frame; frameNumber < FLAGS_start_frame + totalImageCount / cameraCount; ++frameNumber) {
    for (unsigned int cameraNumber = 0; cameraNumber < cameraCount; ++cameraNumber) {
      const int imageIndex = frameNumber * cameraCount + cameraNumber;
      const int did = cameraNumber % FLAGS_disk_count;
      readCount[did] = read(binaryFileHandles[did], outputPtr, imageSizeBytes);

      // check if we reached EOF (read returns 0)
      if (readCount[did] == 0) {
        // check if all the files have reached EOF
        if (!std::all_of(readCount, readCount + FLAGS_disk_count, [](int x){ return x == 0; })) {
          continue;
        }

        isDone = true;
        LOG(WARNING) << "Reached EOF";
        break;
      }

      if (imageIndex % 10 == 0 || imageIndex == lastFrame) {
        int percentDoneCurr = imageIndex * 100 / lastFrame;
        LOG_IF(INFO, percentDoneCurr != percentDonePrev) << "Percent done "
          << percentDoneCurr << "%";
        percentDonePrev = percentDoneCurr;
      }

      if (cameraNames[cameraNumber] == sortedCameraNames[FLAGS_top_cam_index]) {
        previewRenderer.addTopImage(outImage);
      }
      if (cameraNames[cameraNumber] == sortedCameraNames[FLAGS_bottom_cam_index]) {
        previewRenderer.addBottomImage(outImage);
      }
      if (cameraNames[cameraNumber] == sortedCameraNames[FLAGS_bottom_cam2_index]) {
        previewRenderer.addBottomImage2(outImage);
      }
    }

    previewRenderer.render(frameNumber);

    if (isDone) {
      break;
    }
  }

  LOG(INFO) << "Closing binary files...";
  for (int i = 0; i < FLAGS_disk_count; ++i) {
    close(binaryFileHandles[i]);
  }

  return EXIT_SUCCESS;
}
