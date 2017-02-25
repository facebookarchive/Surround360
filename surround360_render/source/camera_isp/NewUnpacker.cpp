/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

extern "C" {
#include <sys/stat.h>
#include <sys/types.h>
}

#include <functional>
#include <future>
#include <iomanip>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

#include "BinaryFootageFile.hpp"
#include "CameraIspPipe.h"
#include "Raw12Converter.hpp"
#include "StringUtil.h"
#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace surround360;
using namespace surround360::util;

DEFINE_string(isp_dir,          "",     "directory containing ISP config files");
DEFINE_string(output_dir,       "",     "output directory");
DEFINE_string(output_raw_dir,   "",     "output directory for raw images (will not save if empty)");
DEFINE_string(bin_list,         "",     "comma-separated list of .bin files");
DEFINE_int32(start_frame,       0,      "start frame (per camera)");
DEFINE_int32(frame_count,       0,      "number of frames to unpack (per camera)");

void makeCameraDir(const string outDir, const uint32_t serial) {
  const string dir = outDir + "/" + to_string(serial);
  mkdir(dir.c_str(), 0755);
}

string createFilename(
    const string outDir,
    const uint32_t serial,
    const size_t frameIndex,
    const string extension) {

  static const int kNumDigits = 6;
  return outDir + "/" + std::to_string(serial)
    + "/" + intToStringZeroPad(frameIndex, kNumDigits) + extension;
}

int main(int argc, char *argv[]) {
  initSurround360(argc, argv);
  requireArg(FLAGS_isp_dir, "isp_dir");
  requireArg(FLAGS_output_dir, "output_dir");
  requireArg(FLAGS_bin_list, "bin_list");

  static unordered_map<uint32_t, string> ispConfigurations;
  vector<BinaryFootageFile> footageFiles;

  std::istringstream binList(FLAGS_bin_list);
  std::string binFile;
  while (std::getline(binList, binFile, ',')) {
    footageFiles.emplace_back(binFile);
  }

  set<uint32_t> serialNumbers;

  for (auto& footageFile : footageFiles) {
    LOG(INFO) << "Reading " << footageFile.getFilename() << "...";

    footageFile.open();
    using futureType = std::future<void>;
    vector<futureType> taskHandles;
    const int numCameras = footageFile.getNumberOfCameras();
    vector<uint32_t> cameraIndexToSerial(numCameras);

    const int startFrame = FLAGS_start_frame;
    const int endFrame = FLAGS_frame_count == 0
      ? footageFile.getNumberOfFrames() - 1
      : startFrame + FLAGS_frame_count - 1;

    for (int cameraIndex = 0; cameraIndex < numCameras; ++cameraIndex) {
      auto taskHandle = std::async(
        std::launch::async,
        [=, &footageFile, &serialNumbers] {
          string json;
          int percentDonePrev = 0;
          for (int frameIndex = startFrame; frameIndex <= endFrame; ++frameIndex) {
            auto frame = footageFile.getFrame(frameIndex, cameraIndex);
            const auto serial = reinterpret_cast<const uint32_t*>(frame)[1];

            if (frameIndex == startFrame) {
              serialNumbers.insert(serial);
              makeCameraDir(FLAGS_output_dir, serial);

              if (!FLAGS_output_raw_dir.empty()) {
                makeCameraDir(FLAGS_output_raw_dir, serial);
              }

              const string fname(FLAGS_isp_dir + "/" + to_string(serial) + ".json");
              ifstream ifs(fname, std::ios::in);
              json = string(
                (std::istreambuf_iterator<char>(ifs)),
                (std::istreambuf_iterator<char>()));
            }

            const auto width = footageFile.getMetadata().width;
            const auto height = footageFile.getMetadata().height;

            auto upscaled = Raw12Converter::convertFrame(frame, width, height);

            if (!FLAGS_output_raw_dir.empty()) {
              const string filenameRaw =
                createFilename(FLAGS_output_raw_dir, serial, frameIndex, ".tiff");
              Mat rawImage(height, width, CV_16UC1, upscaled->data());
              imwriteExceptionOnFail(filenameRaw, rawImage, util::tiffParams);
            }

            const int imageSize = width * height * 3 * sizeof(uint16_t);
            auto coloredImage = make_unique<vector<uint8_t>>(imageSize);

            static const bool kFast = false;
            static const int kOutputBpp = 16;
            CameraIspPipe isp(json, kFast, kOutputBpp);
            isp.setBitsPerPixel(footageFile.getBitsPerPixel());
            isp.enableToneMap();
            isp.loadImage(reinterpret_cast<uint8_t*>(upscaled->data()), width, height);
            isp.setup();
            isp.initPipe();
            isp.getImage(reinterpret_cast<uint8_t*>(coloredImage->data()));

            Mat outputImage(height, width, CV_16UC3, coloredImage->data());
            const string filename =
              createFilename(FLAGS_output_dir, serial, frameIndex, ".png");
            imwriteExceptionOnFail(filename, outputImage);

            if (cameraIndex == 0) {
              const int percentDoneCurr =
                (frameIndex - startFrame + 1) * 100 / FLAGS_frame_count;
              LOG_IF(INFO, percentDoneCurr != percentDonePrev)
                << "Percent done " << percentDoneCurr << "%";
              percentDonePrev = percentDoneCurr;
            }
          }});
      taskHandles.push_back(move(taskHandle));
    }

    for (auto handleIdx = 0; handleIdx < taskHandles.size(); ++handleIdx) {
      taskHandles[handleIdx].get();
    }
  }

  // Rename output directories from serial number to camN, sorted by serial
  // number
  size_t ordinal = 0;
  for (auto& serial : serialNumbers) {
    ostringstream oldDir, newDir;
    oldDir << FLAGS_output_dir << "/" << serial;
    newDir << FLAGS_output_dir << "/cam" << ordinal;

    const string oldDirname(oldDir.str());
    const string newDirname(newDir.str());

    rename(oldDirname.c_str(), newDirname.c_str());
    ++ordinal;
  }
}
