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

#include "BinaryFootageFile.h"
#include "CameraIspPipe.h"
#include "RawConverter.h"
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
  ostringstream dirStream;
  dirStream << outDir << "/" << serial;
  mkdir(dirStream.str().c_str(), 0755);
}

string createFilename(
    const string outDir,
    const uint32_t serial,
    const size_t frameIndex,
    const string extension) {

  static const int kNumDigits = 6;
  ostringstream filenameStream;
  filenameStream << outDir << "/" << serial << "/"
                 << setfill('0') << setw(kNumDigits) << frameIndex
                 << extension;
  return filenameStream.str();
}

int main(int argc, char *argv[]) {
  initSurround360(argc, argv);
  requireArg(FLAGS_isp_dir, "isp_dir");
  requireArg(FLAGS_output_dir, "output_dir");
  requireArg(FLAGS_bin_list, "bin_list");

  static unordered_map<uint32_t, string> ispConfigurations;
  vector<BinaryFootageFile> footageFiles;

  istringstream binList(FLAGS_bin_list);
  string binFile;
  while (getline(binList, binFile, ',')) {
    footageFiles.emplace_back(binFile);
  }

  set<uint32_t> serialNumbers[footageFiles.size()];

  for (int fileIndex = 0; fileIndex < footageFiles.size(); ++fileIndex) {
    auto& footageFile = footageFiles[fileIndex];

    LOG(INFO) << "Reading " << footageFile.getFilename() << "...";

    footageFile.open();
    using futureType = future<void>;
    vector<futureType> taskHandles;
    const int numCameras = footageFile.getNumberOfCameras();

    if (numCameras == 0) {
      LOG(INFO) << "No cameras found...";
      continue;
    }

    vector<uint32_t> cameraIndexToSerial(numCameras);

    const int endFrameMax = footageFile.getNumberOfFrames() - 1;
    const int startFrame = FLAGS_start_frame;
    int endFrame = FLAGS_frame_count == 0
      ? endFrameMax
      : startFrame + FLAGS_frame_count - 1;
    const int frameCount = endFrame - startFrame + 1;

    if (startFrame > endFrameMax) {
      stringstream ss;
      ss << "Start frame (" << startFrame
         << ") larger than total number of frames (" << endFrameMax << ")";
      throw VrCamException(ss.str());
    }

    if (endFrame > endFrameMax) {
      LOG(WARNING)
        << "End frame (" << endFrame
        << ") larger than total number of frames (" << endFrameMax << ")";
      endFrame = endFrameMax;
    }

    for (int cameraIndex = 0; cameraIndex < numCameras; ++cameraIndex) {
      auto taskHandle = async(
        launch::async,
        [=, &footageFile, &serialNumbers] {
          string json;
          int percentDonePrev = 0;
          for (int frameIndex = startFrame; frameIndex <= endFrame; ++frameIndex) {
            auto frame = footageFile.getFrame(frameIndex, cameraIndex);
            const auto serial = reinterpret_cast<const uint32_t*>(frame)[1];

            LOG(INFO) << "Looking for serial number " << serial;

            if (serialNumbers[fileIndex].count(serial) == 0) {
              serialNumbers[fileIndex].insert(serial);
              makeCameraDir(FLAGS_output_dir, serial);

              if (FLAGS_output_raw_dir != "") {
                makeCameraDir(FLAGS_output_raw_dir, serial);
              }
            }

            const auto width = footageFile.getMetadata().width;
            const auto height = footageFile.getMetadata().height;

            auto upscaled = footageFile.getBitsPerPixel() == 8
              ? RawConverter::convert8Frame(frame, width, height)
              : RawConverter::convert12Frame(frame, width, height);

            if (FLAGS_output_raw_dir != "") {
              const string filenameRaw = createFilename(
                FLAGS_output_raw_dir, serial, frameIndex, ".tiff");
              Mat rawImage(height, width, CV_16UC1, upscaled->data());
              imwriteExceptionOnFail(filenameRaw, rawImage, util::tiffParams);
            }

            if (cameraIndex == 0) {
              const int percentDoneCurr =
                (frameIndex - startFrame + 1) * 100 / frameCount;
              LOG_IF(INFO, percentDoneCurr != percentDonePrev)
                << "Percent done " << percentDoneCurr << "%";
              percentDonePrev = percentDoneCurr;
            }

            const string fname(
              FLAGS_isp_dir + "/" + to_string(serial) + ".json");
            ifstream ispJsonStream(fname, ios::in);

            // If ISP file not found, do not throw and exit, we can still unpack
            // raws
            if (!ispJsonStream) {
              LOG(INFO) << "Cannot convert to RGB, file not found: " << fname;
            } else {
              json = string(
                (istreambuf_iterator<char>(ispJsonStream)),
                (istreambuf_iterator<char>()));

              const int imageSize = width * height * 3 * sizeof(uint16_t);
              auto coloredImage = make_unique<vector<uint8_t>>(imageSize);

              static const bool kFast = false;
              static const int kOutputBpp = 16;
              CameraIspPipe isp(json, kFast, kOutputBpp);
              isp.setBitsPerPixel(footageFile.getBitsPerPixel());
              isp.enableToneMap();
              isp.loadImage(
                  reinterpret_cast<uint8_t*>(upscaled->data()), width, height);
              isp.setup();
              isp.initPipe();
              isp.getImage(reinterpret_cast<uint8_t*>(coloredImage->data()));

              Mat outputImage(height, width, CV_16UC3, coloredImage->data());
              const string filename =
                createFilename(FLAGS_output_dir, serial, frameIndex, ".png");
              imwriteExceptionOnFail(filename, outputImage);
            }
          }});
      taskHandles.push_back(move(taskHandle));
    }

    for (auto handleIdx = 0; handleIdx < taskHandles.size(); ++handleIdx) {
      taskHandles[handleIdx].get();
    }
  }

  // Merge serialNumbers[1+] into serialNumbers[0]
  for (int fileIndex = 1; fileIndex < footageFiles.size(); ++fileIndex) {
    serialNumbers[0].insert(
      serialNumbers[fileIndex].begin(),
      serialNumbers[fileIndex].end());
  }

  // Rename output directories from serial number to camN, sorted by serial
  // number
  size_t ordinal = 0;
  for (auto& serial : serialNumbers[0]) {
    ostringstream oldDir, newDir;
    oldDir << FLAGS_output_dir << "/" << serial;
    newDir << FLAGS_output_dir << "/cam" << ordinal;

    const string oldDirname(oldDir.str());
    const string newDirname(newDir.str());

    rename(oldDirname.c_str(), newDirname.c_str());
    ++ordinal;
  }
}
