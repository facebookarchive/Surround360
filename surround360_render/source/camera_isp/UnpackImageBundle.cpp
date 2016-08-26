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
#include <iostream>
#include <string>
#include <vector>

#include "CvUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;

DEFINE_int32(image_width,     2048,   "expected image width");
DEFINE_int32(image_height,    2048,   "expected image height");
DEFINE_string(binary_prefix,  "",     "path to binary image disk up to timestamp_ (i.e. before 0,1)");
DEFINE_int32(disk_count,      2,      "number of consumer threads");
DEFINE_int32(start_frame,     0,      "start frame (per camera)");
DEFINE_int32(frame_count,     0,      "number of frames to unpack (per camera)");
DEFINE_string(dest_path,      "",     "path to folder to unpack images");

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_binary_prefix, "binary_prefix");
  requireArg(FLAGS_dest_path, "dest_path");

  const size_t imageSize = FLAGS_image_width * FLAGS_image_height;
  vector<string> cameraNames;
  const string camera_names_path = FLAGS_binary_prefix + "cameranames.txt";
  ifstream cameraNamesFile(camera_names_path);
  if (!cameraNamesFile) {
    throw VrCamException("file read failed:" + camera_names_path);
  }

  string line;
  while (std::getline(cameraNamesFile, line)) {
    cameraNames.push_back(line);
  }
  cameraNamesFile.close();

  const int cameraCount = cameraNames.size();

  // Preallocate the output image
  Mat outImage(FLAGS_image_height, FLAGS_image_width, CV_8U);
  void* outputPtr = outImage.ptr(0);

  // Create dest directory
  struct stat st = {0};
  string destPath(FLAGS_dest_path);
  if (stat(destPath.c_str(), &st) == -1) {
    mkdir(destPath.c_str(), 0755);
  }

  // Read raw bytes and assemble them into images
  int binaryFile[FLAGS_disk_count];

  // Each bin file can have different number of frames
  int frameCount[FLAGS_disk_count];

  size_t readCount[FLAGS_disk_count];

  // Total number of frames is properly updated later if FLAGS_frame_count is 0
  int totalFrameCount = FLAGS_frame_count * cameraCount;

  LOG(INFO) << "Reading binary files...";

  for (int i = 0; i < FLAGS_disk_count; ++i) {
    string fileName(FLAGS_binary_prefix + to_string(i) + ".bin");
    binaryFile[i] = open(fileName.c_str(), O_RDONLY);
    if (binaryFile[i] < 0) {
      throw VrCamException(
        "error opening binary file. err: " + string(strerror(errno)) +
        " filename: " + fileName);
    }

    if (FLAGS_frame_count == 0) {
      // Update frame count to all the available frames
      const size_t bytesInFile = lseek(binaryFile[i], 0, SEEK_END) + 1;
      frameCount[i] =  (bytesInFile / imageSize);
      lseek(binaryFile[i], 0, SEEK_SET);

      VLOG(1) << "Total frame count binary " << i << ": " << frameCount[i];
    }

    // posix_fadvices speeds things up in Linux, but doesn't work in Darwin (Mac OS X)
#if _XOPEN_SOURCE >= 600 || _POSIX_C_SOURCE >= 200112L
    const size_t fileSizeStart = FLAGS_start_frame * cameraCount * imageSize;
    const int numFrames = FLAGS_frame_count == 0 ? frameCount[i] : (FLAGS_frame_count * cameraCount);
    const int fileSize = numFrames * imageSize;
    posix_fadvise(binaryFile[i], fileSizeStart, fileSize, POSIX_FADV_DONTNEED);
    posix_fadvise(binaryFile[i], fileSizeStart, fileSize, POSIX_FADV_SEQUENTIAL);
#endif

    // Move pointer to start frame
    const int numFramesSkip = FLAGS_start_frame * ((cameraCount / FLAGS_disk_count) + ((i + 1) % 2));
    lseek(binaryFile[i], numFramesSkip * imageSize, SEEK_SET);

    if (FLAGS_frame_count == 0) {
      totalFrameCount += frameCount[i];
    }

    readCount[i] = -1;
  }

  const int lastFrame = FLAGS_start_frame * cameraCount + totalFrameCount - 1;

  bool isDone = false;

  int percentDonePrev = 0;
  for (int frameNumber = FLAGS_start_frame; frameNumber < FLAGS_start_frame + totalFrameCount / cameraCount; ++frameNumber) {
    for (unsigned int cameraNumber = 0; cameraNumber < cameraCount; ++cameraNumber) {
      const int frameIndex = frameNumber * cameraCount + cameraNumber;
      const int did = cameraNumber % FLAGS_disk_count;
      readCount[did] = read(binaryFile[did], outputPtr, imageSize);

      // Check if we reached EOF (read returns 0)
      if (readCount[did] == 0) {
        // Check if all the files have reached EOF
        if (!std::all_of(readCount, readCount + FLAGS_disk_count, [](int x){ return x == 0; })) {
          continue;
        }

        isDone = true;
        LOG(WARNING) << "Reached EOF";
        break;
      }

      if (frameIndex % 10 == 0 || frameIndex == lastFrame) {
        int percentDoneCurr = frameIndex * 100 / lastFrame;
        LOG_IF(INFO, percentDoneCurr != percentDonePrev) << "Percent done " << percentDoneCurr << "%";
        percentDonePrev = percentDoneCurr;
      }

      string outFilename =
        FLAGS_dest_path + "/img_" +
        to_string(frameNumber) + "_cam_" +
        cameraNames[cameraNumber] + "_raw8.bmp";
      imwriteExceptionOnFail(outFilename, outImage);
    }

    if (isDone) {
      break;
    }
  }

  LOG(INFO) << "Closing binary files...";

  for (int i = 0; i < FLAGS_disk_count; ++i) {
    close(binaryFile[i]);
  }

  return EXIT_SUCCESS;
}
