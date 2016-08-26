/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <x86intrin.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

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
DEFINE_int32(file_count,      2,      "count of input files");
DEFINE_int32(start_frame,     0,      "start frame (per camera)");
DEFINE_int32(frame_count,     0,      "number of frames to unpack (per camera)");
DEFINE_string(dest_path,      "",     "path to folder to unpack images");
DEFINE_int32(nbits,           8,      "bits per pixel: 8, 12, 16");

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_binary_prefix, "binary_prefix");
  requireArg(FLAGS_dest_path, "dest_path");

  const size_t imageSize = FLAGS_image_width * FLAGS_image_height * (FLAGS_nbits / 8.0f);
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
  Mat outImage(FLAGS_image_height, FLAGS_image_width, CV_16U);

  // Create dest directory
  struct stat st = {0};
  string destPath(FLAGS_dest_path);
  if (stat(destPath.c_str(), &st) == -1) {
    mkdir(destPath.c_str(), 0755);
  }

  // Read raw bytes and assemble them into images
  int fd[FLAGS_file_count];
  off_t pos[FLAGS_file_count];
  // Each bin file can have different number of frames
  int imageCount[FLAGS_file_count];
  size_t readCount[FLAGS_file_count];

  // Total number of frames is properly updated later if FLAGS_frame_count is 0
  int totalImageCount = FLAGS_frame_count * cameraCount;

  LOG(INFO) << "Reading binary files...";

  for (int i = 0; i < FLAGS_file_count; ++i) {
    string fileName(FLAGS_binary_prefix + to_string(i) + ".bin");
    fd[i] = open(fileName.c_str(), O_RDONLY);

    if (fd[i] < 0) {
      throw VrCamException(
        "error opening binary file. err: " + string(strerror(errno)) +
        " filename: " + fileName);
    }

    if (FLAGS_frame_count == 0) {
      // Update frame count to all the available frames
      struct stat st;
      fstat(fd[i], &st);
      imageCount[i] = st.st_size / imageSize;
      VLOG(1) << "Total frame count binary " << i << ": " << imageCount[i];
    }

    // posix_fadvices speeds things up in Linux, but doesn't work in Darwin (Mac OS X)
#if _XOPEN_SOURCE >= 600 || _POSIX_C_SOURCE >= 200112L
    const size_t fileSizeStart = FLAGS_start_frame * cameraCount * imageSize;
    const int numFrames = FLAGS_frame_count == 0 ? imageCount[i] : (FLAGS_frame_count * cameraCount);
    const int fileSize = numFrames * imageSize;
    posix_fadvise(fd[i], fileSizeStart, fileSize, POSIX_FADV_DONTNEED);
    posix_fadvise(fd[i], fileSizeStart, fileSize, POSIX_FADV_SEQUENTIAL);
#endif

    // Move pointer to start frame
    pos[i] = imageSize * FLAGS_start_frame
      * ((cameraCount / FLAGS_file_count) + ((i < cameraCount % FLAGS_file_count) ? 1 : 0);

    if (FLAGS_frame_count == 0) {
      totalImageCount += imageCount[i];
    }

    readCount[i] = -1;
  }

  const int lastFrame = FLAGS_start_frame * cameraCount + totalImageCount - 1;

  bool isDone = false;
  auto imgbuf = std::make_unique<unsigned char[]>(imageSize);

  int percentDonePrev = 0;
  for (
       int frameNumber = FLAGS_start_frame;
       frameNumber < FLAGS_start_frame + totalImageCount / cameraCount;
       ++frameNumber) {
    for (uint32_t cameraNumber = 0; cameraNumber < cameraCount; ++cameraNumber) {
      const int frameIndex = frameNumber * cameraCount + cameraNumber;
      const int idx = cameraNumber % FLAGS_file_count;

      readCount[idx] = pread(fd[idx], imgbuf, imageSize, pos[idx]);

      // Check if we reached EOF (read returns 0)
      if (readCount[idx] == 0) {
        // Check if all the files have reached EOF
        if (!std::all_of(readCount, readCount + FLAGS_file_count, [](int x){ return x == 0; })) {
          continue;
        }

        isDone = true;
        LOG(WARNING) << "Reached EOF";
        break;
      }

      uint32_t p = 0;

      /* traverse bits and extract 16-bit values depending on the
         pixel format used */
      for (uint32_t y = 0; y < FLAGS_image_height; ++y) {
        for (uint32_t x = 0; x < FLAGS_image_width; ++x) {
          uint32_t pixval;
          if (FLAGS_nbits == 8) {
            pixval = imgbuf[p];
            // scale the value to 16-bits, 2^16 / 2^8 = 2^8
            outImage.at<uint16_t>(y, x) = pixval << 8 | pixval;
            ++p;
          } else if (FLAGS_nbits == 12) {
            uint16_t pixval = *(uint16_t*)(imgbuf + p);
            uint32_t unswizzled = 0;

            /*
              bit order of pixels in 12-bit is in a proprietary packed format:
                               byte 2                   byte 1                     byte 0
                    +-------------------------+-------------------------+-------------------------+
               bit  + 12 11 10  9  8  7  6  5 |  4  3  2  1  4  3  2  1 | 12 11 10  9  8  7  6  5 |
                    +-------------------------+-------------------------+-------------------------+

               we unpack the bits from byte 2 and 1 by simply reading
               the 16-bit value at offset (addr + 1) and shifting it
               to the right by 4 bits.

               recovering the value from bytes 0 and 1 is done with,
               first swapping the byte-order to obtain (byte 0, byte 1)
               value and issuing parallel bit extract instruction
               with a mask that deposits bits from source operand into
               a contiguous region of the destination operand when a
               corresponding bit of mask is set.
            */
            if (x & 1) {
              unswizzled = pixval >> 4;
              ++p;
            } else {
              unswizzled = _pext_u32(__builtin_bswap16(pixval & 0xfff), 0xff0f);
            }
            ++p;

            // scale the value to 16 bits, 2^16 / 2^12 = 2^4
            outImage.at<uint16_t>(y, x) = unswizzled << 4 | unswizzled >> 8;
          } else if (FLAGS_nbits == 16) {
            pixval = *(uint16_t*)(imgbuf + p);
            outImage.at<uint16_t>(y, x) = pixval;
            p += sizeof(uint16_t);
          }
        }
      }

      pos[idx] += readCount[idx];

      if (frameIndex % 10 == 0 || frameIndex == lastFrame) {
        int percentDoneCurr = frameIndex * 100 / lastFrame;
        LOG_IF(INFO, percentDoneCurr != percentDonePrev) << "Percent done " << percentDoneCurr << "%";
        percentDonePrev = percentDoneCurr;
      }

      // save as TIFF, as we need 16-bits per channel
      string outFilename =
        FLAGS_dest_path + "/img_" +
        to_string(frameNumber) + "_cam_" +
        cameraNames[cameraNumber] + "_raw" + to_string(FLAGS_nbits) + ".tiff";
      imwriteExceptionOnFail(outFilename, outImage);
    }

    if (isDone) {
      break;
    }
  }

  LOG(INFO) << "Closing binary files...";

  for (auto f : fd) {
    close(f);
  }

  return EXIT_SUCCESS;
}
