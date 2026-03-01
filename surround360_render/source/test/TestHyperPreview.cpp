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

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "CvUtil.h"
#include "ImageWarper.h"
#include "MathUtil.h"
#include "RigDescription.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace cv;
using namespace std;
using namespace surround360;
using namespace surround360::math_util;
using namespace surround360::util;
using namespace surround360::warper;

DEFINE_int32(image_width,           2048,       "expected image width");
DEFINE_int32(image_height,          2048,       "expected image height");
DEFINE_string(binary_prefix,        "",         "path to binary image disk up to timestamp_ (i.e. before 0,1)");
DEFINE_int32(file_count,            2,          "number of consumer threads");
DEFINE_int32(start_frame,           0,          "start frame (per camera)");
DEFINE_int32(frame_count,           0,          "number of frames to unpack (per camera, 0 = all)");
DEFINE_string(rig_json_file,        "",         "path to json file drescribing camera array");
DEFINE_int32(eqr_width,             2048,       "height of spherical projection image (0 to 2pi)");
DEFINE_int32(eqr_height,            1024,        "height of spherical projection image (0 to pi)");
DEFINE_string(preview_dest,         "",         "path to write equirect preview frames");
DEFINE_double(gamma,                1.0,        "gamma correction exponent");
DEFINE_int32(top_cam_index,         0,          "index of the top camera");
DEFINE_int32(bottom_cam_index,      15,         "index of the primary bottom camera");
DEFINE_int32(bottom_cam2_index,     16,         "index of the secondary bottom camera");
DEFINE_int32(enable_pole_removal,   true,       "if true, the secondary bottom camera is used to remove the pole in the primary bottom camera image");
DEFINE_double(softmax_coef,         30.0,       "steepness of softmax");

// First bytes on each .bin file contain the following metadata:
// 1) magic number
// 2) timestamp
// 3) bin file count
// 4) total number of bin files
// 5) image width
// 6) image height
// 7) bits per pixel
// 8) number of unique cameras in bin file
static const int kHeaderMagicNum      = 0;
static const int kHeaderTimestamp     = 1;
static const int kHeaderBinFileIdx    = 2;
static const int kHeaderBinFileCount  = 3;
static const int kHeaderWidth         = 4;
static const int kHeaderHeight        = 5;
static const int kHeaderNbits         = 6;
static const int kHeaderNumCams       = 7;

static const float kNorm16to8 = 255.0f / 65535.0f;

struct PreviewRenderer {
  vector<Mat> raw;
  vector<Camera> cameras;
  vector<Mat> warped;
  cv::Size outputSize;

  PreviewRenderer() {
    RigDescription rig(FLAGS_rig_json_file);
    cameras.push_back(rig.findCameraByDirection(Camera::Vector3::UnitZ()));
    cameras.push_back(rig.findCameraByDirection(-Camera::Vector3::UnitZ()));
    cameras.push_back(rig.findLargestDistCamAxisToRigCenter());

    cameras[0] = Camera::createRescaledCamera(cameras[0], 0.5);
    cameras[1] = Camera::createRescaledCamera(cameras[1], 0.5);
    cameras[2] = Camera::createRescaledCamera(cameras[2], 0.5);

    warped.push_back(precomputeProjectionWarp(cameras[0]));
    warped.push_back(precomputeProjectionWarp(cameras[1]));
    warped.push_back(precomputeProjectionWarp(cameras[2]));
  }

  void initRaws(const Size& size, const int type) {
    Mat m = Mat::zeros(size, type);
    for (int i = 0; i < 3; ++i) {
      raw.push_back(m.clone());
    }
  }

  void addTopImage(const Mat& image) {
    raw[0] = image.clone();
  }

  void addBottomImage(const Mat& image) {
    raw[1] = image.clone();
  }

  void addBottomImage2(const Mat& image) {
    raw[2] = image.clone();
  }

  Mat precomputeProjectionWarp(const Camera& cam) {
    Mat eqrWarpMat = cv::Mat::zeros(cv::Size(FLAGS_eqr_width, FLAGS_eqr_height), CV_32FC2);
    for (int y = 0; y < FLAGS_eqr_height; ++y) {
      for (int x = 0; x < FLAGS_eqr_width; ++x) {
        const float theta = 2.0f * M_PI * (1.0 - (x + 0.5f) / float(FLAGS_eqr_width));
        const float phi = M_PI * (y + 0.5f) / float(FLAGS_eqr_height);
        const Camera::Vector2 camImagePoint = projectEquirectToCam(
          theta, phi, cam, Camera::kNearInfinity);
        eqrWarpMat.at<Point2f>(y, x) = Point2f(camImagePoint.x(), camImagePoint.y());
      }
    }
    return eqrWarpMat;
  }

  Mat projectToEquirect(const Camera& cam, Mat& srcImg, const Mat& warpMat) {
    Mat projectedCamImg;
    remap(
      srcImg,
      projectedCamImg,
      warpMat,
      Mat(),
      CV_INTER_CUBIC,
      BORDER_CONSTANT);
    return projectedCamImg;
  }

  void render(int frameNumber) {
    vector<Mat> rgb;
    vector<Mat> layers;
    for (int i = 0; i < raw.size(); ++i) {
      rgb.push_back(simpleDemosaic(raw[i]));
      cvtColor(rgb[i], rgb[i], CV_BGR2BGRA);
      if (i > 0) {
        topDownAlphaFade(rgb[i]);
      }
      radialAlphaFade(rgb[i]);
      layers.push_back(projectToEquirect(cameras[i], rgb[i], warped[i]));
    }

    Mat eqrImage = flattenLayersAlphaSoftmax(layers, FLAGS_softmax_coef);
    stringstream ss;
    ss << std::setw(6) << std::setfill('0') << frameNumber;
    const string outFilename = FLAGS_preview_dest + "/" + ss.str() + ".jpg";
    imwriteExceptionOnFail(outFilename, eqrImage * kNorm16to8);
  }

  static Mat simpleDemosaic(const Mat& src) {
    static const int scale = 2;
    static const float kMaxValue16 = 2^16-1;
    static const float kMaxValue32 = 2^32-1;
    Mat dest(src.size() / scale, CV_32FC3);
    for (int y = 0; y < dest.rows; ++y) {
      for (int x = 0; x < dest.cols; ++x) {
        float g1 = src.at<uint16_t>(y * scale, x * scale) / kMaxValue16;
        float g2 = src.at<uint16_t>(y * scale + 1, x * scale + 1) / kMaxValue16;
        float b = src.at<uint16_t>(y * scale, x * scale + 1) / kMaxValue16;
        float r = src.at<uint16_t>(y * scale + 1, x * scale) / kMaxValue16;
        float g = (g1 + g2) / 2.0f;
        r = powf(r, FLAGS_gamma);
        g = powf(g, FLAGS_gamma);
        b = powf(b, FLAGS_gamma);
        dest.at<Vec3f>(y, x) =
          Vec3f(b * kMaxValue32, g * kMaxValue32, r * kMaxValue32);
      }
    }

    return dest;
  }
};

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_binary_prefix, "binary_prefix");
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_preview_dest, "preview_dest");

  size_t imageSize;
  uint32_t cameraCount;
  uint32_t imageWidth;
  uint32_t imageHeight;
  uint32_t nBits;

  int fd[FLAGS_file_count];
  vector<string> binFilenames;
  for (int i = 0; i < FLAGS_file_count; ++i) {
    string fileName(FLAGS_binary_prefix + "/" + to_string(i) + ".bin");
    binFilenames.push_back(fileName);
    fd[i] = open(fileName.c_str(), O_RDONLY);
    if (fd[i] < 0) {
      throw VrCamException(
        "error opening binary file. err: " + string(strerror(errno)) +
        " filename: " + fileName);
    }
  }

  cameraCount = 0;
  uint32_t timestamps[FLAGS_file_count];
  for (uint32_t i = 0; i < FLAGS_file_count; ++i) {
    static const int kNbitsMetadata = 4096 * CHAR_BIT;
    std::vector<unsigned char> imgbufMetadata(kNbitsMetadata);
    size_t readCount = pread(fd[i], &imgbufMetadata[0], kNbitsMetadata, 0);
    if (readCount <= 0) {
      throw VrCamException("empty binary file: " + binFilenames[i]);
    }

    auto tag = reinterpret_cast<uint32_t*>(&imgbufMetadata[0]);
    const uint32_t magicNumber = tag[kHeaderMagicNum];
    if (magicNumber != 0xfaceb00c) {
      throw VrCamException("binary file not tagged: " + binFilenames[i]);
    }

    const uint32_t binIdx = tag[kHeaderBinFileIdx];
    if (binIdx != i) {
      throw VrCamException("binary file name in metadata != loaded .bin file");
    }

    const uint32_t numBinFiles = tag[kHeaderBinFileCount];
    if (numBinFiles != FLAGS_file_count) {
      throw VrCamException("binary file count in metadata != num .bin files");
    }

    const uint32_t widthI = tag[kHeaderWidth];
    const uint32_t heightI = tag[kHeaderHeight];
    const uint32_t nbitsI = tag[kHeaderNbits];
    const uint32_t numCamsI = tag[kHeaderNumCams];

    if (i == 0) {
      imageWidth = widthI;
      imageHeight = heightI;
      nBits = nbitsI;
    } else if (
        widthI != imageWidth || heightI != imageHeight ||
        nbitsI != nBits || timestamps[i] != timestamps[0]) {
      throw VrCamException("metadata not consistent across binary files!");
    }
    cameraCount += numCamsI;
  }
  imageSize = static_cast<float>(imageWidth * imageHeight * nBits) / CHAR_BIT;

  // Preallocate the output image
  Mat outImage(imageHeight, imageWidth, CV_16U);
  void* outputPtr = outImage.ptr(0);

  // Read raw bytes and assemble them into images
  off_t pos[FLAGS_file_count];
  off_t posInit[FLAGS_file_count];

  // Each bin file can have different number of frames
  int frameCount[FLAGS_file_count];
  size_t readCount[FLAGS_file_count];

  // Total number of frames is properly updated later if FLAGS_frame_count is 0
  int totalFrameCount = FLAGS_frame_count * cameraCount;

  LOG(INFO) << "Reading binary files...";

  for (int i = 0; i < FLAGS_file_count; ++i) {
    if (FLAGS_frame_count == 0) {
      struct stat st;
      fstat(fd[i], &st);
      frameCount[i] =  st.st_size / imageSize;
      VLOG(1) << "Total frame count binary " << i << ": " << frameCount[i];
    }

    // posix_fadvices speeds things up in Linux, but doesn't work in Darwin (Mac OS X)
  #if _XOPEN_SOURCE >= 600 || _POSIX_C_SOURCE >= 200112L
    const size_t fileSizeStart = FLAGS_start_frame * cameraCount * imageSize;
    const int numFrames = FLAGS_frame_count == 0 ? frameCount[i] : (FLAGS_frame_count * cameraCount);
    const int fileSize = numFrames * imageSize;
    posix_fadvise(fd[i], fileSizeStart, fileSize, POSIX_FADV_DONTNEED);
    posix_fadvise(fd[i], fileSizeStart, fileSize, POSIX_FADV_SEQUENTIAL);
  #endif

    // Move pointer to start frame
    static const auto kHeaderSize = 4096u;
    posInit[i] = kHeaderSize + imageSize * FLAGS_start_frame * ((cameraCount / FLAGS_file_count) + ((i < cameraCount % FLAGS_file_count) ? 1 : 0));
    pos[i] = posInit[i];

    if (FLAGS_frame_count == 0) {
      totalFrameCount += frameCount[i];
    }

    readCount[i] = -1;
  }

  const int lastFrame = FLAGS_start_frame * cameraCount + totalFrameCount - 1;
  vector<unsigned char> imgbuf(imageSize);

  // Get camera serial numbers
  vector<string> serialNumbers;
  for (int frameNumber = FLAGS_start_frame; frameNumber < FLAGS_start_frame + totalFrameCount / cameraCount; ++frameNumber) {
    for (unsigned int cameraNumber = 0; cameraNumber < cameraCount; ++cameraNumber) {
      const int idx = cameraNumber % FLAGS_file_count;
      readCount[idx] = pread(fd[idx], &imgbuf[0], imageSize, pos[idx]);
      auto tag = reinterpret_cast<uint32_t*>(&imgbuf[0]);
      std::string serialNumber = to_string(tag[1]);
      if (find(serialNumbers.begin(), serialNumbers.end(), serialNumber) != serialNumbers.end()) {
        break;
      }
      serialNumbers.push_back(serialNumber);
      pos[idx] += readCount[idx];
    }
  }
  vector<string> sortedSerialNumbers(serialNumbers);
  sort(sortedSerialNumbers.begin(), sortedSerialNumbers.end());

  // Move pointers back to start frame
  for (int i = 0; i < FLAGS_file_count; ++i) {
    pos[i] = posInit[i];
  }

  LOG(INFO) << "Generating previews...";
  PreviewRenderer previewRenderer;

  // Initialize raw images
  previewRenderer.initRaws(outImage.size(), outImage.type());

  bool isDone = false;
  int percentDonePrev = 0;
  for (int frameNumber = FLAGS_start_frame; frameNumber < FLAGS_start_frame + totalFrameCount / cameraCount; ++frameNumber) {
    for (unsigned int cameraNumber = 0; cameraNumber < cameraCount; ++cameraNumber) {
      auto tag = reinterpret_cast<uint32_t*>(&imgbuf[0]);
      std::string serialNumber = to_string(tag[1]);

      const int frameIndex = frameNumber * cameraCount + cameraNumber;
      const int idx = cameraNumber % FLAGS_file_count;

      const string sn = serialNumbers[cameraNumber];
      if (sn != sortedSerialNumbers[FLAGS_top_cam_index] &&
          sn != sortedSerialNumbers[FLAGS_bottom_cam_index] &&
          sn != sortedSerialNumbers[FLAGS_bottom_cam2_index]) {
        pos[idx] += imageSize;
        continue;
      }

      readCount[idx] = pread(fd[idx], &imgbuf[0], imageSize, pos[idx]);
      pos[idx] += readCount[idx];

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

      if (frameIndex % 10 == 0 || frameIndex == lastFrame) {
        int percentDoneCurr = frameIndex * 100 / lastFrame;
        LOG_IF(INFO, percentDoneCurr != percentDonePrev) << "Percent done "
          << percentDoneCurr << "%";
        percentDonePrev = percentDoneCurr;
      }

      uint32_t p = 0;
      for (uint32_t y = 0; y < imageHeight; ++y) {
        for (uint32_t x = 0; x < imageWidth; ++x) {
          uint32_t pixval;
          if (nBits == 8) {
            pixval = imgbuf[p];
            outImage.at<uint16_t>(y, x) = pixval * 0x101;
            ++p;
          } else if (nBits == 12) {
            uint16_t lo = imgbuf[p];
            uint16_t hi = imgbuf[p + 1];
            uint16_t unswizzled, rep;
            if (x & 1) {
              p += 2;
              unswizzled = hi << 4 | lo >> 4;
            } else {
              p += 1;
              unswizzled = lo << 4 | (hi & 0xF);
            }
            rep = unswizzled << 4 | unswizzled >> 8;
            outImage.at<uint16_t>(y, x) = rep;
          }
        }
      }

      if (sn == sortedSerialNumbers[FLAGS_top_cam_index]) {
        previewRenderer.addTopImage(outImage);
      } else if (sn == sortedSerialNumbers[FLAGS_bottom_cam_index]) {
        previewRenderer.addBottomImage(outImage);
      } else if (sn == sortedSerialNumbers[FLAGS_bottom_cam2_index]) {
        previewRenderer.addBottomImage2(outImage);
      }
    }

    LOG(INFO) << "Rendering frame " << to_string(frameNumber) << "...";

    previewRenderer.render(frameNumber);

    if (isDone) {
      break;
    }
  }

  LOG(INFO) << "Closing binary files...";
  for (int i = 0; i < FLAGS_file_count; ++i) {
    close(fd[i]);
  }

  return EXIT_SUCCESS;
}
