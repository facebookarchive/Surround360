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
#include <map>
#include <string>
#include <vector>

#include "CameraMetadata.h"
#include "CvUtil.h"
#include "IntrinsicCalibration.h"
#include "KeypointMatchers.h"
#include "MathUtil.h"
#include "RingRectification.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace cv::detail;
using namespace surround360;
using namespace surround360::calibration;
using namespace surround360::util;

DEFINE_string(rig_json_file,              "",   "path to json file drescribing camera array");
DEFINE_string(src_intrinsic_param_file,   "",   "path to read intrinsic matrices");
DEFINE_string(root_dir,                   "",   "path to a video root directory");
DEFINE_string(frames_list,                "",   "list of frame indices to process");
DEFINE_string(visualization_dir,          "",   "path to write visualizations");
DEFINE_string(output_transforms_file,     "",   "path to write transforms");

// runs the optimization procedure for joint stereo rectification of the side cameras,
// and produces one perspective transform matrix per camera, which is meant to be applied
// to its rectilinear projection.
int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_src_intrinsic_param_file, "src_intrinsic_param_file");
  requireArg(FLAGS_root_dir, "root_dir");
  requireArg(FLAGS_frames_list, "frames_list");
  requireArg(FLAGS_visualization_dir, "visualization_dir");
  requireArg(FLAGS_output_transforms_file, "output_transforms_file");

  // create directories for visualization. it's OK if these fail because the
  // folders already exist.
  system(string("mkdir " + FLAGS_visualization_dir).c_str());
  system(string("mkdir " + FLAGS_visualization_dir + "/keypoint_vis").c_str());
  system(string("mkdir " + FLAGS_visualization_dir + "/warped").c_str());

  // read the intrinsic/distortion coefs from file
  cv::FileStorage fileStorage(FLAGS_src_intrinsic_param_file, FileStorage::READ);
  if (!fileStorage.isOpened()) {
    throw VrCamException("file read failed: " + FLAGS_src_intrinsic_param_file);
  }
  Mat intrinsic, distCoeffs;
  fileStorage["intrinsic"] >> intrinsic;
  fileStorage["distCoeffs"] >> distCoeffs;
  LOG(INFO) << "intrinsic parameters and distortion coefficients:";
  LOG(INFO) << intrinsic;
  LOG(INFO) << distCoeffs;

  LOG(INFO) << "reading camera json file";
  float cameraRingRadius;
  vector<CameraMetadata> camModelArrayWithTop =
    readCameraProjectionModelArrayFromJSON(
      FLAGS_rig_json_file,
      cameraRingRadius);

  LOG(INFO) << "removing top and side cameras from rig";
  vector<CameraMetadata> camModelArray =
    removeTopAndBottomFromCamArray(camModelArrayWithTop);
  const int numSideCameras = camModelArray.size();

  vector<string> framesList = stringSplit(FLAGS_frames_list, ',');
  assert(!framesList.empty());

  vector<vector<Mat>> sideCamImages(framesList.size(), vector<Mat>());
  for (int frameIdx = 0; frameIdx < framesList.size(); ++frameIdx) {
    const string& frameName = framesList[frameIdx];
    LOG(INFO) << "getting side camera images from frame: " << frameName;
    for (int i = 0; i < numSideCameras; ++i) {
      const string srcImagePath = FLAGS_root_dir + "/vid/" +
        framesList[frameIdx] + "/isp_out/" + camModelArray[i].cameraId + ".png";
      LOG(INFO) << "\t" << camModelArray[i].cameraId << ": " << srcImagePath;
      Mat origImage = imreadExceptionOnFail(srcImagePath, CV_LOAD_IMAGE_COLOR);
      Mat imageUndistorted;
      cvUndistortBicubic(
        origImage, imageUndistorted, intrinsic, distCoeffs, intrinsic);
      sideCamImages[frameIdx].push_back(imageUndistorted);
    }
  }

  LOG(INFO) << "finding keypoint matches and optimizing rectification";
  const vector<Mat> finalTransforms = optimizeRingRectification(
    camModelArray,
    sideCamImages,
    framesList.size(),
    FLAGS_visualization_dir + "/keypoint_vis");
  FileStorage transformsFile(FLAGS_output_transforms_file, FileStorage::WRITE);
  for (int i = 0; i < numSideCameras; ++i) {
    transformsFile << camModelArray[i].cameraId << finalTransforms[i];
  }

  LOG(INFO) << "generating visualizations";

  // Remap entire image from rectilinear to spherical
  float fovH = camModelArray[0].fovHorizontal;
  float fovV = camModelArray[0].fovHorizontal / camModelArray[0].aspectRatioWH;

  vector<Mat> warpedSideImages;
  for (int i = 0; i < numSideCameras; ++i) {
    LOG(INFO) << "warping image " << i;
    LOG(INFO) << "sideCamImages[0][i].size()=" << sideCamImages[0][i].size();
    Mat warpedImage;
    warpPerspective(
      sideCamImages[0][i], warpedImage, finalTransforms[i], sideCamImages[0][i].size());

    Mat sphericalImage = projectRectilinearToSpherical(
      warpedImage,
      fovH,
      fovV,
      sideCamImages[0][i].cols,
      sideCamImages[0][i].rows);

    warpedSideImages.push_back(sphericalImage);

    imwriteExceptionOnFail(
      FLAGS_visualization_dir + "/warped/" + camModelArray[i].cameraId + ".png",
      sphericalImage);
  }

  // putting them all in one image makes it easier to check rectification
  Mat stackedImage = stackHorizontal(warpedSideImages);
  imwriteExceptionOnFail(
    FLAGS_visualization_dir + "/stacked.png",
    stackedImage);

  return EXIT_SUCCESS;
}
