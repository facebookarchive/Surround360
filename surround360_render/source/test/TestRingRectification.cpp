/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <stdio.h>

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
DEFINE_string(src_undistorted,            "",   "path to undistorted side camera images dir");
DEFINE_string(match_vis_dir,              "",   "path to write keypoint matching visualizations");
DEFINE_string(four_pt_vis_dir,            "",   "path to write 4 point perspective visualization");
DEFINE_string(warped_output_dir,          "",   "path to write result warped images");
DEFINE_string(output_transforms_file,     "",   "path to write transforms");
DEFINE_string(src_undistorted_features,   "",   "path to undistorted side camera images dir for feature extraction");

// runs the optimization procedure for joint stereo rectification of the side cameras,
// and produces one perspective transform matrix per camera, which is meant to be applied
// to its rectilinear projection.
int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_src_undistorted, "src_undistorted");
  requireArg(FLAGS_match_vis_dir, "match_vis_dir");
  requireArg(FLAGS_four_pt_vis_dir, "four_pt_vis_dir");
  requireArg(FLAGS_warped_output_dir, "warped_output_dir");
  requireArg(FLAGS_output_transforms_file, "output_transforms_file");
  requireArg(FLAGS_src_undistorted_features, "src_undistorted_features");

  LOG(INFO) << "reading camera json file";
  vector<CameraMetadata> camModelArrayWithTop =
    readCameraProjectionModelArrayFromJSON(FLAGS_rig_json_file);

  LOG(INFO) << "removing top and side cameras from rig";
  vector<CameraMetadata> camModelArray =
    removeTopAndBottomFromCamArray(camModelArrayWithTop);
  const int numSideCameras = camModelArray.size();

  LOG(INFO) << "loading source images";
  vector<Mat> sideCamImages;
  for (int i = 0; i < numSideCameras; ++i) {
    LOG(INFO) << camModelArray[i].cameraId;
    sideCamImages.push_back(
      imreadExceptionOnFail(FLAGS_src_undistorted + "/" + camModelArray[i].cameraId + ".png",
      CV_LOAD_IMAGE_COLOR));
  }

  LOG(INFO) << "loading source images for feature extraction";
  vector<string> featureDirs = getFilesInDir(FLAGS_src_undistorted_features, false);
  vector<vector<Mat>> sideCamImagesFeatures(featureDirs.size(), vector<Mat>());
  for (size_t iDir = 0; iDir < featureDirs.size(); iDir++) {
    LOG(INFO) << "reading images from dir: " << featureDirs[iDir];
    for (size_t i = 0; i < numSideCameras; ++i) {
      LOG(INFO) << featureDirs[iDir] << ": " << camModelArray[i].cameraId;
      sideCamImagesFeatures[iDir].push_back(
        imreadExceptionOnFail(
          FLAGS_src_undistorted_features + "/" + featureDirs[iDir] + "/" + camModelArray[i].cameraId + ".png",
          CV_LOAD_IMAGE_COLOR));
    }
  }

  LOG(INFO) << "saving transforms";
  const vector<Mat> finalTransforms = optimizeRingRectification(
    camModelArray,
    sideCamImagesFeatures,
    featureDirs.size(),
    FLAGS_match_vis_dir);
  FileStorage transformsFile(FLAGS_output_transforms_file, FileStorage::WRITE);
  for (int i = 0; i < numSideCameras; ++i) {
    transformsFile << camModelArray[i].cameraId << finalTransforms[i];
  }

  LOG(INFO) << "saving warped images";
  vector<Mat> warpedSideImages;

  // Remap entire image from rectilinear to spherical
  float fovH = camModelArray[0].fovHorizontal;
  float fovV = camModelArray[0].fovHorizontal / camModelArray[0].aspectRatioWH;

  for (int i = 0; i < numSideCameras; ++i) {
    LOG(INFO) << "warping image " << i;
    LOG(INFO) << "sideCamImages[i].size()=" << sideCamImages[i].size();
    Mat warpedImage;
    warpPerspective(
      sideCamImages[i], warpedImage, finalTransforms[i], sideCamImages[i].size());

    Mat sphericalImage = projectRectilinearToSpherical(
      warpedImage,
      fovH,
      fovV,
      sideCamImages[i].cols,
      sideCamImages[i].rows);

    warpedSideImages.push_back(sphericalImage);

    imwriteExceptionOnFail(
      FLAGS_warped_output_dir + "/" + camModelArray[i].cameraId + "_sphe.png",
      sphericalImage);
  }

  // putting them all in one image makes it easier to check rectification
  Mat stackedImage = stackHorizontal(warpedSideImages);
  imwriteExceptionOnFail(
    FLAGS_warped_output_dir + "/stacked.png",
    stackedImage);

  return EXIT_SUCCESS;
}
