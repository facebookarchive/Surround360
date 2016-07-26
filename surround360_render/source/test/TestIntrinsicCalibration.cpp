/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "CvUtil.h"
#include "IntrinsicCalibration.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::calibration;
using namespace surround360::util;

DEFINE_string(mode,                               "",      "e.g., build_intrinsic_model, or undistort");
DEFINE_bool(show_undistorted,                     false,  "show undistorted images in the GUI.");
DEFINE_int32(resize_width,                        -1,     "resize images to this width");
DEFINE_int32(resize_height,                       -1,     "resize images to this height");
DEFINE_int32(checkerboard_width,                  8,      "num of checker corners horizontal");
DEFINE_int32(checkerboard_height,                 5,      "num of checker corners height");
DEFINE_double(checker_size,                       1.0,    "size of a checker in real units (i.e. mm)");
DEFINE_double(sensor_width,                       1.0,    "width of image sensor (same units as checker)");
DEFINE_double(sensor_height,                      1.0,    "height of image sensor (same units as checker)");
DEFINE_string(src_checkerboards_dir,              "",     "directory containing images of checkerboards for intrinsic calibration");
DEFINE_string(dest_param_file,                    "",     "path to write intrinsic matrices");
DEFINE_string(src_intrinsic_param_file,           "",     "path to read intrinsic matrices");
DEFINE_string(src_fisheye_img_dir,                "",     "path to dir of images to undistort");
DEFINE_string(dest_undistorted_dir,               "",     "path to write undistorted images");
DEFINE_string(src_images_dir,                     "",     "path to dir containing checkerboard images");
DEFINE_string(out_pointlist_file,                 "",     "path to write point list file");
DEFINE_string(rig_json_file,                      "",     "path to rig json file");
DEFINE_string(camera_id,                          "",     "id of camera in rig json file");
DEFINE_string(vis_output_dir,                     "",     "path to write output visualizations");
DEFINE_string(fisheye_optical_center_src_image,   "",     "image with diffuser over the lens for estimating optical center");
DEFINE_int32(fisheye_optical_center_threshold,    128,    "threshold applied to fisheye image for optical center estimation");

// reads all of the images in --src_images_dir, and searches for checkboards in each image
// outputs a text file with each line specifying cameraId (inferred from image filename),
// and a checker corner coordinate
void findCheckerCorners() {
  requireArg(FLAGS_src_images_dir, "src_images_dir");
  requireArg(FLAGS_out_pointlist_file, "out_pointlist_file");

  LOG(INFO) << "src_images_dir=" << FLAGS_src_images_dir;
  LOG(INFO) << "checkerboard_width=" << FLAGS_checkerboard_width;
  LOG(INFO) << "checkerboard_height=" << FLAGS_checkerboard_height;

  Size boardSize = Size(FLAGS_checkerboard_width, FLAGS_checkerboard_height);

  vector<string> srcImages = getFilesInDir(FLAGS_src_images_dir, false);
  ofstream outfile;
  outfile.open(FLAGS_out_pointlist_file);
  for (const string& filename : srcImages) {
    const string srcPath = FLAGS_src_images_dir + "/" + filename;

    vector<string> filenameParts = stringSplit(filename, '.');
    assert(filenameParts.size() == 2);
    const string cameraId = filenameParts[0];

    Mat srcImage = imreadExceptionOnFail(srcPath.c_str(), CV_LOAD_IMAGE_COLOR);

    vector<Point2f> cornerPoints;
    bool foundCheckerboard = findChessboardCorners(
      srcImage,
      boardSize,
      cornerPoints,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    LOG(INFO)
      << "filename=" << filename
      << " cameraId=" << cameraId
      << " foundCheckerboard=" << foundCheckerboard
      << " cornerPoints=" << cornerPoints.size();

    // this is a placeholder for when we have multiple images/frames
    const int IMG_INDEX = 0;
    for (const Point2d& pt : cornerPoints) {
      outfile << cameraId << "\t" << IMG_INDEX << "\t"
        << pt.x << "\t" << pt.y << endl;
    }

    static const int kVisualizationCircleRadius = 3;
    static const Scalar kVisualizationCircleColor = Scalar(0, 0, 255);
    static const int kVisualizationCircleThickness = 2;
    static const cv::Size kPreviewSize = Size(1024, 1024);
    for (const Point2d& pt : cornerPoints) {
      circle(
        srcImage,
        pt,
        kVisualizationCircleRadius,
        kVisualizationCircleColor,
        kVisualizationCircleThickness,
        CV_AA);
    }
    Mat smallImage;
    resize(srcImage, smallImage, kPreviewSize);
    imshow("preview", smallImage);
    waitKey(0);
  }
  outfile.close();
}

// reads all of the images in --src_checkerboards_dir, detects checkerboards in each
// image, then uses the checker corners to build an intrinsic calibration model for the
// camera. the intrinsic parameters are saved to --dest_param_file
void buildIntrinsicModel() {
  requireArg(FLAGS_src_checkerboards_dir, "src_checkerboards_dir");
  requireArg(FLAGS_dest_param_file, "dest_param_file");
  requireArgGeqZero(FLAGS_resize_width, "resize_width");
  requireArgGeqZero(FLAGS_resize_height, "resize_height");

  // get the list of images in the source colder
  vector<string> srcImages = getFilesInDir(FLAGS_src_checkerboards_dir, true);
  if (srcImages.size() == 0) {
    throw VrCamException(
      "could not find src_checkerboards_dir: " + FLAGS_src_checkerboards_dir);
  }

  LOG(INFO) << "intrinsic calibration file list:";
  for (const string& filename : srcImages) {
    LOG(INFO) << filename;
  }

  Mat intrinsic, distCoeffs;
  intrinsicCheckerboardCalibration(
    FLAGS_checker_size,
    FLAGS_sensor_width,
    FLAGS_sensor_height,
    FLAGS_checkerboard_width,
    FLAGS_checkerboard_height,
    FLAGS_resize_width,
    FLAGS_resize_height,
    srcImages,
    FLAGS_show_undistorted,
    intrinsic,
    distCoeffs);

  // print camera params
  LOG(INFO) << "intrinsic parameters and distortion coefficients:";
  LOG(INFO) << intrinsic;
  LOG(INFO) << distCoeffs;

  // write camera params to files
  cv::FileStorage fileStorage(FLAGS_dest_param_file.c_str(), cv::FileStorage::WRITE);
  fileStorage << "intrinsic" << intrinsic;
  fileStorage << "distCoeffs" << distCoeffs;
}

// reads all of the images in --src_fisheye_img_dir, and applies an intrinsic camera model
// to correct barrel distortion. writes undistorted (rectilinear) images to
//--dest_undistorted_dir. this is used for debugging / visualizing intrinsic calibration
// results. in the output rectilinear images, straight lines should be straight
void undistortImagesInFolder() {
  requireArg(FLAGS_src_intrinsic_param_file, "src_intrinsic_param_file");
  requireArg(FLAGS_src_fisheye_img_dir, "src_fisheye_img_dir");
  requireArg(FLAGS_dest_undistorted_dir, "dest_undistorted_dir");
  requireArgGeqZero(FLAGS_resize_width, "resize_width");
  requireArgGeqZero(FLAGS_resize_height, "resize_height");

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

  // get the list of images in the source folder
  vector<string> srcImages = getFilesInDir(FLAGS_src_fisheye_img_dir, false);
  if (srcImages.size() == 0) {
    throw VrCamException(
      "could not find src_fisheye_img_dir: " + FLAGS_src_fisheye_img_dir);
  }

  for (const string& filename : srcImages) {
    const string srcPath = FLAGS_src_fisheye_img_dir + "/" + filename;
    const string destPath = FLAGS_dest_undistorted_dir + "/" + filename;
    LOG(INFO) << "srcPath=" << srcPath << " destPath=" << destPath;
    undistortResizeConvert(
      FLAGS_resize_width,
      FLAGS_resize_height,
      intrinsic,
      distCoeffs,
      srcPath,
      destPath);
  }
}

// reads --fisheye_optical_center_src_image, which should be an image captured through a
// fisheye lens with a diffuser and a lightsource. the image is thresholded to estimate
// the fisheye lens optical center.
void estimateFisheyeOpticalCenter() {
  requireArg(FLAGS_fisheye_optical_center_src_image, "fisheye_optical_center_src_image");
  requireArg(FLAGS_vis_output_dir, "vis_output_dir");

  const Mat image = imreadExceptionOnFail(FLAGS_fisheye_optical_center_src_image, 1);
  const Mat mask = estimateOpticalCenterFromDiffusedImage(
    image, FLAGS_fisheye_optical_center_threshold);
  imwriteExceptionOnFail(FLAGS_vis_output_dir + "/mask.png", mask);
}

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_mode, "mode");

  if (FLAGS_mode == "build_intrinsic_model") {
    buildIntrinsicModel();
  } else if (FLAGS_mode == "undistort") {
    undistortImagesInFolder();
  } else if (FLAGS_mode == "find_checker_corners") {
    findCheckerCorners();
  } else if (FLAGS_mode == "fisheye_optical_center") {
    estimateFisheyeOpticalCenter();
  } else {
    throw VrCamException("unrecognized mode: " + FLAGS_mode);
  }
  return EXIT_SUCCESS;
}
