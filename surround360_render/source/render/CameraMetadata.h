/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <vector>

#include "CvUtil.h"

namespace surround360 {
namespace calibration {

using namespace cv;
using namespace std;
using namespace util;

// describes properties of a camera such as whether it is pointing up/down, whether it is
// a fisheye lens, whether it is flipped or rotated around its primary axis, its FOV, etc.
struct CameraMetadata {
  string cameraId;
  float fovHorizontal; // degrees
  float aspectRatioWH; // ratio of width to height of image plane (non-fisheye)

  // fisheye only params (note fisheye doesn't have all of the above either)
  bool isFisheye, isTop, isBottom, isBottom2;
  float fisheyeFovDegrees;
  float fisheyeFovDegreesCrop; // we will crop down to only this much
  float imageCenterX, imageCenterY;
  float usablePixelsRadius;
  float fisheyeRotationDegrees; // rotation around up
  bool flip180; // currently only supported for the secondary bottom camera

  CameraMetadata() :
    isFisheye(false),
    isTop(false),
    isBottom(false),
    isBottom2(false),
    flip180(false) {}
};

// reads a JSON file that stores metadata for an array of cameras
vector<CameraMetadata> readCameraProjectionModelArrayFromJSON(
  const string& jsonFilePath);

// takes a camera model array (which includes "camera_id" names for each camera)
// and a folder containing images. verifies the image prefixes match the camera
// names.
void verifyImageDirFilenamesMatchCameraArray(
  const vector<CameraMetadata>& cameraArray,
  const string& imageDir);

// wrapper for imreadExceptionOnFail( for use in std::threads
static void imreadInStdThread(string path, int flags, Mat* dest) {
  *dest = imreadExceptionOnFail(path, flags);
}

// takes a camera projection model array and a path to a folder containing
// images. loads the images and matches them up with cameras by filename.
// returns an array of pairs of camera model and image. this function assumes
// valid data and does not check for errors. it is recommend to run
// verifyImageDirFilenamesMatchCameraArray before this. this is multi-threaded.
void loadCameraImagePairs(
  const vector<CameraMetadata>& cameraArray,
  const string& imageDir,
  vector< pair<CameraMetadata, Mat> >& camImagePairs);

// return an array of all cameras without the top/bottom cameras
vector<CameraMetadata> removeTopAndBottomFromCamArray(
  const vector<CameraMetadata>& camModelArray);

// get the top camera
CameraMetadata getTopCamModel(const vector<CameraMetadata>& camModelArray);

// get the primary bottom camera
CameraMetadata getBottomCamModel(const vector<CameraMetadata>& camModelArray);

// get the secondary bottom camera
CameraMetadata getBottomCamModel2(const vector<CameraMetadata>& camModelArray);

// returns the meta data for a camera specified by id from within the array, or throws an
// exception if it is not there.
CameraMetadata getCameraById(
  const vector<CameraMetadata>& camModelArray,
  const string& id);

} // namespace calibration
} // namespace surround360
