/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "CvUtil.h"
#include "ImageWarper.h"
#include "StringUtil.h"
#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360::util;
using namespace surround360::warper;

DEFINE_string(test_dir, "", "path to dir with test files");

void testSideFisheyeProjection() {
  const string srcPath = FLAGS_test_dir + "/cam0.png";
  Mat srcImage = imreadExceptionOnFail(srcPath, -1);
  CameraMetadata camModel;
  camModel.isFisheye = true;
  camModel.usablePixelsRadius = 1298;
  camModel.imageCenterX = 1224;
  camModel.imageCenterY = 1024;
  Mat eqrImage = sideFisheyeToSpherical(srcImage, camModel, 2048, 2048);
  imwriteExceptionOnFail(FLAGS_test_dir + "/eqr.png", eqrImage);
}

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_test_dir, "test_dir");

  testSideFisheyeProjection();
  return EXIT_SUCCESS;
}
