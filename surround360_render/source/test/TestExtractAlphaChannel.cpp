/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

// this is a simple utility for extracting the alpha channel from a 4-channel image,
// used for debugging.

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "StringUtil.h"
#include "CvUtil.h"
#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360::util;

DEFINE_string(src, "", "path to source 4-channel image");

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_src, "src");

  Mat srcImage = imreadExceptionOnFail(FLAGS_src, -1); // -1 = load RGBA
  vector<Mat> channels;
  split(srcImage, channels);
  imwriteExceptionOnFail(FLAGS_src + "_a.png", channels[3]);

  vector<Mat> colorChannels = {channels[0], channels[1], channels[2]};
  Mat bgr;
  merge(colorChannels, bgr);
  imwriteExceptionOnFail(FLAGS_src + "_rgb.png", bgr);

  return EXIT_SUCCESS;
}
