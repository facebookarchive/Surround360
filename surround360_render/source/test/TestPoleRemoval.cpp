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
#include "PoleRemoval.h"
#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;

DEFINE_string(src_images_dir,       "",   "path to folder containing camera images");
DEFINE_string(pole_masks_dir,       "",   "path to folder containing pole masks");
DEFINE_string(output_dir,           "",   "path to write results");
DEFINE_string(rig_json_file,        "",   "path to json file drescribing camera array");
DEFINE_string(flow_alg,             "",   "optical flow algorithm to use");
DEFINE_int32(alpha_feather_size,    100,  "feather on images before projection fixes an artifact");

// a standalone test of pole removal for the bottom cameras. reads a camera array
// description from --rig_json_file, camera images from --src_images_dir and pole masks
// from --pole_masks_dir. applies the optical flow algorithm specified by --flow_alg to
// generate an image combining the two bottom camera images but with the masked regions
// removed.
int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_src_images_dir, "src_images_dir");
  requireArg(FLAGS_pole_masks_dir, "pole_masks_dir");
  requireArg(FLAGS_output_dir, "output_dir");
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_flow_alg, "flow_alg");

  vector<CameraMetadata> camModelArrayWithTop =
    readCameraProjectionModelArrayFromJSON(FLAGS_rig_json_file);

  CameraMetadata bottomCamModel;
  Mat bottomImage;
  combineBottomImagesWithPoleRemoval(
    FLAGS_src_images_dir,
    FLAGS_pole_masks_dir,
    "NONE",
    FLAGS_output_dir,
    true, // save debug images
    false, // save extra data for next frame of flow (not needed for test)
    FLAGS_flow_alg,
    FLAGS_alpha_feather_size,
    camModelArrayWithTop,
    bottomCamModel,
    bottomImage);

  return EXIT_SUCCESS;
}
