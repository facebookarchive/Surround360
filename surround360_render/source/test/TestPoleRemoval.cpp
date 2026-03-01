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

#include "Camera.h"
#include "CvUtil.h"
#include "PoleRemoval.h"
#include "RigDescription.h"
#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;

DEFINE_string(src_images_dir,           "",             "path to folder containing camera images");
DEFINE_string(frame_number,             "",             "frame number (6-digit zero-padded)");
DEFINE_string(pole_masks_dir,           "",             "path to folder containing pole masks");
DEFINE_string(output_dir,               "",             "path to write results");
DEFINE_string(rig_json_file,            "",             "path to json file drescribing camera array");
DEFINE_string(flow_alg,                 "pixflow_low",  "optical flow algorithm to use");
DEFINE_int32(alpha_feather_size,        31,             "feather on images before projection fixes an artifact");

// a standalone test of pole removal for the bottom cameras. reads a camera array
// description from --rig_json_file, camera images from --src_images_dir and pole masks
// from --pole_masks_dir. applies the optical flow algorithm specified by --flow_alg to
// generate an image combining the two bottom camera images but with the masked regions
// removed.
int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_src_images_dir, "src_images_dir");
  requireArg(FLAGS_frame_number, "frame_number");
  requireArg(FLAGS_pole_masks_dir, "pole_masks_dir");
  requireArg(FLAGS_output_dir, "output_dir");
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_flow_alg, "flow_alg");

  RigDescription rig(FLAGS_rig_json_file);
  const Camera& cam = rig.findCameraByDirection(-Camera::Vector3::UnitZ());
  const Camera& cam2 = rig.findLargestDistCamAxisToRigCenter();
  const bool flip180 = cam.up().dot(cam2.up()) < 0 ? true : false;

  system(std::string("mkdir -p " + FLAGS_output_dir + "/debug/" + FLAGS_frame_number).c_str());

  Mat bottomImage;
  static const bool kSaveDebugImages = true;
  static const bool kSaveExtraData = false;
  combineBottomImagesWithPoleRemoval(
    FLAGS_src_images_dir,
    FLAGS_frame_number,
    FLAGS_pole_masks_dir,
    "NONE",
    FLAGS_output_dir,
    kSaveDebugImages,
    kSaveExtraData,
    FLAGS_flow_alg,
    FLAGS_alpha_feather_size,
    rig.getBottomCameraId(),
    rig.getBottomCamera2Id(),
    Camera::approximateUsablePixelsRadius(cam),
    Camera::approximateUsablePixelsRadius(cam2),
    flip180,
    bottomImage);

  return EXIT_SUCCESS;
}
