/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "Config.hpp"

using namespace surround360;

CameraConfig& CameraConfig::get() {
  static CameraConfig config;
  return config;
}

CameraConfig::CameraConfig()
  : bits(12), triggerMode(0), frameInterval(1.0f) {
}
