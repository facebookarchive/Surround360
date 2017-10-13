/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <atomic>
#include "Config.hpp"

namespace surround360 {
  class CameraConfig {
  public:
    std::atomic<float> shutter {10.0f};
    std::atomic<float> fps {0.0f};
    std::atomic<float> frameInterval {1.0f};
    std::atomic<float> whitebalance[2];
    std::atomic<float> bits {12};
    std::atomic<float> gain {0.0f};
    std::atomic<int>   previewCam {0};
    std::atomic<int>   triggerMode {0};
    std::atomic<bool>  raw {false};
  public:
    static CameraConfig& get();

  private:
    CameraConfig();

  };
}
