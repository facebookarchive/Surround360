#pragma once

#include <atomic>
#include "Config.hpp"

namespace surround360 {
  class CameraConfig {
  public:
    std::atomic<float> shutter {10.0f};
    std::atomic<float> fps {5.0f};
    std::atomic<float> whitebalance[2];
    std::atomic<float> bits {8};
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
