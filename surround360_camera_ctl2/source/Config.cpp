#include "Config.hpp"

using namespace surround360;

CameraConfig& CameraConfig::get() {
  static CameraConfig config;
  return config;
}

CameraConfig::CameraConfig()
  : bits(8), triggerMode(0) {
}
