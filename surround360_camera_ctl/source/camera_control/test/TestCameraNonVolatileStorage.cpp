/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */

#include <iostream>
#include <fstream>

#include "PointGrey.hpp"

#include <gflags/gflags.h>

using namespace std;
using namespace surround360;
using namespace fc;

// Needed for compilation. PointGreyCamera class expects these
DEFINE_double(brightness,   10.449,   "Set brightness value.");
DEFINE_double(exposure,     0.850,    "Set exposure value.");
DEFINE_double(fps,          30.0,     "Set frame rate.");
DEFINE_double(gain,         0.0,      "Set gain.");
DEFINE_double(gamma,        1.250,    "Set gamma.");
DEFINE_double(shutter,      20.0,     "Set shutter speed.");

DEFINE_string(write,        "",       "file to copy into storage");
DEFINE_string(read,         "",       "file to copy storage into");

int main(int argc, char* argv[]) {
  std::string exe = argv[0];
  google::SetUsageMessage(
      "Read or write camera's non-volatile storage:\n"
      "  " + exe + " [-write | -read] <filename>"
  );
  google::ParseCommandLineNonHelpFlags(&argc, &argv, true);

  if (FLAGS_read.empty() && FLAGS_write.empty()) {
    google::ShowUsageWithFlags(argv[0]);
    return EXIT_SUCCESS;
  }

  const int numCameras = PointGreyCamera::findCameras();

  cout << "Number of cameras detected: " << numCameras << endl;
  if (numCameras != 1) {
    cout << "Expected a single camera, exiting" << endl;
    throw std::runtime_error("Expected a single camera");
  }

  // Connect to the camera
  PointGreyCameraPtr camera = PointGreyCamera::getCamera(0);

  // Power on the camera
  camera->powerCamera(true);

  // Print camera information
  cout << camera << endl;

  if (!camera->isDataFlashSupported()) {
    cout << "Camera has no non-volatile storage" << endl;
    throw std::runtime_error("Camera has no non-volatile storage");
  }

  if (!FLAGS_read.empty()) {
    cout << "Starting to read " << FLAGS_read << " from flash" << endl;

    camera->readFromInternalStorage(FLAGS_read);

    cout << "Successfully read file from flash" << endl;
  }

  if (!FLAGS_write.empty()) {
    cout << "Starting to write " << FLAGS_write << " to flash" << endl;

    std::ifstream file(FLAGS_write, std::ios::binary);
    if (!file) {
      cout << "Unable to open file" << endl;
      throw std::runtime_error("Unable to open file");
    }

    std::vector<char> data {
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>()
    };
    cout << "File size is " << data.size() << " bytes" << endl;

    camera->writeToInternalStorage(data);

    cout << "Successfully wrote file to flash" << endl;
  }

  // Disconnect the camera
  camera->detach();

  return EXIT_SUCCESS;
}
