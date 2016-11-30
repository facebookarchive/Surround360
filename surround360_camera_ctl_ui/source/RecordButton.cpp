/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "RecordButton.hpp"
#include "CameraController.hpp"
#include "Config.hpp"
#include <iostream>
#include <pthread.h>

using namespace surround360;
using namespace Gtk;
using namespace std;

RecordButton::RecordButton()
  : Gtk::ToggleButton() {
  set_label("Record");
}
