/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "StillButton.hpp"

using namespace surround360;
using namespace Gtk;

StillButton::StillButton()
  : Gtk::Button() {
  set_label("Single frame");
}
