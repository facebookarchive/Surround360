/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include <iostream>
#include <set>
#include <vector>
#include <tuple>
#include <cassert>
#include <glibmm/arrayhandle.h>
#include <gtkmm.h>
#include <gtkmm/glarea.h>
#include <GL/freeglut_std.h>

#include "PointGrey.hpp"
#include "CameraListView.hpp"
#include "CameraView.hpp"
#include "MainWindow.hpp"

using namespace std;
using namespace surround360;

int main(int argc, char *argv[]) {
  std::ios::sync_with_stdio(false);

  Gtk::Main kit(argc, argv);
  MainWindow window;
  kit.run(window);

  return 0;
}
