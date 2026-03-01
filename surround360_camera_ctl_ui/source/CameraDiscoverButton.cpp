/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "CameraDiscoverButton.hpp"
#include <iostream>

using namespace surround360;
using namespace Gtk;

CameraDiscoverButton::CameraDiscoverButton(CameraListView& listView)
  : Gtk::Button(), m_listViewRef(listView) {
  set_label("Refresh cameras");
}

void CameraDiscoverButton::on_clicked() {
  using namespace std;
  cout << "refreshing..." << endl;
  m_listViewRef.discoverCameras();
}
