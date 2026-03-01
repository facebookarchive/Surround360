/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <gtkmm.h>
#include "CameraListView.hpp"

namespace surround360 {
  class CameraDiscoverButton : public Gtk::Button {
  public:
    CameraDiscoverButton(CameraListView& listView);
    void on_clicked();

  private:
    CameraListView& m_listViewRef;
  };
}
