/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <vector>
#include <tuple>
#include <set>
#include <gtkmm.h>
#include "CameraListModel.hpp"
#include "PointGrey.hpp"

namespace surround360 {
  class CameraListView : public Gtk::TreeView {
  private:
    void getCameraSerialNumbers(SerialIndexVector& v);

  public:
    CameraListView();

    void discoverCameras();

    CameraListModel m_model;
    Glib::RefPtr<Gtk::ListStore>  m_store;
  };
}
