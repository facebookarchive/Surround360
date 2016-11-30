/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <gtkmm.h>

namespace surround360 {
  class CameraListModel : public Gtk::TreeModel::ColumnRecord {
  public:
    CameraListModel() {
      add(m_name);
      add(m_serial);
      add(m_interface);
    }

    Gtk::TreeModelColumn<unsigned int>  m_name;
    Gtk::TreeModelColumn<unsigned int>  m_serial;
    Gtk::TreeModelColumn<Glib::ustring> m_interface;
  };
}
