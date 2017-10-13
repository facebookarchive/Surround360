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
  class GainListModel : public Gtk::TreeModel::ColumnRecord {
  public:
    GainListModel() {
      add(m_name);
      add(m_gain);
    }

    Gtk::TreeModelColumn<Glib::ustring> m_name;
    Gtk::TreeModelColumn<float>  m_gain;
  };
}
