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
  class PreviewCamListModel : public Gtk::TreeModel::ColumnRecord {
  public:
    PreviewCamListModel() {
      add(m_num);
      add(m_busNum);
    }

    Gtk::TreeModelColumn<unsigned int> m_num;
    Gtk::TreeModelColumn<unsigned int> m_busNum;
  };
}
