/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <gtkmm.h>

#include "FpsListModel.hpp"

namespace surround360 {
  class FramerateComboBox : public Gtk::ComboBox {
  public:
    FramerateComboBox();
    void configureFps();
    void on_changed() override;

  protected:
    FpsListModel                 m_fpsModel;
    Glib::RefPtr<Gtk::ListStore> m_refTreeModel;
  };
}
