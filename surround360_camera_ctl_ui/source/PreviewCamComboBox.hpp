/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <gtkmm.h>

#include "PreviewCamListModel.hpp"

namespace surround360 {
  class PreviewCamComboBox : public Gtk::ComboBox {
  public:
    PreviewCamComboBox();
    void reload();

  protected:
    PreviewCamListModel          m_model;
    Glib::RefPtr<Gtk::ListStore> m_refTreeModel;
    void on_changed() override;
  };
}
