/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <gtkmm.h>

#include "GainListModel.hpp"

namespace surround360 {
  class GainComboBox : public Gtk::ComboBox {
  public:
    GainComboBox();
    void configureGains(const float maxGain);
    void on_changed();

  protected:
    GainListModel                 m_gainModel;
    Glib::RefPtr<Gtk::ListStore>  m_refTreeModel;
  };
}
