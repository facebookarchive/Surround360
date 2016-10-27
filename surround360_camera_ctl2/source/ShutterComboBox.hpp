#pragma once

#include <gtkmm.h>

#include "ShutterListModel.hpp"

namespace surround360 {
  class ShutterComboBox : public Gtk::ComboBox {
  public:
    ShutterComboBox();
    void configureSpeeds(const float fps);
    void on_changed();

  protected:
    ShutterListModel             m_shutterSpeedModel;
    Glib::RefPtr<Gtk::ListStore> m_refTreeModel;
  };
}
