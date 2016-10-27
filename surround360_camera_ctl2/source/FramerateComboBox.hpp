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
