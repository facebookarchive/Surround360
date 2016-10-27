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
