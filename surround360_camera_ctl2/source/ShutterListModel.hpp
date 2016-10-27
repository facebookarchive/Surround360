#pragma once

#include <gtkmm.h>

namespace surround360 {
  class ShutterListModel : public Gtk::TreeModel::ColumnRecord {
  public:
    ShutterListModel() {
      add(m_name);
      add(m_speed);
    }

    Gtk::TreeModelColumn<Glib::ustring> m_name;
    Gtk::TreeModelColumn<float>  m_speed;
  };
}
