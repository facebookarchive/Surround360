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
