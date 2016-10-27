#pragma once

#include <gtkmm.h>

namespace surround360 {
  class FpsListModel : public Gtk::TreeModel::ColumnRecord {
  public:
    FpsListModel() {
      add(m_fps);
    }

    Gtk::TreeModelColumn<float>  m_fps;
  };
}
