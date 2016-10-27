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
