#pragma once

#include <gtkmm.h>
#include "CameraListView.hpp"

namespace surround360 {
  class CameraDiscoverButton : public Gtk::Button {
  public:
    CameraDiscoverButton(CameraListView& listView);
    void on_clicked();

  private:
    CameraListView& m_listViewRef;
  };
}
