#pragma once

#include <gtkmm.h>
#include "CameraView.hpp"
#include "ShutterComboBox.hpp"
#include "FramerateComboBox.hpp"

namespace surround360 {
  class PreviewButton : public Gtk::Button {
  public:
    PreviewButton();

  protected:
    void on_clicked() override;

  private:
    bool m_previewing;
  };
}
