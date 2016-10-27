#pragma once

#include <gtkmm.h>
#include "CameraView.hpp"
#include "ShutterComboBox.hpp"
#include "FramerateComboBox.hpp"

namespace surround360 {
  class RecordButton : public Gtk::ToggleButton {
  public:
    RecordButton();
  };
}
