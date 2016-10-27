#include "StillButton.hpp"

using namespace surround360;
using namespace Gtk;

StillButton::StillButton()
  : Gtk::Button() {
  set_label("Single frame");
}
