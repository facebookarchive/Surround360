#include "RecordButton.hpp"
#include "CameraController.hpp"
#include "Config.hpp"
#include <iostream>
#include <pthread.h>

using namespace surround360;
using namespace Gtk;
using namespace std;

RecordButton::RecordButton()
  : Gtk::ToggleButton() {
  set_label("Record");
}
