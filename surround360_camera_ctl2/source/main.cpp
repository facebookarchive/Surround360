#include <iostream>
#include <set>
#include <vector>
#include <tuple>
#include <cassert>
#include <glibmm/arrayhandle.h>
#include <gtkmm.h>
#include <gtkmm/glarea.h>
#include <GL/freeglut_std.h>

#include "PointGrey.hpp"
#include "CameraListView.hpp"
#include "CameraView.hpp"
#include "MainWindow.hpp"

using namespace std;
using namespace surround360;

int main(int argc, char *argv[]) {
  auto app = Gtk::Application::create(argc, argv, "surround360.camera_control");

  MainWindow window;
  app->run(window);

  return 0;
}
