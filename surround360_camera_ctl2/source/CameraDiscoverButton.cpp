#include "CameraDiscoverButton.hpp"
#include <iostream>

using namespace surround360;
using namespace Gtk;

CameraDiscoverButton::CameraDiscoverButton(CameraListView& listView)
  : Gtk::Button(), m_listViewRef(listView) {
  set_label("Refresh cameras");
}

void CameraDiscoverButton::on_clicked() {
  using namespace std;
  cout << "refreshing..." << endl;
  m_listViewRef.discoverCameras();
}
