/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "CameraListView.hpp"

using namespace surround360;
using namespace std;

CameraListView::CameraListView()
  : Gtk::TreeView(),
    m_model(),
    m_store(Gtk::ListStore::create(m_model)) {

  set_model(m_store);
  append_column("Camera", m_model.m_name);
  append_column("Serial #", m_model.m_serial);
  append_column("Interface", m_model.m_interface);

  discoverCameras();
}

void CameraListView::discoverCameras() {
  SerialIndexVector cameraSerials;
  SerialIndexVector sorted;
  int k = 1;
  getCameraSerialNumbers(cameraSerials);

  sort(cameraSerials.begin(), cameraSerials.end(),
       [](const SerialIndexTuple& left, const SerialIndexTuple& right)
       { return get<1>(left) < get<1>(right); });

  m_store->clear();
  for (auto&& var : cameraSerials) {
    Gtk::TreeModel::Row row = *(m_store->append());
    row[m_model.m_name]      = k;
    row[m_model.m_serial]    = get<1>(var);
    row[m_model.m_interface] = "usb " + to_string(get<2>(var));
    ++k;
  }

  show_all_children();
}

void CameraListView::getCameraSerialNumbers(SerialIndexVector& v) {
  unsigned int ncameras = 0;

  ncameras = PointGreyCamera::findCameras();

  for (int k = 0; k < ncameras; ++k) {
    PointGreyCameraPtr c = PointGreyCamera::getCamera(k);
    v.push_back(std::make_tuple(k, c->getSerialNumber(), c->getInterfaceSpeed()));
  }
}
