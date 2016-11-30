/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "PreviewCamComboBox.hpp"
#include "PointGrey.hpp"
#include <string>
#include <iostream>
#include <memory>
#include "CameraController.hpp"
#include "Config.hpp"

using namespace surround360;
using namespace Gtk;
using namespace std;

PreviewCamComboBox::PreviewCamComboBox()
  : Gtk::ComboBox() {
  m_refTreeModel = ListStore::create(m_model);
  set_model(m_refTreeModel);
}

void PreviewCamComboBox::reload() {
  SerialIndexVector serials;

  m_refTreeModel->clear();

  const unsigned int n = PointGreyCamera::findCameras();
  for (unsigned int k = 0; k < n; ++k) {
    PointGreyCameraPtr c = PointGreyCamera::getCamera(k);
    serials.emplace_back(k, c->getSerialNumber(), 0);
  }

  sort(serials.begin(), serials.end(),
       [](const SerialIndexTuple& left, const SerialIndexTuple& right)
       { return get<1>(left) < get<1>(right); });

  for (auto m = 0; m < serials.size(); ++m) {
    TreeModel::Row row = *(m_refTreeModel->append());
    row[m_model.m_num] = m;
    row[m_model.m_busNum] = get<0>(serials[m]);

    if (m == 0) {
      set_active(row);
    }
  }

  pack_start(m_model.m_num);
}

void PreviewCamComboBox::on_changed() {
  try {
    CameraController& ctl = CameraController::get();
    auto iter = get_active();
    auto& cfg = CameraConfig::get();

    cfg.previewCam = (*iter)[m_model.m_busNum];
    ctl.setPreviewCamera(cfg.previewCam);
  } catch (...) {
    cout << "Error when setting a preview camera" << endl;
  }
}
