/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "FramerateComboBox.hpp"
#include <string>
#include <iostream>
#include "Config.hpp"

using namespace surround360;
using namespace Gtk;
using namespace std;

FramerateComboBox::FramerateComboBox()
  : Gtk::ComboBox() {
  m_refTreeModel = ListStore::create(m_fpsModel);
  set_model(m_refTreeModel);

  configureFps();
}

void FramerateComboBox::configureFps() {
  float fps = 5.0f;
  bool once = true;

  m_refTreeModel->clear();

  while (fps <= 30.0f) {
    TreeModel::Row row = *(m_refTreeModel->append());
    row[m_fpsModel.m_fps] = fps;

    if (once) {
      set_active(row);
      once = false;
    }
    fps += 1.0f;
  }

  pack_start(m_fpsModel.m_fps);
}

void FramerateComboBox::on_changed() {
  auto& cfg = CameraConfig::get();

  auto iter = get_active();
  auto row = *iter;
  cout << "Setting framerate to " << row[m_fpsModel.m_fps] << endl;
  cfg.fps = row[m_fpsModel.m_fps];
}
