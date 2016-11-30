/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "ShutterComboBox.hpp"
#include <string>
#include <iostream>
#include "Config.hpp"

using namespace surround360;
using namespace Gtk;
using namespace std;

static const struct {
  float  speed;
  string name;
} shutterSpeeds[] = {
  { .speed = 1/4.0f,    .name = "1/4" },
  { .speed = 1/8.0f,    .name = "1/8" },
  { .speed = 1/10.0f,   .name = "1/10" },
  { .speed = 1/16.0f,   .name = "1/16" },
  { .speed = 1/25.0f,   .name = "1/25" },
  { .speed = 1/32.0f,   .name = "1/32" },
  { .speed = 1/50.0f,   .name = "1/50" },
  { .speed = 1/60.0f,   .name = "1/60" },
  { .speed = 1/100.0f,  .name = "1/100" },
  { .speed = 1/125.0f,  .name = "1/125" },
  { .speed = 1/150.0f,  .name = "1/150" },
  { .speed = 1/200.0f,  .name = "1/200" },  // 5ms
  { .speed = 1/250.0f,  .name = "1/250" },
  { .speed = 1/500.0f,  .name = "1/500" },
  { .speed = 1/600.0f,  .name = "1/600" },
  { .speed = 1/700.0f,  .name = "1/700" },
  { .speed = 1/1000.0f, .name = "1/1000" },
};

ShutterComboBox::ShutterComboBox()
  : Gtk::ComboBox() {
  m_refTreeModel = ListStore::create(m_shutterSpeedModel);
  set_model(m_refTreeModel);

  configureSpeeds(30.0f);
}

void ShutterComboBox::configureSpeeds(const float fps) {
  const float upperBound = (1.0f / fps) * 1000.0f;
  bool once = true;

  m_refTreeModel->clear();

  for (int k = 0; k < sizeof(shutterSpeeds)/sizeof(shutterSpeeds[0]); ++k) {

    if (shutterSpeeds[k].speed * 1000.0f > upperBound)
      continue;

    TreeModel::Row row = *(m_refTreeModel->append());
    row[m_shutterSpeedModel.m_speed] = shutterSpeeds[k].speed;
    row[m_shutterSpeedModel.m_name]  = shutterSpeeds[k].name;

    if (once) {
      set_active(row);
      once = false;
    }

  }

  pack_start(m_shutterSpeedModel.m_name);
}

void ShutterComboBox::on_changed() {
  auto& cfg = CameraConfig::get();
  auto iter = get_active();
  auto row = *iter;
  cfg.shutter = row[m_shutterSpeedModel.m_speed] * 1000.0f;
}
