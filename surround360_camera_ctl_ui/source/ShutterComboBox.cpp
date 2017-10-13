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
  { .speed = 1 / 4.0f,      .name = "1/4" },        // 250 ms
  { .speed = 1 / 8.0f,      .name = "1/8" },        // 125 ms
  { .speed = 1 / 10.0f,     .name = "1/10" },       // 100 ms
  { .speed = 1 / 16.0f,     .name = "1/16" },       // 62.5 ms
  { .speed = 1 / 25.0f,     .name = "1/25" },       // 40 ms
  { .speed = 1 / 32.0f,     .name = "1/32" },       // 31.25 ms
  { .speed = 1 / 50.0f,     .name = "1/50" },       // 20 ms
  { .speed = 1 / 60.0f,     .name = "1/60" },       // 16.66 ms
  { .speed = 1 / 100.0f,    .name = "1/100" },      // 10 ms
  { .speed = 1 / 125.0f,    .name = "1/125" },      // 8 ms
  { .speed = 1 / 150.0f,    .name = "1/150" },      // 6.67 ms
  { .speed = 1 / 200.0f,    .name = "1/200" },      // 5 ms
  { .speed = 1 / 250.0f,    .name = "1/250" },      // 4 ms
  { .speed = 1 / 500.0f,    .name = "1/500" },      // 2 ms
  { .speed = 1 / 600.0f,    .name = "1/600" },      // 1.67 ms
  { .speed = 1 / 700.0f,    .name = "1/700" },      // 1.43 ms
  { .speed = 1 / 1000.0f,   .name = "1/1000" },     // 1 ms
  { .speed = 1 / 2000.0f,   .name = "1/2000" },     // 0.5 ms
  { .speed = 1 / 4000.0f,   .name = "1/4000" },     // 0.25 ms
  { .speed = 1 / 8000.0f,   .name = "1/8000" },     // 0.125 ms
  { .speed = 1 / 12000.0f,  .name = "1/12000" },    // 0.083 ms
};

ShutterComboBox::ShutterComboBox()
    : Gtk::ComboBox() {

  m_canUpdate = true;
  m_isFirstConfig = true;
  m_refTreeModel = ListStore::create(m_shutterSpeedModel);
  set_model(m_refTreeModel);
  pack_start(m_shutterSpeedModel.m_name);
}

void ShutterComboBox::configureSpeeds(const float maxShutterMs) {
  static const float kDefaultValue = 1 / 100.0f;

  m_canUpdate = false;
  m_refTreeModel->clear();
  m_canUpdate = true;

  auto& cfg = CameraConfig::get();

  bool updateSelection = false;
  for (int k = 0; k < sizeof(shutterSpeeds) / sizeof(shutterSpeeds[0]); ++k) {
    const float speed = shutterSpeeds[k].speed;
    const Glib::ustring name = shutterSpeeds[k].name;
    const float speedMs = 1000.f * speed;

    // Ignore shutters outside frame rate range
    if (speedMs > maxShutterMs) {
      // If selected shutter is outside range we will have to update it to the
      // next available value
      if (speedMs == cfg.shutter) {
        updateSelection = true;
      }
      continue;
    }

    TreeModel::Row row = *(m_refTreeModel->append());
    row[m_shutterSpeedModel.m_speed] = speed;
    row[m_shutterSpeedModel.m_name]  = name;

    // Set active row
    if (m_isFirstConfig && speed == kDefaultValue) {
      set_active(row);
      m_isFirstConfig = false;
    } else if (speedMs == cfg.shutter) {
      set_active(row);
    } else if (updateSelection) {
      set_active(row);
      updateSelection = false;
    }
  }
}

void ShutterComboBox::on_changed() {
  if (!m_canUpdate) {
    return;
  }

  auto& cfg = CameraConfig::get();
  auto iter = get_active();
  auto row = *iter;
  const float speedMs = 1000.0f * row[m_shutterSpeedModel.m_speed];

  if (speedMs != cfg.shutter) {
    cout << "Setting shutter to " << speedMs << "ms" << endl;
    cfg.shutter = speedMs;
  }
}
