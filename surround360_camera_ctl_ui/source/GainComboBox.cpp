/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include <iostream>
#include <string>

#include "GainComboBox.hpp"
#include "Config.hpp"

using namespace surround360;
using namespace Gtk;
using namespace std;

static const struct {
  float  gain;
  string name;
} gains[] = {
  { .gain = 0.0f, .name = "0 dB" },
  { .gain = 1.0f, .name = "1 dB" },
  { .gain = 2.0f, .name = "2 dB" },
  { .gain = 3.0f, .name = "3 dB" },
  { .gain = 4.0f, .name = "4 dB" },
  { .gain = 5.0f, .name = "5 dB" },
  { .gain = 6.0f, .name = "6 dB" },
  { .gain = 7.0f, .name = "7 dB" },
  { .gain = 8.0f, .name = "8 dB" },
  { .gain = 9.0f, .name = "9 dB" },
  { .gain = 10.0f, .name = "10 dB" },
};

GainComboBox::GainComboBox()
    : Gtk::ComboBox() {

  m_refTreeModel = ListStore::create(m_gainModel);
  set_model(m_refTreeModel);
  pack_start(m_gainModel.m_name);
}

void GainComboBox::configureGains(const float maxGain) {
  static const float kDefaultValue = 0.0f;
  m_refTreeModel->clear();

  for (int k = 0; k < sizeof(gains) / sizeof(gains[0]); ++k) {
    const float gain = gains[k].gain;
    const Glib::ustring name = gains[k].name;

    // Ignore gains outside range
    if (gain > maxGain) {
      continue;
    }

    TreeModel::Row row = *(m_refTreeModel->append());
    row[m_gainModel.m_gain] = gain;
    row[m_gainModel.m_name] = name;

    if (gain == kDefaultValue) {
      set_active(row);
    }
  }
}

void GainComboBox::on_changed() {
  auto& cfg = CameraConfig::get();
  auto iter = get_active();
  auto row = *iter;
  const float gain = row[m_gainModel.m_gain];

  if (gain != cfg.gain) {
    cout << "Setting gain to " << gain << endl;
    cfg.gain = gain;
  }
}
