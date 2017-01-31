/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include <gtkmm.h>
#include <gdkmm.h>
#include <GL/gl.h>
#include <GL/freeglut_std.h>

#include <iterator>
#include <memory>

#include "MainWindow.hpp"
#include "CameraController.hpp"
#include "PointGrey.hpp"
#include "Config.hpp"

using namespace surround360;
using namespace Gtk;

MainWindow::MainWindow()
  : Gtk::Window(),
    m_topVbox(ORIENTATION_HORIZONTAL, 10),
    m_controlVbox(ORIENTATION_VERTICAL, 10),
    m_cameraVbox(ORIENTATION_VERTICAL, 10),
    m_previewCamSelBox(),
    m_glarea(),
    m_cameraView(m_glarea),
    m_cameraListVbox(ORIENTATION_VERTICAL, 10),
    m_recordVbox(ORIENTATION_VERTICAL, 10),
    m_recordBtn(),
    m_previewBtn(),
    m_8bit("8-bit"),
    m_12bit("12-bit"),
    m_isp("ISP"),
    m_raw("Raw Bayer"),
    m_rotationPicker("Load rotations..."),
    m_refreshCamerasBtn(m_cameraListView) {

  auto& ctl = CameraController::get(m_cameraView);

  set_title("Surround360");
  set_position(WindowPosition::WIN_POS_CENTER);
  set_default_size(1200, 800);

  m_topVbox.set_spacing(10);
  add(m_topVbox);

  m_controlVbox.set_spacing(10);
  m_topVbox.add(m_controlVbox);

  // configure FPS selection frame and combo box
  m_fpsFrame.set_label("Framerate");
  m_fpsFrame.add(m_fpsSelectionBox);
  m_controlVbox.add(m_fpsFrame);

  // configure shutter selection frame and combo box
  m_shutterFrame.set_label("Shutter");
  m_shutterFrame.add(m_shutterFrameVbox);

  m_shutterFrameVbox.set_orientation(ORIENTATION_VERTICAL);
  m_shutterFrameVbox.add(m_shutterOverrideEntry);
  m_shutterFrameVbox.add(m_shutterSelectionBox);

  m_controlVbox.add(m_shutterFrame);

  // configure preview camera selection frame and combo box
  m_previewSelFrame.set_label("Preview");
  m_previewSelFrame.add(m_previewCamSelBox);
  m_controlVbox.add(m_previewSelFrame);

  // add record frame and buttons
  m_opFrame.set_label("Control");
  m_opFrame.add(m_recordVbox);
  m_recordVbox.set_orientation(ORIENTATION_VERTICAL);

  auto grp = m_8bit.get_group();
  m_12bit.set_group(grp);

  auto grp2 = m_isp.get_group();
  m_raw.set_group(grp2);

  m_recordVbox.add(m_8bit);
  m_recordVbox.add(m_12bit);
  m_recordVbox.add(m_isp);
  m_recordVbox.add(m_raw);

  m_recordVbox.add(m_previewBtn);
  m_recordVbox.add(m_recordBtn);
  m_recordVbox.add(m_stillBtn);
  m_recordVbox.add(m_rotationPicker);
  m_controlVbox.add(m_opFrame);

  m_topVbox.add(m_cameraVbox);

  m_glarea.set_hexpand(true);
  m_glarea.set_vexpand(true);
  m_glarea.set_auto_render(true);
  m_cameraVbox.add(m_glarea);

  m_glarea.signal_realize().connect(sigc::mem_fun(m_cameraView, &CameraView::onRealize));
  m_glarea.signal_render().connect(sigc::mem_fun(m_cameraView, &CameraView::onRender));
  m_glarea.signal_unrealize().connect(sigc::mem_fun(m_cameraView, &CameraView::onUnrealize));

  m_cameraListVbox.add(m_cameraListView);
  m_cameraListVbox.add(m_refreshCamerasBtn);

  m_cameraListView.discoverCameras();
  m_topVbox.add(m_cameraListVbox);

  m_previewCamSelBox.reload();

  connectSignals();

  show_all();
}

MainWindow::~MainWindow() {
}

void MainWindow::connectSignals() {
  m_shutterSelectionBox.signal_changed().connect(sigc::mem_fun(this, &MainWindow::updatePreviewParams));
  m_fpsSelectionBox.signal_changed().connect(sigc::mem_fun(this, &MainWindow::updatePreviewParams));
  m_stillBtn.signal_clicked().connect(sigc::mem_fun(this, &MainWindow::singleTakeDialog));
  m_recordBtn.signal_clicked().connect(sigc::mem_fun(this, &MainWindow::takeNameDialog));
  m_8bit.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::bitSelectorClicked));
  m_12bit.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::bitSelectorClicked));
  m_raw.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::rawIspSelector));
  m_shutterOverrideEntry.signal_activate().connect(sigc::mem_fun(this, &MainWindow::shutterOverride));
  m_shutterOverrideEntry.signal_focus_out_event().connect(sigc::mem_fun(this, &MainWindow::shutterOverrideFocusOut));

  m_rotationPicker.signal_clicked().connect(sigc::mem_fun(this, &MainWindow::rotationFilePicker));
}

void MainWindow::shutterOverride() {
  auto str = m_shutterOverrideEntry.get_text();
  try {
    auto val = std::stof(str);
    auto& cfg = CameraConfig::get();
    cfg.shutter = val;

    updatePreviewParams();
  } catch (...) {
    return;
  }
}

bool MainWindow::shutterOverrideFocusOut(GdkEventFocus* evt) {
  shutterOverride();
  return true;
}

void MainWindow::updatePreviewParams() {
  auto& ctl = CameraController::get();
  auto& cfg = CameraConfig::get();

  ctl.updateCameraParams(cfg.shutter, cfg.fps, cfg.gain, cfg.bits);
}

void MainWindow::takeNameDialog() {
  if (m_recordBtn.get_active()) {
    vector<string> dirnames(2);
    for (auto k = 0; k < dirnames.size(); ++k) {
      string title = "Select a destination folder " + to_string(k + 1);
      Gtk::FileChooserDialog dlg(title, Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);

      dlg.set_transient_for(*this);
      dlg.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
      dlg.add_button("_Select", Gtk::RESPONSE_OK);

      int result = dlg.run();
      string dirname;

      switch (result) {
      case Gtk::RESPONSE_OK:
        dirname = dlg.get_filename();
        if (dirname.size() > 0) {
          dirnames[k] = dirname;
        }
        break;

      default:
        break;
      }
    }

    if (all_of(dirnames.cbegin(), dirnames.cend(),
	       [](const string& filepath) { return filepath.size() > 0; })) {

      auto& ctl = CameraController::get();
      ctl.setPaths(dirnames);
      ctl.startRecording();
      m_8bit.set_sensitive(false);
      m_12bit.set_sensitive(false);
      m_fpsSelectionBox.set_sensitive(false);
      m_shutterSelectionBox.set_sensitive(false);
      m_recordBtn.set_active();
      cout << "Started recording" << endl;
    } else {
      m_recordBtn.set_active(false);
    }
  } else {
    auto& ctl = CameraController::get();
    ctl.stopRecording();

    m_8bit.set_sensitive(true);
    m_12bit.set_sensitive(true);
    m_fpsSelectionBox.set_sensitive(true);
    m_shutterSelectionBox.set_sensitive(true);
  }
}

void MainWindow::singleTakeDialog() {
  Gtk::FileChooserDialog dlg(
    "Select a destination folder",
    Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);

  dlg.set_transient_for(*this);
  dlg.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
  dlg.add_button("_Select", Gtk::RESPONSE_OK);

  int result = dlg.run();
  string dirname;

  switch (result) {
  case Gtk::RESPONSE_OK:
    dirname = dlg.get_filename();
    if (dirname.size() > 0) {
      auto& ctl = CameraController::get();
      vector<string> paths { dirname, dirname };
      ctl.setPaths(paths);
      ctl.startRecording(true);
    }
    break;

  default:
    break;
    // do nothing
  }
}

void MainWindow::rawIspSelector() {
  auto& cfg = CameraConfig::get();
  cfg.raw = m_raw.get_active() ? true : false;
}

void MainWindow::bitSelectorClicked() {
  auto& cfg = CameraConfig::get();
  auto newbits = m_8bit.get_active() ? 8 : 12;
  if (cfg.bits != newbits) {
    cfg.bits = newbits;
    cfg.triggerMode = 0;

    auto& ctl = CameraController::get();
    ctl.updateCameraParams(cfg.shutter, cfg.fps, cfg.gain, cfg.bits);
  }
}

void MainWindow::rotationFilePicker() {
  Gtk::FileChooserDialog dlg(
    "Select a camera rotation description file",
    Gtk::FILE_CHOOSER_ACTION_OPEN);

  dlg.set_transient_for(*this);
  dlg.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
  dlg.add_button("_Select", Gtk::RESPONSE_OK);

  int result = dlg.run();
  string fname;

  switch (result) {
  case Gtk::RESPONSE_OK:
    fname = dlg.get_filename();
    if (fname.size() > 0) {
      auto params = make_unique<vector<float>>();
      ifstream stream(fname);
      auto it = istream_iterator<float>(stream);
      copy(it, istream_iterator<float>(), back_inserter(*params));
      m_cameraView.setRotations(params);
    }
    break;

  default:
    break;
    // do nothing
  }
}
