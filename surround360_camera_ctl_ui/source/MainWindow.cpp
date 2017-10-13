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
#include <utility>

#include "MainWindow.hpp"
#include "CameraController.hpp"
#include "Config.hpp"
#include "PointGrey.hpp"

using namespace surround360;
using namespace Gtk;

MainWindow::MainWindow()
  : Gtk::Window(),
    m_topVbox(ORIENTATION_HORIZONTAL, 10),
    m_controlVbox(ORIENTATION_VERTICAL, 10),
    m_cameraVbox(ORIENTATION_VERTICAL, 10),
    m_fpsVbox(ORIENTATION_VERTICAL, 10),
    m_24fps("24 fps"),
    m_30fps("30 fps"),
    m_timelapseHBox(ORIENTATION_HORIZONTAL, 10),
    m_timelapse("Timelapse"),
    m_timelapseAdjustment(Gtk::Adjustment::create(1.0, 1.0, 10.0, 1.0, 5.0, 0.0)),
    m_timelapseSpin(m_timelapseAdjustment),
    m_timelapseLabelSeconds("sec"),
    m_previewCamSelBox(),
    m_glarea(),
    m_cameraView(m_glarea),
    m_cameraListVbox(ORIENTATION_VERTICAL, 10),
    m_recordVbox(ORIENTATION_VERTICAL, 10),
    m_recordBtn(),
    m_previewBtn(),
    m_8bit("8-bit"),
    m_12bit("12-bit"),
    m_refreshCamerasBtn(m_cameraListView) {

  auto& ctl = CameraController::get(m_cameraView);

  set_title("Surround360");
  set_position(WindowPosition::WIN_POS_CENTER);
  set_default_size(1200, 800);

  m_topVbox.set_spacing(10);
  add(m_topVbox);

  m_controlVbox.set_border_width(5);
  m_controlVbox.set_spacing(10);
  m_topVbox.add(m_controlVbox);

  // configure FPS selection frame and combo box
  m_fpsFrame.set_label("Framerate");
  m_fpsFrame.set_border_width(5);
  m_fpsVbox.set_border_width(5);
  m_fpsVbox.add(m_30fps);
  m_fpsVbox.add(m_24fps);
  m_timelapse.set_tooltip_text("Seconds between frames");
  m_timelapseHBox.add(m_timelapse);
  m_timelapseSpin.set_wrap();
  m_timelapseHBox.add(m_timelapseSpin);
  m_timelapseHBox.add(m_timelapseLabelSeconds);
  m_fpsVbox.add(m_timelapseHBox);

  m_fpsFrame.add(m_fpsVbox);
  m_controlVbox.add(m_fpsFrame);
  
  pair<float, float> fpsMinMax =
    ctl.getPropertyMinMax(PointGreyCamera::CameraProperty::FRAME_RATE);
  m_fpsMin = ceil(fpsMinMax.first);

  // configure shutter selection frame and combo box
  m_shutterFrame.set_label("Shutter");
  m_shutterFrame.set_border_width(5);
  m_shutterFrame.add(m_shutterSelectionBox);
  m_shutterSelectionBox.set_border_width(5);
  m_controlVbox.add(m_shutterFrame);

  // configure gain selection frame and combo box
  m_gainFrame.set_label("Gain");
  m_gainFrame.set_border_width(5);
  m_gainFrame.add(m_gainSelectionBox);
  m_gainSelectionBox.set_border_width(5);
  m_controlVbox.add(m_gainFrame);

  pair<float, float> gainMinMax =
    ctl.getPropertyMinMax(PointGreyCamera::CameraProperty::GAIN);
  m_gainSelectionBox.configureGains(gainMinMax.second);

  // configure preview camera selection frame and combo box
  m_previewSelFrame.set_label("Preview");
  m_previewSelFrame.set_border_width(5);
  m_previewSelFrame.add(m_previewCamSelBox);
  m_previewCamSelBox.set_border_width(5);
  m_controlVbox.add(m_previewSelFrame);

  // add record frame and buttons
  m_opFrame.set_label("Control");
  m_opFrame.set_border_width(5);

  auto grpFps = m_30fps.get_group();
  m_24fps.set_group(grpFps);
  m_timelapse.set_group(grpFps);

  auto grpBitDepth = m_12bit.get_group();
  m_8bit.set_group(grpBitDepth);

  m_recordVbox.set_border_width(5);
  m_recordVbox.add(m_12bit);
  m_recordVbox.add(m_8bit);
  m_recordVbox.add(m_previewBtn);
  m_recordVbox.add(m_recordBtn);
  m_recordVbox.add(m_stillBtn);
  m_opFrame.add(m_recordVbox);

  m_controlVbox.add(m_opFrame);

  m_cameraVbox.set_border_width(5);
  m_topVbox.add(m_cameraVbox);

  m_glarea.set_hexpand(true);
  m_glarea.set_vexpand(true);
  m_glarea.set_auto_render(true);
  m_cameraVbox.add(m_glarea);

  m_glarea.signal_realize().connect(sigc::mem_fun(m_cameraView, &CameraView::onRealize));
  m_glarea.signal_render().connect(sigc::mem_fun(m_cameraView, &CameraView::onRender));
  m_glarea.signal_unrealize().connect(sigc::mem_fun(m_cameraView, &CameraView::onUnrealize));

  m_cameraListVbox.set_border_width(5);
  m_cameraListVbox.add(m_cameraListView);
  m_cameraListVbox.add(m_refreshCamerasBtn);

  m_cameraListView.discoverCameras();
  m_topVbox.add(m_cameraListVbox);

  m_previewCamSelBox.reload();

  connectSignals();

  // Force initial values
  bitSelectorClicked();
  fpsSelectorClicked();

  configureCameras();

  auto& cfg = CameraConfig::get();
  cout << "initial FPS: " << cfg.fps << endl;
  cout << "initial shutter: " << cfg.shutter << endl;
  cout << "initial gain: " << cfg.gain << endl;
  cout << "initial bits: " << cfg.bits << endl;
  
  m_previewBtn.set_sensitive(true);
  m_recordBtn.set_sensitive(false);
  m_stillBtn.set_sensitive(false);

  show_all();
}

MainWindow::~MainWindow() {
}

void MainWindow::connectSignals() {
  m_24fps.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::fpsSelectorClicked));
  m_30fps.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::fpsSelectorClicked));
  m_timelapse.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::fpsSelectorClicked));
  m_timelapseSpin.signal_value_changed().connect(sigc::mem_fun(*this, &MainWindow::timelapseSpinChanged));
  m_shutterSelectionBox.signal_changed().connect(sigc::mem_fun(this, &MainWindow::updatePreviewParams));
  m_gainSelectionBox.signal_changed().connect(sigc::mem_fun(this, &MainWindow::updatePreviewParams));
  m_previewBtn.signal_clicked().connect(sigc::mem_fun(this, &MainWindow::startPreview));
  m_recordBtn.signal_clicked().connect(sigc::mem_fun(this, &MainWindow::takeNameDialog));
  m_stillBtn.signal_clicked().connect(sigc::mem_fun(this, &MainWindow::singleTakeDialog));
  m_8bit.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::bitSelectorClicked));
  m_12bit.signal_toggled().connect(sigc::mem_fun(this, &MainWindow::bitSelectorClicked));
}

void MainWindow::updatePreviewParams() {
  auto& ctl = CameraController::get();
  auto& cfg = CameraConfig::get();

  ctl.updateCameraParams(cfg.shutter, cfg.fps, cfg.frameInterval, cfg.gain, cfg.bits);
}

void MainWindow::startPreview() {
  auto& ctl = CameraController::get();

  if (!m_previewBtn.isPreviewing()) {
    ctl.startProducer(1);
    ctl.startConsumers(2);
    m_previewBtn.setPreviewing(true);
    m_previewBtn.set_sensitive(false);
    m_recordBtn.set_sensitive(true);
    m_stillBtn.set_sensitive(true);
  }
}

void MainWindow::takeNameDialog() {
  m_previewBtn.set_sensitive(false);
  m_stillBtn.set_sensitive(false);
    
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
      m_24fps.set_sensitive(false);
      m_30fps.set_sensitive(false);
      m_timelapse.set_sensitive(false);
      m_8bit.set_sensitive(false);
      m_12bit.set_sensitive(false);
      m_shutterSelectionBox.set_sensitive(false);
      m_recordBtn.set_active();
      cout << "Started recording" << endl;
    } else {
      m_recordBtn.set_active(false);
    }
  } else {
    auto& ctl = CameraController::get();
    ctl.stopRecording();
    m_24fps.set_sensitive(true);
    m_30fps.set_sensitive(true);
    m_timelapse.set_sensitive(true);
    m_8bit.set_sensitive(true);
    m_8bit.set_sensitive(true);
    m_12bit.set_sensitive(true);
    m_shutterSelectionBox.set_sensitive(true);
    m_stillBtn.set_sensitive(true);
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

// If timelapse, we set fps to 10fps and get every N-th frame
void MainWindow::fpsSelectorClicked() {
  auto& cfg = CameraConfig::get();
  auto newFps = m_24fps.get_active() ? 24 : (m_30fps.get_active() ? 30 : m_fpsMin);
  const int newFrameInterval = m_timelapse.get_active()
    ? (m_timelapseSpin.get_value_as_int() * newFps) : 1;

  if (cfg.fps != newFps || newFrameInterval != cfg.frameInterval) {
    cfg.fps = newFps;
    cfg.frameInterval =
      m_timelapse.get_active() ? newFrameInterval : 1;

    auto& ctl = CameraController::get();
    if (m_timelapse.get_active()) {
      cout << "Setting timelapse every " << (newFrameInterval / m_fpsMin) << " seconds" << endl;
    } else {
      cout << "Setting FPS to " << newFps << endl;
    }

    ctl.updateCameraParams(
      cfg.shutter, cfg.fps, cfg.frameInterval, cfg.gain, cfg.bits);

    if (!m_previewBtn.isPreviewing()) {
      ctl.updateCameraParameters();
    }

    m_shutterSelectionBox.configureSpeeds(1000.0f / newFps);
  }
}

void MainWindow::timelapseSpinChanged() {
  m_timelapse.set_active();
  const int newFrameInterval = m_timelapseSpin.get_value_as_int() * m_fpsMin;

  auto& cfg = CameraConfig::get();
  if (cfg.fps != m_fpsMin || newFrameInterval != cfg.frameInterval) {
    cfg.fps = m_fpsMin;
    cfg.frameInterval = newFrameInterval;

    auto& ctl = CameraController::get();
    cout << "Setting timelapse every " << (newFrameInterval / m_fpsMin) << " seconds" << endl;

    ctl.updateCameraParams(
      cfg.shutter, cfg.fps, cfg.frameInterval, cfg.gain, cfg.bits);
  }
}

void MainWindow::bitSelectorClicked() {
  auto& cfg = CameraConfig::get();
  auto newbits = m_8bit.get_active() ? 8 : 12;
  if (cfg.bits != newbits) {
    cfg.bits = newbits;
    cfg.triggerMode = 0;

    auto& ctl = CameraController::get();
    cout << "Setting bit depth to " << newbits << endl;
    ctl.updateCameraParams(
      cfg.shutter, cfg.fps, cfg.frameInterval, cfg.gain, cfg.bits);
  }
}

void MainWindow::configureCameras() {
  auto& cfg = CameraConfig::get();
  auto& ctl = CameraController::get();
  ctl.configureCameras(
    cfg.shutter, cfg.fps, cfg.frameInterval, cfg.gain, cfg.bits);
}
