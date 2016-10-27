#include "PreviewButton.hpp"
#include "CameraController.hpp"
#include "Config.hpp"
#include <iostream>
#include <pthread.h>

using namespace surround360;
using namespace Gtk;
using namespace std;

PreviewButton::PreviewButton()
  : Gtk::Button(),
    m_previewing(false) {
  set_label("Start Preview");
}

void PreviewButton::on_clicked() {
  CameraController& controller = CameraController::get();

  if (!m_previewing) {
    auto& cfg = CameraConfig::get();

    controller.configureCameras(cfg.shutter, cfg.fps, cfg.gain, cfg.bits);
    controller.startProducer(1);
    controller.startConsumers(2);
    controller.startSlaveCapture();
    controller.startMasterCapture();

    m_previewing = true;
  }
}
