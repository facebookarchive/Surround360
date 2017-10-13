/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include "CameraView.hpp"
#include "ShutterComboBox.hpp"
#include "GainComboBox.hpp"
#include "RecordButton.hpp"
#include "StillButton.hpp"
#include "CameraListView.hpp"
#include "CameraDiscoverButton.hpp"
#include "PreviewCamComboBox.hpp"
#include "PreviewButton.hpp"

namespace surround360 {
  class MainWindow : public Gtk::Window {
  public:
    MainWindow();
    ~MainWindow() override;

  private:
    void connectSignals();
    void startPreview();
    void updatePreviewParams();
    void takeNameDialog();
    void fpsSelectorClicked();
    void timelapseSpinChanged();
    void bitSelectorClicked();
    void configureCameras();
    void singleTakeDialog();

  protected:
    Gtk::Box                        m_topVbox;
    Gtk::Box                        m_controlVbox;
    Gtk::Box                        m_cameraVbox;

    Gtk::Frame                      m_fpsFrame;
    Gtk::Box                        m_fpsVbox;
    Gtk::RadioButton                m_24fps;
    Gtk::RadioButton                m_30fps;
    Gtk::Box                        m_timelapseHBox;
    Gtk::RadioButton                m_timelapse;
    Glib::RefPtr<Gtk::Adjustment>   m_timelapseAdjustment;
    Gtk::SpinButton                 m_timelapseSpin;
    Gtk::Label                      m_timelapseLabelSeconds;
    int                             m_fpsMin;

    Gtk::Frame                      m_shutterFrame;
    ShutterComboBox                 m_shutterSelectionBox;

    Gtk::Frame                      m_gainFrame;
    GainComboBox                    m_gainSelectionBox;

    Gtk::Frame                      m_previewSelFrame;
    PreviewCamComboBox              m_previewCamSelBox;

    Gtk::GLArea                     m_glarea;
    CameraView                      m_cameraView;

    Gtk::Box                        m_cameraListVbox;
    CameraListView                  m_cameraListView;
    CameraDiscoverButton            m_refreshCamerasBtn;

    Gtk::Frame                      m_opFrame;
    Gtk::Box                        m_recordVbox;
    Gtk::RadioButton                m_8bit;
    Gtk::RadioButton                m_12bit;
    RecordButton                    m_recordBtn;
    PreviewButton                   m_previewBtn;
    StillButton                     m_stillBtn;
  };
}
