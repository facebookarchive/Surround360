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
#include "FramerateComboBox.hpp"
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
    void updatePreviewParams();
    void takeNameDialog();
    void bitSelectorClicked();
    void rawIspSelector();
    void singleTakeDialog();
    void rotationFilePicker();
    void shutterOverride();
    bool shutterOverrideFocusOut(GdkEventFocus* evt);

  protected:
    Gtk::Box             m_topVbox;
    Gtk::Box             m_controlVbox;
    Gtk::Box             m_cameraVbox;

    Gtk::Frame           m_fpsFrame;
    FramerateComboBox    m_fpsSelectionBox;

    Gtk::Entry           m_shutterOverrideEntry;
    Gtk::Box             m_shutterFrameVbox;
    Gtk::Frame           m_shutterFrame;
    ShutterComboBox      m_shutterSelectionBox;

    Gtk::Frame           m_previewSelFrame;
    PreviewCamComboBox   m_previewCamSelBox;

    Gtk::GLArea          m_glarea;
    CameraView           m_cameraView;

    Gtk::Box             m_cameraListVbox;
    CameraListView       m_cameraListView;
    CameraDiscoverButton m_refreshCamerasBtn;

    Gtk::Frame           m_opFrame;
    Gtk::Box             m_recordVbox;
    Gtk::RadioButton     m_8bit;
    Gtk::RadioButton     m_12bit;
    Gtk::RadioButton     m_isp;
    Gtk::RadioButton     m_raw;
    RecordButton         m_recordBtn;
    PreviewButton        m_previewBtn;
    StillButton          m_stillBtn;

    Gtk::Button          m_rotationPicker;
  };
}
