/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */

#pragma once

#include <Camera.hpp>

#include <iostream>
#include <memory>
#include <set>

#include <flycapture/FlyCapture2.h>

namespace surround360 {
  namespace fc = FlyCapture2;

  class PointGreyCamera;
  typedef std::shared_ptr<PointGreyCamera> PointGreyCameraPtr;

  class PointGreyCamera : protected Camera {
  public:
    static const unsigned int FRAME_WIDTH = 2048;
    static const unsigned int FRAME_HEIGHT = 2048;
    static const unsigned int FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT;

    static std::shared_ptr<PointGreyCamera> getCamera(const unsigned int index);
    static unsigned int findCameras();

    int attach();
    int detach();
    int init(bool isMaster = false);
    int stopCapture();
    int startCapture();
    int setMaster();

    void* getFrame();
    unsigned int getDroppedFramesCounter() const;
    int getSerialNumber() const;
    int reset();
    int powerCamera(bool onOff);
    int toggleStrobeOut(int pin, bool onOff);
    void prepareShutterSpeedUpdate(double shutter);
    void commitShutterSpeedUpdate();
    void prepareGainUpdate(double gain);
    void commitGainUpdate();

    static void printError(int error, bool doExit = true);
    static void printError(fc::Error error, bool doExit = true);

    enum CameraProperty {
      BRIGHTNESS = 0,
      GAIN,
      GAMMA,
      SHUTTER,
      WHITE_BALANCE,
      FRAME_RATE,
    };

    std::string getProperty(CameraProperty p);

    ~PointGreyCamera();

  private:
    std::shared_ptr<fc::Camera> m_camera;
    bool isMaster;
    unsigned int droppedFramesCounter;
    fc::PGRGuid m_guid;
    fc::InterfaceType m_ifaceType;

    double m_shutterSpeedUpdate;
    double m_shutterSpeed;
    double m_gainUpdate;
    double m_gain;

  private:
    static fc::BusManager& getBusManager();

    PointGreyCamera(std::shared_ptr<fc::Camera>& camera, fc::PGRGuid& guid);
    PointGreyCamera(PointGreyCamera& camera);

    void setCameraTrigger(bool isMaster);

    bool pollForTriggerReady();
    void setCameraGrabMode(int timeoutBuffer);

    void updateDroppedFrames(
      fc::ImageMetadata meta,
      unsigned int* droppedFrames,
      unsigned int* droppedFramesCur,
      unsigned int i);

    void setCameraPixelFormat(fc::PixelFormat pf);

    bool setCameraProps();

    bool setCameraPropRel(
      fc::PropertyType propType,
      unsigned int value);

    bool setCameraPropAbs(
      fc::PropertyType propType,
      const char* value);


    void  printAndSaveCameraProperties(bool isRaw);

    std::ostream& printCameraInfo(std::ostream& stream) const;

    void saveCMDArgs(const std::string& destDir, int argc, char* argv[]);

    void saveDroppedFrames(
      unsigned int droppedFrames,
      const std::string& destDir);

    std::string getPropsRange(bool isRaw);

    float getPropAbsFromRel(
      fc::Camera* pCam,
      fc::PropertyType propType,
      unsigned int value);

    std::string getPropRange(fc::PropertyType propType);

    int getPropRelFromAbs(
      fc::PropertyType propType,
      const char* value);

    void embedImageInfo();

    void setPixelFormat(fc::PixelFormat pf);

    void getPixelFormatFromBitDepth(fc::PixelFormat* pf, unsigned int nBits);

    friend std::ostream& operator<<(
      std::ostream& stream,
      const PointGreyCamera& c);

  };

  std::ostream& operator<<(
    std::ostream& stream,
    const PointGreyCamera& c);

  std::ostream& operator<<(
    std::ostream& stream,
    const PointGreyCameraPtr& c);
}
