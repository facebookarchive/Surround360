/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

#include <Camera.hpp>

#include <iostream>
#include <memory>
#include <set>

#include <flycapture/FlyCapture2.h>

namespace surround360 {
  namespace fc = FlyCapture2;

  typedef std::tuple<unsigned int, unsigned int, unsigned int> SerialIndexTuple;
  typedef std::vector<SerialIndexTuple> SerialIndexVector;
  typedef SerialIndexVector::iterator SerialIndexIterator;

  class PointGreyCamera;
  using PointGreyCameraPtr = std::shared_ptr<PointGreyCamera>;

  class PointGreyCamera : protected Camera {
  public:
    static std::shared_ptr<PointGreyCamera> getCamera(const unsigned int index);
    static unsigned int findCameras();

    int attach();
    int detach();
    int init(
      const bool master,
      const double exposure,
      const double brightness,
      const double gamma,
      const double fps,
      const double shutter,
      const double gain,
      const unsigned int nbits);
    int stopCapture();
    int startCapture();
    int setMaster();

    void* getFrame(void* opaque);
    unsigned int getDroppedFramesCounter() const;
    int getSerialNumber() const;
    int getInterfaceSpeed() const;
    int reset();
    int powerCamera(bool onOff);
    int toggleStrobeOut(int pin, bool onOff);
    void prepareShutterSpeedUpdate(double shutter);
    void commitShutterSpeedUpdate();
    void prepareGainUpdate(double gain);
    void commitGainUpdate();

    bool setCameraProps(
      const std::pair<double, bool>& exposure,
      const std::pair<double, bool>& brightness,
      const std::pair<double, bool>& gamma,
      const std::pair<double, bool>& fps,
      const std::pair<double, bool>& shutter,
      const std::pair<double, bool>& gain);

    unsigned int frameWidth() override;
    unsigned int frameHeight() override;

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

    std::pair<float, float> getPropertyMinMax(CameraProperty p);
    ~PointGreyCamera();
    void updatePixelFormat(int bpp);

  private:
    std::shared_ptr<fc::Camera> m_camera;
    unsigned int m_droppedFrames {0};
    fc::PGRGuid m_guid;
    fc::InterfaceType m_ifaceType;

    double m_shutter{5.0};
    double m_gain{0.0};

    bool m_updateGain {false};
    bool m_updateShutter {false};

    mutable bool serialCached_ {false};
    mutable unsigned int serial_;
    bool m_master{false};

    unsigned int m_width {2448};
    unsigned int m_height {2048};

    const uint32_t kDataFlashCtrl = 0x1240;
    const uint32_t kDataFlashData = 0x1244;
    char *cameraBuffers;
    
  private:
    static fc::BusManager& getBusManager();

    PointGreyCamera(
      std::shared_ptr<fc::Camera> camera,
      fc::PGRGuid& guid,
      fc::InterfaceType iftype);

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

    uint32_t readRegister(uint32_t address);
    void writeRegister(uint32_t address, uint32_t value);
    bool isDataFlashSupported();
    uint32_t getDataFlashSize();
    uint64_t getDataFlashOffset();
    void commitPageToDataFlash();
    void throwError(const fc::Error& error);

    void readFileAtIndex(uint32_t idx);

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
