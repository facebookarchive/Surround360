/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include <PointGrey.hpp>

#include <gflags/gflags.h>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <cstdlib>
#include <malloc.h>
#include <cassert>
#include "Config.hpp"

using namespace std;
using namespace surround360;
using namespace fc;

PointGreyCamera::PointGreyCamera(
  shared_ptr<fc::Camera> camera,
  fc::PGRGuid& guid,
  fc::InterfaceType iftype)
  : m_camera(camera),
    m_guid(guid),
    m_master(false),
    m_ifaceType(iftype),
    cameraBuffers(nullptr)
{
}

BusManager& PointGreyCamera::getBusManager() {
  static BusManager busMgr;
  return busMgr;
}

void PointGreyCamera::printError(int error, bool doExit) {
  if (error != PGRERROR_OK) {
    if (doExit) {
      throw "Error" + to_string(error);
    }
  }
}

void PointGreyCamera::printError(Error error, bool doExit) {
  cerr << error.GetDescription() << endl;
  printError(error.GetType(), doExit);
}

unsigned int PointGreyCamera::findCameras() {
  unsigned int numCameras = 0;

  getBusManager().GetNumOfCameras(&numCameras);
  return numCameras;
}

shared_ptr<PointGreyCamera> PointGreyCamera::getCamera(
  const unsigned int index) {

  shared_ptr<fc::Camera> camera(new fc::Camera());
  fc::PGRGuid guid;
  fc::InterfaceType ifaceType;

  getBusManager().GetCameraFromIndex(index, &guid);
  getBusManager().GetInterfaceTypeFromGuid(&guid, &ifaceType);

  PointGreyCameraPtr pgcam =
    shared_ptr<PointGreyCamera>(new PointGreyCamera(camera, guid, ifaceType));

  pgcam->attach();

  bool supported;
  Format7Info fmt7Info;
  fmt7Info.mode = MODE_7;

  camera->GetFormat7Info(&fmt7Info, &supported);

  if (!supported) {
    fmt7Info.mode = MODE_0;
    camera->GetFormat7Info(&fmt7Info, &supported);
  }

  pgcam->m_width = fmt7Info.maxWidth;
  pgcam->m_height = fmt7Info.maxHeight;

  return pgcam;
}

void* PointGreyCamera::getFrame(void* opaque) {
  fc::Image* img = reinterpret_cast<fc::Image*>(opaque);

  Error error = m_camera->RetrieveBuffer(img);

  if (error != PGRERROR_OK) {
    throw "Error retrieving frame buffer.";
  }

  m_droppedFrames = img->GetMetadata().embeddedFrameCounter;

  return img->GetData();
}

int PointGreyCamera::toggleStrobeOut(int pin, bool onOff) {
  StrobeControl strobeControl;
  strobeControl.source = pin;
  Error err = m_camera->GetStrobe(&strobeControl);
  if (err != PGRERROR_OK) {
    assert(!"FML");
    return err.GetType();
  }

  // Check if we need to change register
  if (strobeControl.onOff == onOff) {
    if (!onOff || (onOff &&
                   strobeControl.polarity == 0 &&
                   strobeControl.delay == 0.0f &&
                   strobeControl.duration == 0.0f)) {
      return 0;
    }
  }

  strobeControl.onOff = onOff;

  if (onOff) {
    strobeControl.polarity = 0; // low (falling edge) signal
    strobeControl.delay = 0.0f; // ms
    strobeControl.duration = 0.0f; // ms. duration 0 = shutter time
  }

  err = m_camera->SetStrobe(&strobeControl);
  return err.GetType();
}

unsigned int PointGreyCamera::getDroppedFramesCounter() const {
  return m_droppedFrames;
}

int PointGreyCamera::startCapture() {
  fc::Error error = m_camera->StartCapture();
  if (error != PGRERROR_OK) {
    cerr << "Error starting capture on camera " << getSerialNumber() << endl;
    cerr << "Error: " << error.GetType() << " " << error.GetDescription() << endl;
    error.PrintErrorTrace();
    throw "Error starting capture on camera " + to_string(getSerialNumber());
  }

  return error.GetType();
}

int PointGreyCamera::stopCapture() {
  fc::Error error = m_camera->StopCapture();
  if (error != PGRERROR_OK) {
    throw "Error stopping capture on camera " + to_string(getSerialNumber());
  }
  return error.GetType();
}

int PointGreyCamera::detach() {
  toggleStrobeOut(2, false);
  toggleStrobeOut(3, false);
  m_camera->StopCapture();
  fc::Error error = m_camera->Disconnect();
  return error.GetType();
}

int PointGreyCamera::attach() {
  fc::Error error = m_camera->Connect(&m_guid);
  return error.GetType();
}

int PointGreyCamera::getInterfaceSpeed() const {
  switch (m_ifaceType) {
  case fc::INTERFACE_USB3:
    return 3;

  case fc::INTERFACE_USB2:
  default:
    return 2;
  }
}

int PointGreyCamera::reset() {
  unsigned int numMemChannels;
  m_camera->GetMemoryChannelInfo(&numMemChannels);

  // Iterate to numMemChannels+1, since 0th channel is default
  for (unsigned int k = 0; k < numMemChannels + 1; ++k) {
    m_camera->RestoreFromMemoryChannel(k);
  }

  return 0;
}

pair<float, float> PointGreyCamera::getPropertyMinMax(CameraProperty p) {
  Property prop; 

  switch (p) {
    case CameraProperty::BRIGHTNESS:
      prop.type = fc::BRIGHTNESS;
      break;

    case CameraProperty::GAIN:
      prop.type = fc::GAIN;
      break;

    case CameraProperty::GAMMA:
      prop.type = fc::GAMMA;
      break;

    case CameraProperty::SHUTTER:
      prop.type = fc::SHUTTER;
      break;

    case CameraProperty::WHITE_BALANCE:
      prop.type = fc::WHITE_BALANCE;
      break;
  
    case CameraProperty::FRAME_RATE:
      prop.type = fc::FRAME_RATE;
      break;

    default:
      break;
  }

  PropertyInfo propInfo(prop.type);
  fc::Error error = m_camera->GetPropertyInfo(&propInfo);
  if (error != PGRERROR_OK) {
    throw "Error getting camera property.";
  }
  
  pair<float, float> minMax(propInfo.absMin, propInfo.absMax);
  return minMax;
}

int PointGreyCamera::powerCamera(bool on) {
  const unsigned int kCameraPower = 0x610;
  const unsigned int kPowerVal = on ? 0x80000000 : 0x00000000;
  const unsigned int kSleepMsPowerOff = 100;
  unsigned int regVal = 0;
  unsigned int retries = 10;

  // Check if we need to modify register
  m_camera->ReadRegister(kCameraPower, &regVal);
  if ((regVal & kPowerVal) != 0) {
    return 0;
  }

  printError(m_camera->WriteRegister(kCameraPower, kPowerVal));

  // Wait for camera to complete power-up
  do {
    usleep(kSleepMsPowerOff * 2000);

    Error error = m_camera->ReadRegister(kCameraPower, &regVal);
    if (error == PGRERROR_TIMEOUT) {
      // ignore timeout errors, camera may not be responding to
      // register reads during power-up
    }

    printError(error);

    retries--;
  } while ((regVal & kPowerVal) == 0 && retries > 0);

  return 0;
}

bool PointGreyCamera::pollForTriggerReady() {
  const unsigned int k_softwareTrigger = 0x62C;
  Error error;
  unsigned int regVal = 0;

  do {
    error = m_camera->ReadRegister(k_softwareTrigger, &regVal);
    if (error != PGRERROR_OK) {
      printError(error);
      return false;
    }
  } while ((regVal >> 31) != 0);

  return true;
}

int PointGreyCamera::init(
  const bool isMaster,
  const double exposure,
  const double brightness,
  const double gamma,
  const double fps,
  const double shutter,
  const double gain,
  const unsigned int nbits) {
  const int pinStrobe = 2; // pin #3 (red wire)
  const int pinTrigger = 3; // pin #4 (green wire)

  // Time (ms) to wait for an image when calling RetrieveBuffer
  const int timeoutBuffer = -1;
  fc::InterfaceType ifType;

  m_master = isMaster;

  getBusManager().GetInterfaceTypeFromGuid(&m_guid, &ifType);
  if (ifType != fc::INTERFACE_USB3) {
    return -1;
  }

  PixelFormat pf;
  switch (nbits) {
  case 16:
    pf = PIXEL_FORMAT_RAW16;
    break;
  case 12:
    pf = PIXEL_FORMAT_RAW12;
    break;
  case 8:
  default:
    pf = PIXEL_FORMAT_RAW8;
    break;
  }
  setPixelFormat(pf);
  setCameraTrigger(isMaster);

  if (!isMaster) {
    // Poll to ensure camera is ready
    if (!pollForTriggerReady()) {
      throw "Error polling for trigger ready!";
    }
  }

  // Set camera properties
  setCameraProps(make_pair(exposure, true),
                 make_pair(brightness, true),
                 make_pair(gamma, true),
                 make_pair(fps, true),
                 make_pair(shutter, true),
                 make_pair(gain, true));

  // Auto shutter
  if (shutter == 0.0f) {
    setCameraPropAbs(fc::SHUTTER, "0.000");
  }

  // embed frame counter into image metadata
  embedImageInfo();

  // Set grabbing mode and buffer retrieval timeout
  setCameraGrabMode(timeoutBuffer);

  const size_t kPageSize = 4096;
  const size_t kHorizRes = 2048;
  const size_t kNumBuffers = 7;
  const int kSingleCameraBufferSize = kPageSize * kHorizRes * kNumBuffers;
  cameraBuffers = reinterpret_cast<char*>(memalign(kPageSize, kSingleCameraBufferSize));

  if (cameraBuffers == nullptr) {
    throw "Not enough memory to allocate camera buffers";
  }
  m_camera->SetUserBuffers(
    reinterpret_cast<unsigned char*>(cameraBuffers), kPageSize * kHorizRes, kNumBuffers);

  return 0;
}

void PointGreyCamera::embedImageInfo() {
  EmbeddedImageInfo embeddedInfo;
  m_camera->GetEmbeddedImageInfo(&embeddedInfo);
  embeddedInfo.frameCounter.onOff = true;
  embeddedInfo.shutter.onOff = true;
  embeddedInfo.gain.onOff = true;
  embeddedInfo.timestamp.onOff = true;
  m_camera->SetEmbeddedImageInfo(&embeddedInfo);
}

std::ostream& PointGreyCamera::printCameraInfo(std::ostream& stream) const {
  CameraInfo camInfo;

  m_camera->GetCameraInfo(&camInfo);

  stream << "Serial number: " << getSerialNumber() << endl
         << "Camera model:  " << camInfo.modelName << endl
         << "Camera vendor: " << camInfo.vendorName << endl
         << "Sensor:        " << camInfo.sensorInfo << endl
         << "Resolution:    " << camInfo.sensorResolution << endl
         << "FW version:    " << camInfo.firmwareVersion << endl
         << "FW build time: " << camInfo.firmwareBuildTime << endl;

  return stream;
}

void PointGreyCamera::setCameraTrigger(bool isMaster) {
  TriggerMode triggerMode;

  m_camera->GetTriggerMode(&triggerMode);

  // Set external trigger params
  // If single image all cameras set to external trigger
  const bool isExternalTrigger = !isMaster;

  triggerMode.onOff = isExternalTrigger;

  // If camera set to trigger on external pulse, it will receive the trigger
  // via pin #4 (green wire)
  if (isExternalTrigger) {
    triggerMode.mode = CameraConfig::get().triggerMode;
    triggerMode.parameter = 0; // only necessary if multi-shot trigger
    triggerMode.source = 3;
  }

  m_camera->SetTriggerMode(&triggerMode);
}

bool PointGreyCamera::setCameraPropRel(
  PropertyType propType,
  unsigned int value) {
  Error error;

  // Get property
  Property prop;
  prop.type = propType;
  error = m_camera->GetProperty(&prop);
  if (error != PGRERROR_OK) {
    printError(error);
    return false;
  }

  // Modify property
  prop.autoManualMode = false;
  prop.valueA = value;

  // Set property
  error = m_camera->SetProperty(&prop);
  if (error != PGRERROR_OK) {
    printError(error);
    return false;
  }

  return true;
}

bool PointGreyCamera::setCameraPropAbs(
  PropertyType propType,
  const char* value) {
  Error error;
  const string val(value);

  // Get property
  Property prop;
  prop.type = propType;
  error = m_camera->GetProperty(&prop);
  if (error != PGRERROR_OK) {
    printError(error);
    return false;
  }

  PropertyInfo propInfo;
  propInfo.type = propType;
  error = m_camera->GetPropertyInfo(&propInfo);

  // Modify property
  if (propType == fc::SHUTTER && val == "0.000") {
    prop.autoManualMode = true; // sh = 0 enables auto mode
  } else {
    prop.autoManualMode = false;
  }
  if (propInfo.absValSupported) {
    prop.absControl = true;
  }
  prop.onePush = false;
  prop.onOff = true;

  if (propType == fc::FRAME_RATE && val == "0.000") {
    prop.onOff = false;
  } else {
    if (propType == fc::WHITE_BALANCE) {
      prop.onOff = false;
    } else {
      prop.absValue = stof(val);
    }
  }

  // Set property
  error = m_camera->SetProperty(&prop);
  if (error != PGRERROR_OK) {
    printError(error);
    return false;
  }

  return true;
}

void PointGreyCamera::prepareShutterSpeedUpdate(double shutter) {
  if (m_updateShutter) {
    m_shutter = shutter;
  }
}

void PointGreyCamera::commitShutterSpeedUpdate() {
  if (m_updateShutter) {
    auto err = setCameraPropAbs(fc::SHUTTER, to_string(m_shutter).c_str());
    m_updateShutter = false;
  }
}

void PointGreyCamera::prepareGainUpdate(double gain) {
  if (m_updateGain) {
    m_gain = gain;
  }
}

void PointGreyCamera::commitGainUpdate() {
  if (m_updateGain) {
    auto err = setCameraPropAbs(fc::GAIN, to_string(m_gain).c_str());
    m_updateGain = false;
  }
}

bool PointGreyCamera::setCameraProps(
  const pair<double, bool>& exposure,
  const pair<double, bool>& brightness,
  const pair<double, bool>& gamma,
  const pair<double, bool>& fps,
  const pair<double, bool>& shutter,
  const pair<double, bool>& gain) {

  const struct {
    const fc::PropertyType type;
    const bool active;
    const string value;
    const string name;
  } params[] = {
    { .type = fc::AUTO_EXPOSURE,
      .active = exposure.second,
      .value = to_string(exposure.first),
      .name = "AUTO_EXP" },
    { .type = fc::BRIGHTNESS,
      .active = brightness.second,
      .value = to_string(brightness.first),
      .name = "BRIGHTNESS" },
    { .type = fc::GAMMA,
      .active = gamma.second,
      .value = to_string(gamma.first),
      .name = "GAMMA"},
    { .type = fc::FRAME_RATE,
      .active = fps.second,
      .value = to_string(fps.first),
      .name = "FPS" },
    { .type = fc::SHUTTER,
      .active = shutter.second,
      .value = to_string(shutter.first),
      .name = "SHUTTER" },
    { .type = fc::GAIN,
      .active = gain.second,
      .value = to_string(gain.first),
      .name = "GAIN" },
    { .type = fc::WHITE_BALANCE,
      .active = true,
      .value = "0 0",
      .name = "WHITE_BALANCE" }
  };

  for (int k = 0; k < sizeof(params)/sizeof(params[0]); ++k) {
    if (params[k].active) {
      if (!setCameraPropAbs(params[k].type, params[k].value.c_str())) {
        return false;
      }
    }
  }

  return true;
}

void PointGreyCamera::setCameraGrabMode(int timeoutBuffer) {
  // Get the camera configuration
  FC2Config config;
  m_camera->GetConfiguration(&config);

  // Set the grab timeout (miliseconds) - TIMEOUT_INFINITE to wait indefinitely
  config.grabTimeout = timeoutBuffer;

  // Set grab mode
  // According to the libdc1394 standard, "setting the drop_frames member of the
  // dc1394_cameracapture structure to a non-zero value causes the DMA capture
  // functions to throw away all frames buffered in the DMA ring buffer except
  // the latest one, which is returned to you.
  // BUFFER_FRAMES uses the camera buffer to store some frames so we don't drop
  // them
  config.grabMode = BUFFER_FRAMES;
  config.grabTimeout = timeoutBuffer;

  // Setting a large number of buffers for each camera may cause the program to
  // hang trying to allocate resources. Try less than 10
  config.numBuffers = 5;

  // Enbale high performance mode
  config.highPerformanceRetrieveBuffer = true;

  // Set the camera configuration
  m_camera->SetConfiguration(&config);
}

void PointGreyCamera::updatePixelFormat(int bpp) {
  fc::PixelFormat pf;
  switch (bpp) {
  case 16:
    pf = PIXEL_FORMAT_RAW16;
    break;
  case 12:
    pf = PIXEL_FORMAT_RAW12;
    break;
  case 8:
  default:
    pf = PIXEL_FORMAT_RAW8;
  }

  setPixelFormat(pf);
}

void PointGreyCamera::setPixelFormat(PixelFormat pf) {
  Mode mode = MODE_7;
  bool supported;

  // Query for available Format 7 modes
  Format7Info fmt7Info;
  fmt7Info.mode = mode;

  m_camera->GetFormat7Info(&fmt7Info, &supported);

  if (!supported) {
    cerr << "Warning: MODE_7 not supported. Falling back to MODE_0" << endl;
    mode = MODE_0;
    fmt7Info.mode = mode;
    printError(m_camera->GetFormat7Info(&fmt7Info, &supported));
  }

  Format7ImageSettings fmt7ImageSettings;
  fmt7ImageSettings.mode = mode;
  fmt7ImageSettings.offsetX = 0;
  fmt7ImageSettings.offsetY = 0;
  fmt7ImageSettings.width = fmt7Info.maxWidth;
  fmt7ImageSettings.height = fmt7Info.maxHeight;

  fmt7ImageSettings.pixelFormat = pf;

  bool valid;
  Format7PacketInfo fmt7PacketInfo;

  // Validate the settings
  printError(m_camera->ValidateFormat7Settings(
               &fmt7ImageSettings,
               &valid,
               &fmt7PacketInfo));

  if (!valid) {
    // Settings are not valid
    cerr << "Format7 settings are not valid" << endl;
    exit(EXIT_FAILURE);
  }

  // Set the settings to the camera
  printError(m_camera->SetFormat7Configuration(
               &fmt7ImageSettings,
               fmt7PacketInfo.maxBytesPerPacket));
}

PointGreyCamera::~PointGreyCamera() {
  detach();

  if (cameraBuffers != nullptr) {
    free(cameraBuffers);
  }
}

ostream& surround360::operator<<(ostream& stream, const PointGreyCamera& c) {
  c.printCameraInfo(stream);
  return stream;
}

ostream& surround360::operator<<(ostream& stream, const PointGreyCameraPtr& cp) {
  stream << *cp;
  return stream;
}

unsigned int PointGreyCamera::frameWidth() {
  return m_width;
}

unsigned int PointGreyCamera::frameHeight() {
  return m_height;
}

int PointGreyCamera::getSerialNumber() const {
  CameraInfo camInfo;

  if (!serialCached_) {
    m_camera->GetCameraInfo(&camInfo);
    serial_ = camInfo.serialNumber;
    serialCached_ = true;
  }
  return serial_;
}


uint32_t PointGreyCamera::readRegister(uint32_t address) {
    uint32_t value;
    fc::Error error = m_camera->ReadRegister(address, &value);
    if (error != fc::PGRERROR_OK) {
        throwError(error);
    }

    return value;
}

void PointGreyCamera::writeRegister(uint32_t address, uint32_t value) {
    fc::Error error = m_camera->WriteRegister(address, value);
    if (error != fc::PGRERROR_OK) {
        throwError(error);
    }
}

bool PointGreyCamera::isDataFlashSupported() {
    uint32_t value = readRegister(kDataFlashCtrl);

    // bit 0 https://www.ptgrey.com/tan/10370
    const uint32_t Presence_Inc = 0x80000000;
    return value & Presence_Inc ? true : false;
}

uint32_t PointGreyCamera::getDataFlashSize() {
    uint32_t value = readRegister(kDataFlashCtrl);

    // bit 8-19 https://www.ptgrey.com/tan/10370
    const uint32_t Page_Size = 0x00FFF000;
    const uint32_t pageSizeExponent = (value & Page_Size) >> 12;

    // bit 20-31 https://www.ptgrey.com/tan/10370
    const uint32_t Num_Pages = 0x00000FFF;
    const uint32_t numPagesExponent = (value & Num_Pages) >> 0;

    return uint32_t(1) << (pageSizeExponent + numPagesExponent);
}

uint64_t PointGreyCamera::getDataFlashOffset() {
    uint32_t value = readRegister(kDataFlashData);

    // from looking at point grey sample code, value appears to be measured in
    // 'quartets'
    // Also: 'Addresses are offsets from the IEEE-1394 base address', see
    // https://www.ptgrey.com/tan/10370
    const uint64_t IEEE_1394_base_address = 0xFFFFF0000000;
    return IEEE_1394_base_address + value * sizeof(uint32_t);
}

void PointGreyCamera::commitPageToDataFlash() {
    uint32_t value = readRegister(kDataFlashCtrl);

    // bit 6 https://www.ptgrey.com/tan/10370
    const uint32_t Clean_Page = 0x02000000;
    value |= Clean_Page;

    writeRegister(kDataFlashCtrl, value);
}

void PointGreyCamera::throwError(const fc::Error& error) {
    error.PrintErrorTrace();
    throw std::runtime_error(error.GetDescription());
}

void PointGreyCamera::readFileAtIndex(uint32_t fileIdx) {
  const uint64_t kFlashOffset = getDataFlashOffset();

  uint64_t offset = kFlashOffset;
  uint32_t recordSize = 0;
  const uint32_t flashSize = getDataFlashSize();

  for (uint32_t currIdx = 0; currIdx != fileIdx && offset < flashSize; ++currIdx) {
    uint32_t currRecordSize = 0;
    auto err = m_camera->ReadRegisterBlock(
      static_cast<uint32_t>(offset >> 32),
      static_cast<uint32_t>(offset),
      &currRecordSize,
      1);

    if (err != fc::PGRERROR_OK) {
      throwError(err);
    }

    offset += currRecordSize + 1;
  }

  uint32_t recSize = 0;
  auto err = m_camera->ReadRegisterBlock(
    static_cast<uint32_t>(offset >> 32),
    static_cast<uint32_t>(offset),
    &recSize,
    1);

  vector<uint32_t> data(recSize, 0);
  err = m_camera->ReadRegisterBlock(
    static_cast<uint32_t>(offset >> 32),
    static_cast<uint32_t>(offset),
    &data[0],
    data.size());

  if (err != fc::PGRERROR_OK) {
    throwError(err);
  }
}
