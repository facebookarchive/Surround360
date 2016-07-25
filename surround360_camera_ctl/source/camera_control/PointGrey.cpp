/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */

#include <PointGrey.hpp>

#include <gflags/gflags.h>
#include <iomanip>
#include <sstream>
#include <unistd.h>

using namespace std;
using namespace surround360;
using namespace fc;

DECLARE_double(fps);
DECLARE_double(gain);
DECLARE_double(gamma);
DECLARE_double(exposure);
DECLARE_double(shutter);
DECLARE_double(brightness);

PointGreyCamera::PointGreyCamera(
  shared_ptr<fc::Camera>& camera,
  fc::PGRGuid& guid)
  : m_camera(camera),
    isMaster(false),
    m_guid(guid),
    m_shutterSpeedUpdate(0.0),
    m_shutterSpeed(FLAGS_shutter),
    m_gainUpdate(0.0),
    m_gain(FLAGS_gain) {
}

BusManager& PointGreyCamera::getBusManager() {
  static BusManager busMgr;
  return busMgr;
}

void PointGreyCamera::printError(int error, bool doExit) {
  if (error != PGRERROR_OK) {
    if (doExit) {
      exit(EXIT_FAILURE);
    }
  }
}

void PointGreyCamera::printError(Error error, bool doExit) {
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
  PointGreyCameraPtr pgcam = shared_ptr<PointGreyCamera>(new PointGreyCamera(camera, guid));

  pgcam->attach();

  return pgcam;
}

void* PointGreyCamera::getFrame() {
  Image rawImage;
  ImageMetadata metadata;

  Error error = m_camera->RetrieveBuffer(&rawImage);
  if (error != PGRERROR_OK) {
    throw "Error retrieving frame buffer.";
  }

  metadata = rawImage.GetMetadata();
  droppedFramesCounter = metadata.embeddedFrameCounter;

  return rawImage.GetData();
}

int PointGreyCamera::toggleStrobeOut(int pin, bool onOff) {
  StrobeControl strobeControl;
  strobeControl.source = pin;
  Error err = m_camera->GetStrobe(&strobeControl);
  if (err != PGRERROR_OK) {
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
  return droppedFramesCounter;
}

int PointGreyCamera::startCapture() {
  fc::Error error = m_camera->StartCapture();
  if (error != PGRERROR_OK) {
    cerr << "Error starting capture..." << endl;
    cerr << "Error: " << error.GetType() << " " << error.GetDescription() << endl;
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
  fc::Error error = m_camera->Disconnect();
  return error.GetType();
}

int PointGreyCamera::attach() {
  fc::Error error = m_camera->Connect(&m_guid);
  return error.GetType();
}

int PointGreyCamera::setMaster() {
  isMaster = true;
  return 0;
}

int PointGreyCamera::getSerialNumber() const {
  CameraInfo camInfo;
  m_camera->GetCameraInfo(&camInfo);
  return camInfo.serialNumber;
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

string PointGreyCamera::getProperty(CameraProperty p) {
  Property prop;
  ostringstream oss;

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

  default:
    break;
  }

  PropertyInfo propInfo(prop.type);
  fc::Error error = m_camera->GetPropertyInfo(&propInfo);
  if (error != PGRERROR_OK) {
    throw "Error getting camera property.";
  }

  if (propInfo.absValSupported) {
    oss << setprecision(3) << fixed
        << float(propInfo.min) << ";" << float(propInfo.max) << ";"
        << propInfo.pUnitAbbr;
  } else {
    oss << float(propInfo.min) << ";" << float(propInfo.max);
  }
  return oss.str();
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
    usleep(kSleepMsPowerOff * 1000);

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

int PointGreyCamera::init(bool isMaster) {
  const int pinStrobe = 2; // pin #3 (red wire)
  const int pinTrigger = 3; // pin #4 (green wire)

  // Time (ms) to wait for an image when calling RetrieveBuffer
  const int timeoutBuffer = -1;
  fc::InterfaceType ifType;

  getBusManager().GetInterfaceTypeFromGuid(&m_guid, &ifType);
  if (ifType != fc::INTERFACE_USB3) {
    return -1;
  }

  setPixelFormat(PIXEL_FORMAT_RAW8);
  setCameraTrigger(isMaster);

  if (!isMaster) {
    // Poll to ensure camera is ready
    if (!pollForTriggerReady()) {
      throw "Error polling for trigger ready!";
    }
  }

  // Set camera properties
  setCameraProps();

  // Auto shutter
  if (FLAGS_shutter == 0.0f) {
    setCameraPropAbs(fc::SHUTTER, "0.000");
  }

  // embed frame counter into image metadata
  embedImageInfo();

  // Set grabbing mode and buffer retrieval timeout
  setCameraGrabMode(timeoutBuffer);
}

void PointGreyCamera::embedImageInfo() {
  EmbeddedImageInfo embeddedInfo;
  m_camera->GetEmbeddedImageInfo(&embeddedInfo);
  embeddedInfo.frameCounter.onOff = true;
  embeddedInfo.shutter.onOff = false;
  embeddedInfo.gain.onOff = false;
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

  // Check if trigger is already set
  if ((triggerMode.onOff == isExternalTrigger && isExternalTrigger == false)
      ||
      (triggerMode.onOff == isExternalTrigger &&
       triggerMode.mode == 0 &&
       triggerMode.parameter == 0 &&
       triggerMode.source == 3)) {
    return;
  }

  triggerMode.onOff = isExternalTrigger;

  // If camera set to trigger on external pulse, it will receive the trigger
  // via pin #4 (green wire)
  if (isExternalTrigger) {
    triggerMode.mode = 0;
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
    printError( error );
    return false;
  }

  // Modify property
  prop.autoManualMode = false;
  prop.valueA = value;

  // Set property
  error = m_camera->SetProperty(&prop);
  if (error != PGRERROR_OK) {
    printError( error );
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
    printError( error );
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
  if (shutter != m_shutterSpeed) {
    m_shutterSpeedUpdate = shutter;
  }
}

void PointGreyCamera::commitShutterSpeedUpdate() {
  if (m_shutterSpeedUpdate != 0.0) {
    setCameraPropAbs(fc::SHUTTER, to_string(m_shutterSpeedUpdate).c_str());
    m_shutterSpeed = m_shutterSpeedUpdate;
    m_shutterSpeedUpdate = 0.0;
  }
}

void PointGreyCamera::prepareGainUpdate(double gain) {
  if (gain != m_gain) {
    m_gainUpdate = gain;
  }
}

void PointGreyCamera::commitGainUpdate() {
  if (m_gainUpdate != 0.0) {
    setCameraPropAbs(fc::GAIN, to_string(m_gainUpdate).c_str());
    m_gain = m_gainUpdate;
    m_gainUpdate = 0.0;
  }
}

bool PointGreyCamera::setCameraProps() {
  if (!setCameraPropAbs(
        fc::AUTO_EXPOSURE,
        to_string(FLAGS_exposure).c_str())) {
    return false;
  }

  if (!setCameraPropAbs(
        fc::BRIGHTNESS,
        to_string(FLAGS_brightness).c_str())) {
    return false;
  }

  if (!setCameraPropAbs(
        fc::GAMMA,
        to_string(FLAGS_gamma).c_str())) {
    return false;
  }

  if (!setCameraPropAbs(
        fc::FRAME_RATE,
        to_string(FLAGS_fps).c_str())) {
    return false;
  }

  if (!setCameraPropAbs(
        fc::SHUTTER,
        to_string(FLAGS_shutter).c_str())) {
    return false;
  }

  if (!setCameraPropAbs(
        fc::GAIN,
        to_string(FLAGS_gain).c_str())) {
    return false;
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

  // Setting a large number of buffers for each camera may cause the program to
  // hang trying to allocate resources. Try less than 10
  config.numBuffers = 5;

  // Enbale high performance mode
  config.highPerformanceRetrieveBuffer = true;

  // Set the camera configuration
  m_camera->SetConfiguration(&config);
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
}

ostream& surround360::operator<<(ostream& stream, const PointGreyCamera& c) {
  c.printCameraInfo(stream);
  return stream;
}

ostream& surround360::operator<<(ostream& stream, const PointGreyCameraPtr& cp) {
  stream << *cp;
  return stream;
}
