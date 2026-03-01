/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#include "CameraController.hpp"
#include <algorithm>
#include <limits>
#include <tuple>
#include <thread>
#include <cassert>
#include <algorithm>

#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include "Config.hpp"

using namespace surround360;
using namespace std;

static shared_ptr<CameraController> globalController;
static pthread_barrier_t startBarrier;

CameraController::CameraController(
  CameraView& camview,
  const unsigned int nproducers,
  const unsigned int nconsumers)
  : m_brightness(10.449f),
    m_exposure(0.850f),
    m_gamma(1.250f),
    m_keepRunning(true),
    m_recording(false),
    m_startRecording(false),
    m_stopRecording(false),
    m_pinStrobe(2),
    m_pinTrigger(3),
    m_cameraView(camview),
    m_previewIndex(-1),
    m_running(false),
    m_oneshot(false),
    m_producerCount(nproducers),
    m_consumerCount(nconsumers),
    m_consumerBuf(nconsumers),
    m_dirname(nconsumers) {
  m_width = PointGreyCamera::getCamera(0)->frameWidth();
  m_height = PointGreyCamera::getCamera(0)->frameHeight();
}

CameraController& CameraController::get(
  CameraView& camview,
  const unsigned int nproducers,
  const unsigned int nconsumers) {

  if (globalController == nullptr) {
    globalController = shared_ptr<CameraController>(new CameraController(camview, nproducers, nconsumers));
  }

  return *globalController;
}

CameraController& CameraController::get() {
  if (globalController != nullptr) {
    return *globalController;
  } else {
    throw "CameraController not initialized";
  }
}

pair<float, float> CameraController::getPropertyMinMax(
    PointGreyCamera::CameraProperty property) {

  const unsigned int ncameras = PointGreyCamera::findCameras();

  if (ncameras > 0) {
    PointGreyCameraPtr c = PointGreyCamera::getCamera(0);
    return c->getPropertyMinMax(property);
  }
}

bool CameraController::configureCameras(
  const float shutter,
  const float framerate,
  const int frameInterval,
  const float gain,
  const int bitsPerPixel) {

  // Time (ms) to wait for an image when calling RetrieveBuffer
  const int timeoutBuffer = -1;

  m_shutter = shutter;
  m_framerate = framerate;
  m_frameInterval = frameInterval;
  m_gain = gain;
  m_bitsPerPixel = bitsPerPixel;

  const unsigned int nCams = PointGreyCamera::findCameras();
  m_masterCameraSerial = INT_MAX;
  m_masterCameraIndex = -1;

  for (int k = 0; k < nCams; ++k) {
    PointGreyCameraPtr camera = PointGreyCamera::getCamera(k);

    if (camera->getSerialNumber() < m_masterCameraSerial) {
      m_masterCameraSerial = camera->getSerialNumber();
      m_masterCameraIndex = k;
    }
    m_camera.push_back(camera);
  }

  for (int k = 0; k < m_camera.size(); ++k) {
    try {
      bool master = k == m_masterCameraIndex;
      m_camera[k]->powerCamera(true);
      if (-1 == m_camera[k]->init(
        master,
        m_exposure,
        m_brightness,
        m_gamma,
        m_framerate,
        m_shutter,
        m_gain,
        m_bitsPerPixel)) {
        throw "error initializing camera " + to_string(k);
      }
    } catch (string& s) {
      cerr << "Exception: " << s << endl;
      m_camera[k]->toggleStrobeOut(m_pinStrobe, false);
      throw "Error powering on and initializing one of the cameras.";
    }
  }

  return true;
}

bool CameraController::updateCameraParams(
  const float shutter,
  const float fps,
  const int frameInterval,
  const float gain,
  const int bits) {

  if (shutter != m_shutter) {
    m_shutter = shutter;
  }

  if (fps != m_framerate) {
    m_framerate = fps;
  }

  if (frameInterval != m_frameInterval) {
    m_frameInterval = frameInterval;
  }

  if (gain != m_gain) {
    m_gain = gain;
  }

  if (bits != m_bitsPerPixel) {
    m_bitsPerPixel = bits;
  }

  m_paramUpdatePending = true;
}

void CameraController::startProducer(const unsigned int count) {
  pthread_barrier_init(&startBarrier, nullptr, m_producerCount + m_consumerCount);

  for (int pid = 0; pid < count; ++pid) {
    m_prodThread.emplace_back(&CameraController::cameraProducer, this, pid);
  }
}

void CameraController::startConsumers(const unsigned int count) {
  for (int cid = 0; cid < count; ++cid) {
    m_consThread.emplace_back(&CameraController::cameraConsumer, this, cid);
    m_consThread.back().detach();
  }
}

void CameraController::setThreadPriority(const uint32_t cpuNumber)
{
  cpu_set_t threadCpuAffinity;
  CPU_ZERO(&threadCpuAffinity);
  CPU_SET(cpuNumber, &threadCpuAffinity);
  sched_setaffinity(0, sizeof(threadCpuAffinity), &threadCpuAffinity);

  sched_param sparam;
  sparam.sched_priority = 99;
  sched_setscheduler(0, SCHED_RR, &sparam);
}

void CameraController::updateCameraParameters()
{
  for (auto& cam : m_camera) {
    cam->setCameraProps(
      make_pair(m_exposure, false),
      make_pair(m_brightness, false),
      make_pair(m_gamma, false),
      make_pair(m_framerate, true),
      make_pair(m_shutter, true),
      make_pair(m_gain, true));

    cam->updatePixelFormat(m_bitsPerPixel);
  }
}

void CameraController::startMainCamera()
{
  m_camera[m_masterCameraIndex]->startCapture();
  m_camera[m_masterCameraIndex]->toggleStrobeOut(m_pinStrobe, true);
}

void CameraController::stopMainCamera()
{
  m_camera[m_masterCameraIndex]->toggleStrobeOut(m_pinStrobe, false);
  m_camera[m_masterCameraIndex]->stopCapture();
}

void CameraController::startOtherCameras()
{
  for (auto i = 0; i < m_camera.size(); ++i) {
    if (i == m_masterCameraIndex) {
      continue;
    }
    m_camera[i]->startCapture();
  }
}

void CameraController::stopOtherCameras()
{
  for (auto i = 0; i < m_camera.size(); ++i) {
    if (i == m_masterCameraIndex) {
      continue;
    }
    m_camera[i]->toggleStrobeOut(m_pinTrigger, false);
    m_camera[i]->stopCapture();
  }
}

void CameraController::cameraProducer(const uint32_t id) {
  const size_t camerasPerProducer = m_camera.size() / m_producerCount;
  const size_t cameraOffset = id * camerasPerProducer;
  const int lastCamera = std::min(cameraOffset + camerasPerProducer, m_camera.size());
  uint32_t frameNumber = 0;
  vector<fc::Image> frame(m_camera.size());
  vector<uint32_t> frameCount(m_camera.size());
  vector<uint32_t> frameCounter(m_camera.size());
  vector<uint32_t> prevFrameCounter(m_camera.size());
  fc::Image previewFrame;

  const size_t maxRes = 4096;
  vector<uint8_t, aligned_allocator<uint8_t, 64>> tmpFrameBuf(maxRes * maxRes);

  setThreadPriority(id);


  m_camera[m_masterCameraIndex]->toggleStrobeOut(m_pinStrobe, false);

  for (auto i = 0; i < m_camera.size(); ++i) {
    if (i == m_masterCameraIndex)
      continue;

    m_camera[i]->startCapture();
  }

  m_camera[m_masterCameraIndex]->startCapture();
  m_camera[m_masterCameraIndex]->toggleStrobeOut(m_pinStrobe, true);

  m_running = true;
  m_paramUpdatePending = true;
  pthread_barrier_wait(&startBarrier);

  while (m_keepRunning) {
    if (!m_recording && m_paramUpdatePending) {
      stopMainCamera();
      stopOtherCameras();

      updateCameraParameters();

      startOtherCameras();
      startMainCamera();
      m_paramUpdatePending = false;
    }

    if (m_oneshot && !m_recording) {
      m_startRecording = true;
    }

    if (m_startRecording) {
      m_startRecording = false;
      m_recording = true;

      pthread_barrier_wait(&startBarrier);
    }

    if (m_stopRecording) {
      for (auto& cbuf : m_consumerBuf) {
        cbuf.done();
      }

      m_stopRecording = false;
      m_recording = false;

      pthread_barrier_wait(&startBarrier);
    }

    // The main frame grab loop
    for (auto i = cameraOffset; i < lastCamera; ++i) {
      const int cid = i % m_consumerCount; // ping-pong between output threads

      FramePacket* nextFrame = nullptr;

      try {
        // Retrieve an image from buffer
        void* bytes = m_camera[i]->getFrame(&frame[i]);

        // loop invariant; if this fails, it means the program is not written correctly
        assert(bytes != nullptr);

        prevFrameCounter[i] = frameCounter[i];
        frameCounter[i] = frame[i].GetMetadata().embeddedFrameCounter;

        if (prevFrameCounter[i] != 0 && (frameCounter[i] - prevFrameCounter[i]) != 1) {
          cerr << "camera " << i << " dropped " << (frameCounter[i] - prevFrameCounter[i]) << " frames" << endl;
        }

        if ((++frameCount[i]) % m_frameInterval > 0) {
          continue;
        }

        if (m_recording) {
          nextFrame = m_consumerBuf[cid].getHead();
          assert(nextFrame != nullptr);
          nextFrame->frameNumber = frameNumber;
          nextFrame->frameSize = frameSize();
          nextFrame->cameraSerial = m_camera[i]->getSerialNumber();
          nextFrame->cameraNumber = i;

          nextFrame->imageBytes = bytes;
          m_consumerBuf[cid].advanceHead();
          ++frameNumber;
        }

        if (i == m_previewIndex) {
          previewFrame.DeepCopy(&frame[i]);
          m_cameraView.updatePreviewFrame(previewFrame.GetData(), previewFrame.GetDataSize(),
                                          previewFrame.GetBitsPerPixel());
          m_cameraView.update();
        }

      } catch (...) {
        cerr << "Error when grabbing a frame from the camera " << i << endl;
        stopMainCamera();
        stopOtherCameras();
        throw "Error grabbing a frame from the camera " + to_string(i);
      }
    }

    if (m_oneshot && m_recording) {
      m_oneshot = false;
      m_stopRecording = true;
    }
  }

  pthread_barrier_wait(&startBarrier);

  for (auto& cbuf : m_consumerBuf) {
    cbuf.done();
  }

  stopMainCamera();
  stopOtherCameras();

  return;
}

void CameraController::writeHeader(const int fd, const uint32_t id)
{
  vector<uint32_t, aligned_allocator<uint32_t, 4096>> vec(1024);

  vec[0] = 0xfaceb00c;
  vec[1] = (uint32_t)time(nullptr);
  vec[2] = id;
  vec[3] = m_consumerCount;
  vec[4] = m_width;
  vec[5] = m_height;
  vec[6] = uint32_t(m_bitsPerPixel);
  vec[7] = (m_camera.size() / m_consumerCount) + ((id < m_camera.size() % m_consumerCount) ? 1 : 0);

  int err = write(fd, &vec[0], vec.size() * sizeof(uint32_t));
  if (err == -1) {
    cerr << strerror(errno) << endl;
    throw string("Error writing header: ") + strerror(errno);
  }
}

void CameraController::cameraConsumer(const unsigned int id) {
  setThreadPriority(m_producerCount + id);

  auto rec = false;
  int fd;
  ssize_t total = 0;
  uint64_t nimg = 0;

  for (auto k = id; k < m_camera.size(); k += m_consumerCount) {
    if (k == m_masterCameraIndex)
      continue;

    m_camera[k]->toggleStrobeOut(m_pinTrigger, false);
  }

  pthread_barrier_wait(&startBarrier);

  while (m_keepRunning) {
    if (!rec && m_recording) {
      string fname = m_dirname[id] + "/" + to_string(id) + ".bin";
      fd = open(fname.c_str(), O_WRONLY | O_NONBLOCK | O_CREAT | O_DIRECT, 0644);
      if (fd < 0) {
        assert(!"Can't create the destination file");
      } else {
        rec = true;
      }

      writeHeader(fd, id);
      pthread_barrier_wait(&startBarrier);
    }

    if (rec && !m_recording) {
      close(fd);
      fd = -1;
      rec = false;
    }

    if (rec) {
      FramePacket* next = nullptr;
      while ((next = m_consumerBuf[id].getTail()) != nullptr) {
        auto ptr = reinterpret_cast<uint32_t*>(next->imageBytes);
        ptr[0] = next->frameSize;
        ptr[1] = next->cameraSerial;

        ssize_t count = write(fd, next->imageBytes, next->frameSize);
        if (count < 0) {
          close(fd);
          throw "error writing data to disk in consumer " + to_string(id);
        }

        total += count;
        ++nimg;
        m_consumerBuf[id].advanceTail();
      }
    }

    if (m_consumerBuf[id].isDone()) {
      m_consumerBuf[id].reset();
      cout << id << " buffers done" << endl;
      pthread_barrier_wait(&startBarrier);
    }
  }

  pthread_barrier_wait(&startBarrier);
}

void CameraController::setPreviewCamera(unsigned int k) {
  const unsigned int nCams = PointGreyCamera::findCameras();
  if (k < nCams) {
    m_previewIndex = k;
  }
}

void CameraController::startRecording(const bool oneshot) {
  if (!m_startRecording) {
    if (!oneshot) {
      m_startRecording = true;
    } else {
      m_oneshot = true;
    }
  }
}

void CameraController::stopRecording() {
  m_stopRecording = true;
}

void CameraController::setPaths(const vector<string>& paths) {
  copy(paths.cbegin(), paths.cend(), m_dirname.begin());
}

CameraController::~CameraController() {
  m_keepRunning = false;
}
