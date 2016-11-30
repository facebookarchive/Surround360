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

#include "Config.hpp"

using namespace surround360;
using namespace std;

static shared_ptr<CameraController> globalController;

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
    m_mutex(nproducers),
    m_slaveMutex(nconsumers),
    m_cond(nproducers),
    m_slaveCond(nconsumers),
    m_cameraView(camview),
    m_previewIndex(-1),
    m_paramUpdatePending(false),
    m_running(false),
    m_oneshot(false),
    m_oneshotCount(nconsumers, -1),
    m_producerCount(nproducers),
    m_consumerCount(nconsumers),
    m_consumerBuf(nconsumers),
    update(4) {

  m_width = PointGreyCamera::getCamera(0)->frameWidth();
  m_height = PointGreyCamera::getCamera(0)->frameHeight();

  m_oneshotIdx = make_unique<std::vector<int>[]>(m_consumerCount);
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

bool CameraController::configureCameras(
  const float shutter,
  const float framerate,
  const float gain,
  const int bitsPerPixel) {

  // Time (ms) to wait for an image when calling RetrieveBuffer
  const int timeoutBuffer = -1;

  m_shutter = shutter;
  m_framerate = framerate;
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
  const float gain,
  const int bits) {

  if (shutter != m_shutter) {
    m_shutter = shutter;
    update[paramShutter] = true;
    m_paramUpdatePending = true;
  }

  if (fps != m_framerate) {
    m_framerate = fps;
    update[paramFramerate] = true;
    m_paramUpdatePending = true;
  }

  if (gain != m_gain) {
    m_gain = gain;
    update[paramGain] = true;
    m_paramUpdatePending = true;
  }

  if (bits != m_bitsPerPixel) {
    m_bitsPerPixel = bits;
    update[paramBits] = true;
    m_paramUpdatePending = true;
  }
}


void CameraController::startProducer(const unsigned int count) {
  for (int pid = 0; pid < count; ++pid) {
    m_prodThread.emplace_back(&CameraController::cameraProducer, this, pid);
  }
}

void CameraController::startConsumers(const unsigned int count) {
  for (int cid = 0; cid < count; ++cid) {
    m_consThread.emplace_back(&CameraController::cameraConsumer, this, cid);
  }
}

void CameraController::cameraProducer(const unsigned int id) {
  const size_t camerasPerProducer = m_camera.size() / m_producerCount;
  const size_t cameraOffset = id * camerasPerProducer;
  const int lastCamera = std::min(cameraOffset + camerasPerProducer, m_camera.size());
  unsigned int frameCount = 0;
  unsigned int frameNumber = 0;

  cpu_set_t threadCpuAffinity;
  CPU_ZERO(&threadCpuAffinity);
  CPU_SET(id, &threadCpuAffinity);
  sched_setaffinity(0, sizeof(threadCpuAffinity), &threadCpuAffinity);

  sched_param sparam;
  sparam.sched_priority = 99;
  sched_setscheduler(0, SCHED_RR, &sparam);

  unique_lock<mutex> lock(m_mutex[id]);
  m_cond[id].wait(lock);

  m_running = true;

  while (m_keepRunning) {
    if (m_paramUpdatePending) {
      if (update[paramBits]) {
        // if switching pixel formats, stop master camera first so it doesn't trigger slaves
        m_camera[m_masterCameraIndex]->stopCapture();
      }

      // update all cameras except the master camera
      for (int k = 0; k < m_camera.size(); ++k) {
        if (k == m_masterCameraIndex) {
          continue;
        }

        m_camera[k]->setCameraProps(
          make_pair(m_exposure, false),
          make_pair(m_brightness, false),
          make_pair(m_gamma, false),
          make_pair(m_framerate, update[paramFramerate]),
          make_pair(m_shutter, update[paramShutter]),
          make_pair(m_gain, true));

        if (update[paramBits]) {
          m_camera[k]->stopCapture();
          m_camera[k]->updatePixelFormat(m_bitsPerPixel);
          m_camera[k]->startCapture();
        }
      }

      // update master camera last
      m_camera[m_masterCameraIndex]->setCameraProps(
        make_pair(m_exposure, false),
        make_pair(m_brightness, false),
        make_pair(m_gamma, false),
        make_pair(m_framerate, update[paramFramerate]),
        make_pair(m_shutter, update[paramShutter]),
        make_pair(m_gain, true));

      if (update[paramBits]) {
        // if we're updating pixel formats, resume triggering of the master cam after switch
        m_camera[m_masterCameraIndex]->updatePixelFormat(m_bitsPerPixel);
        m_camera[m_masterCameraIndex]->startCapture();
        update[paramBits] = false;
      }
      m_paramUpdatePending = false;
    }

    // single shot frame grab
    if (m_oneshot) {
      writeCameraNames(m_dirname[id] + "/cameranames.txt");

      // in one shot mode, we publish 1 frame from each camera and stop recording
      for (auto i = cameraOffset; i < lastCamera; ++i) {
        void* bytes = m_camera[i]->getFrame();

        // populate indices of cameras that the single frame is coming from
        // in camera iteration order (this will be the disk order)
        m_oneshotIdx[i % m_consumerCount].emplace_back(i);

        auto nextFrame = m_consumerBuf[i % m_consumerCount].getHead();
        assert(nextFrame != nullptr);
        nextFrame->frameNumber = frameNumber;
        nextFrame->frameSize = frameSize();
        nextFrame->cameraNumber = i;
        nextFrame->cameraSerial = m_camera[i]->getSerialNumber();
        memcpy(nextFrame->imageBytes, bytes, frameSize());
        m_consumerBuf[i % m_consumerCount].advanceHead();
      }

      for (auto k = 0; k < m_consumerCount; ++k) {
        // load oneshot control counter
        m_oneshotCount[k] = m_oneshotIdx[k].size();
      }

      // unset producer one-shot flag so we don't re-enter on next iteration of grab loop
      m_oneshot = false;
    }

    // main grab loop
    const size_t sz = frameSize();

    for (auto i = cameraOffset; i < lastCamera; ++i, ++frameCount) {
      const int cid = i % m_consumerCount; // ping-pong between output threads

      if (i == 0 && m_startRecording) {
        m_startRecording = false;
        m_recording = true;
        writeCameraNames(m_dirname[id] + "/cameranames.txt");
      }

      if (i == 0 && m_stopRecording) {
        m_stopRecording = false;
        m_recording = false;
        for (auto& cbuf : m_consumerBuf) {
          cbuf.done();
        }
      }

      // Retrieve an image from buffer
      FramePacket* nextFrame = nullptr;

      try {
        void* bytes = m_camera[i]->getFrame();
        // loop invariant; if this fails, it means the program is not written correctly
        assert(bytes != nullptr);

        if (m_recording) {
          // if recording, copy the frame to the buffer
          nextFrame = m_consumerBuf[cid].getHead();
          assert(nextFrame != nullptr);
          nextFrame->frameNumber = frameNumber;
          nextFrame->frameSize = frameSize();
          nextFrame->cameraSerial = m_camera[i]->getSerialNumber();
          nextFrame->cameraNumber = i;
          memcpy(nextFrame->imageBytes, bytes, frameSize());
          m_consumerBuf[cid].advanceHead();
          ++frameNumber;
        }

        if (i == m_previewIndex) {
          if (!m_recording) {
            auto start = reinterpret_cast<uint8_t*>(bytes);
            auto end = start + frameSize();
            auto ptr = make_unique<vector<uint8_t>>(start, end);
            m_cameraView.updatePreviewFrame(&(*ptr)[0]);
          } else {
            m_cameraView.updatePreviewFrame(nextFrame->imageBytes);
          }
          m_cameraView.update();
        }
      } catch (...) {
        cerr << "Error when grabbing a frame from the camera " << i << endl;

        for (auto& cam : m_camera) {
          cam->stopCapture();
          cam->toggleStrobeOut(m_pinStrobe, false);
          cam->detach();
        }
        throw "Error grabbing a frame from one of the cameras.";
      }
    }
  }
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
  cpu_set_t threadCpuAffinity;
  CPU_ZERO(&threadCpuAffinity);
  CPU_SET(m_producerCount + id, &threadCpuAffinity);

  sched_setaffinity(0, sizeof(threadCpuAffinity), &threadCpuAffinity);

  auto rec = false;
  int fd;
  ssize_t total = 0;
  unique_lock<mutex> lock(m_slaveMutex[id]);
  uint64_t nimg = 0;

  m_slaveCond[id].wait(lock);

  while (m_keepRunning) {
    // for one shot count only, we read pre-specified number of frames,
    // based on indices from m_oneshotIdx vector and save them to disk
    if (m_oneshotCount[id] == m_oneshotIdx[id].size()) {
      string fname = m_dirname[id] + "/" + to_string(id) + ".bin";
      fd = open(fname.c_str(), O_WRONLY | O_NONBLOCK | O_CREAT | O_DIRECT, 0644);
      if (fd < 0) {
        assert(!"Can't create the destination file");
      }

      writeHeader(fd, id);

      for (auto& idx : m_oneshotIdx[id]) {
        auto next = m_consumerBuf[id].getTail();
        uint32_t* ptr = reinterpret_cast<uint32_t*>(next->imageBytes);
        ptr[0] = next->frameSize;
        ptr[1] = next->cameraSerial;

        ssize_t count = write(fd, next->imageBytes, next->frameSize);
        if (count < 0) {
          throw "error writing data to disk in consumer " + to_string(id);
        }

        m_consumerBuf[id].advanceTail();
      }

      m_oneshotCount[id] = -1;
      m_oneshotIdx[id].clear();
    }

    if (!rec && m_recording) {
      string fname = m_dirname[id] + "/" + to_string(id) + ".bin";
      fd = open(fname.c_str(), O_WRONLY | O_NONBLOCK | O_CREAT | O_DIRECT, 0644);
      if (fd < 0) {
        assert(!"Can't create the destination file");
      } else {
        rec = true;
      }

      writeHeader(fd, id);
    }

    if (rec && !m_recording) {
      close(fd);
      fd = -1;
      rec = false;
    }

    if (rec) {
      FramePacket* next;
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
  }
}

void CameraController::startSlaveCapture() {
  for (auto k = 0; k < m_camera.size(); ++k) {
    m_camera[k]->toggleStrobeOut(1, false);

    if (k == m_masterCameraIndex) {
      continue;
    }

    m_camera[k]->toggleStrobeOut(3, false);
    m_camera[k]->startCapture();
  }

  for (auto& cond : m_slaveCond) {
    cond.notify_one();
  }
}

void CameraController::startMasterCapture() {
  m_camera[m_masterCameraIndex]->toggleStrobeOut(m_pinStrobe, true);
  m_camera[m_masterCameraIndex]->startCapture();

  for (auto& cond : m_cond) {
    cond.notify_one();
  }
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

void CameraController::setPaths(const string path[2]) {
  m_dirname[0] = path[0];
  m_dirname[1] = path[1];
}

CameraController::~CameraController() {
  m_keepRunning = false;

  for (auto& th : m_prodThread) {
    th.join();
  }

  for (auto& th : m_consThread) {
    th.join();
  }

  for (auto& cam : m_camera) {
    cam->stopCapture();
    cam->detach();
  }
}

void CameraController::writeCameraNames(const string& path) {
  ofstream cameraNamesFile(path);
  for (auto& cam : m_camera) {
    cameraNamesFile << to_string(cam->getSerialNumber()) << "\n";
  }
  cameraNamesFile.close();
}
