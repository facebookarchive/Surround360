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
#include "IMX.hpp"
#include "CMOSIS.hpp"

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
    m_whiteBalance("450 796"),
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
    producerCount(nproducers),
    consumerCount(nconsumers),
    update(4) {

  m_masterCamera = make_tuple(-1, -1);

  m_width = PointGreyCamera::getCamera(0)->frameWidth();
  m_height = PointGreyCamera::getCamera(0)->frameHeight();
  m_rawFrame = new uint8_t[m_width * m_height];

  m_oneshotIdx = make_unique<std::vector<int>[]>(consumerCount);

#if 0
  if (m_width == sizeof(IMX[0])/sizeof(IMX[0][0])) {
    for (size_t y = 0; y < m_height; ++y) {
      for (size_t x = 0; x < m_width; ++x) {
	if (IMX[y][x] == 1) {
	  m_sample.push_back(y * m_width + x);
	}
      }
    }
  } else {
    for (size_t y = 0; y < m_height; ++y) {
      for (size_t x = 0; x < m_width; ++x) {
	if (CMOSIS[y][x] == 1) {
	  m_sample.push_back(y * m_width + x);
	}
      }
    }
  }
#endif
}

CameraController& CameraController::get(
  CameraView& camview,
  const unsigned int nproducers,
  const unsigned int nconsumers) {

  if (globalController == nullptr) {
    globalController =
      shared_ptr<CameraController>(new CameraController(camview, nproducers, nconsumers));
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
  m_masterCamera = std::make_tuple(numeric_limits<int>::max(), -1);

  for (int k = 0; k < nCams; ++k) {
    PointGreyCameraPtr camera = PointGreyCamera::getCamera(k);
    auto key = std::get<0>(m_masterCamera);
    auto serial = camera->getSerialNumber();

    if (serial < key) {
      m_masterCamera = make_tuple(serial, k);
    }

    m_camera.push_back(camera);
  }

  int k = 0;
  for (auto& camera : m_camera) {
    try {
      bool master = (k == std::get<1>(m_masterCamera)) ? true : false;
      ++k;

      camera->powerCamera(true);
      if (-1 == camera->init(
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
    } catch (...) {
      camera->toggleStrobeOut(m_pinStrobe, false);
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
  m_consumerBuf = new ConsumerBuffer[count];

  for (int cid = 0; cid < count; ++cid) {
    const unsigned int kAlignment = 4096;
    const unsigned long nents = 1000u;
    const unsigned long long sz = frameSize() * nents;

    m_consThread.emplace_back(&CameraController::cameraConsumer, this, cid);
  }
}

void CameraController::cameraProducer(const unsigned int id) {
  const size_t camerasPerProducer = m_camera.size() / m_mutex.size();
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
      auto k = std::get<1>(m_masterCamera);
      auto m = 0;

      if (update[paramBits]) {
	m_camera[k]->stopCapture();
      }

      for (auto& cam : m_camera) {
	bool master = k == m;

	++m;
	if (master) {
	  continue;
	}

	cam->setCameraProps(
	  make_pair(m_exposure, false),
	  make_pair(m_brightness, false),
	  make_pair(m_gamma, false),
	  make_pair(m_framerate, update[paramFramerate]),
	  make_pair(m_shutter, update[paramShutter]),
	  make_pair(m_gain, false));

	if (update[paramBits]) {
	  cam->stopCapture();
	  cam->updatePixelFormat(m_bitsPerPixel);
	  cam->startCapture();
	}
      }

      m_camera[k]->setCameraProps(
	make_pair(m_exposure, false),
	make_pair(m_brightness, false),
	make_pair(m_gamma, false),
	make_pair(m_framerate, update[paramFramerate]),
	make_pair(m_shutter, update[paramShutter]),
	make_pair(m_gain, false));

      if (update[paramBits]) {
	m_camera[k]->updatePixelFormat(m_bitsPerPixel);
	m_camera[k]->startCapture();
	update[paramBits] = false;
      }
      m_paramUpdatePending = false;
    }

    vector<size_t> histogram(256);

    if (m_oneshot) {
      ofstream cameraNamesFile(m_dirname + "/cameranames.txt");
      for (auto& cam : m_camera) {
	cameraNamesFile << to_string(cam->getSerialNumber()) << "\n";
      }
      cameraNamesFile.close();

      for (auto i = cameraOffset; i < lastCamera; ++i) {
	void* bytes = m_camera[i]->getFrame();
	m_oneshotIdx[i % consumerCount].push_back(i);

	auto nextFrame = m_consumerBuf[i % consumerCount].getHead();
	nextFrame->frameNumber = frameNumber;
	nextFrame->cameraNumber = i;
	nextFrame->imageBytes = bytes;
	m_consumerBuf[i % consumerCount].advanceHead();
      }

      for (auto k = 0; k < consumerCount; ++k) {
	m_oneshotCount[k] = m_oneshotIdx[k].size();
      }

      m_oneshot = false;
    }

    for (auto i = cameraOffset; i < lastCamera; ++i) {
      const size_t sz = frameSize();
      const int cid = i % consumerCount; // ping-pong between output threads
      ++frameCount;

      if (i == 0 && m_startRecording) {
	m_startRecording = false;
	m_recording = true;

	ofstream cameraNamesFile(m_dirname + "/cameranames.txt");
	for (auto& cam : m_camera) {
	  cameraNamesFile << to_string(cam->getSerialNumber()) << "\n";
	}
	cameraNamesFile.close();
      }

      if (i == 0 && m_stopRecording) {
	m_stopRecording = false;
	m_recording = false;
	m_consumerBuf[0].done();
	m_consumerBuf[1].done();
      }

      // Retrieve an image from buffer
      FramePacket* nextFrame = nullptr;

      try {
	void* bytes = m_camera[i]->getFrame();
	// loop invariant; if this fails, it means the program is not written correctly
	assert(bytes != nullptr);

	if (m_recording) {
	  nextFrame = m_consumerBuf[cid].getHead();
	  nextFrame->frameNumber = frameNumber;
	  nextFrame->cameraNumber = i;
	  nextFrame->imageBytes = bytes;
	  m_consumerBuf[cid].advanceHead();
	  ++frameNumber;
	}

	for (auto idx : m_sample) {
	  auto p = (uint8_t*)bytes;
	  ++histogram[p[idx]];
	}

	if (i == m_previewIndex) {
	  m_cameraView.updatePreviewFrame(bytes);
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

void CameraController::cameraConsumer(const unsigned int id) {
  cpu_set_t threadCpuAffinity;
  CPU_ZERO(&threadCpuAffinity);
  CPU_SET(id, &threadCpuAffinity);

  sched_setaffinity(0, sizeof(threadCpuAffinity), &threadCpuAffinity);

  auto rec = false;
  int fd;
  ssize_t total = 0;
  unique_lock<mutex> lock(m_slaveMutex[id]);
  uint64_t nimg = 0;

  m_slaveCond[id].wait(lock);

  while (m_keepRunning) {
    if (m_oneshotCount[id] == m_oneshotIdx[id].size()) {
      string fname = m_dirname + "/" + to_string(id) + ".bin";
      fd = open(fname.c_str(), O_WRONLY | O_NONBLOCK | O_CREAT, 0644);
      if (fd < 0) {
	assert(!"Can't create the destination file");
      }

      for (auto& idx : m_oneshotIdx[id]) {
	auto next = m_consumerBuf[id].getTail();
	ssize_t count = write(fd, next->imageBytes, frameSize());
	if (count < 0) {
          throw "error writing data to disk in consumer " + to_string(id);
	}

	m_consumerBuf[id].advanceTail();
      }

      m_oneshotCount[id] = -1;
      m_oneshotIdx[id].clear();
    }

    if (!rec && m_recording) {
      string fname = m_dirname + "/" + to_string(id) + ".bin";
      fd = open(fname.c_str(), O_WRONLY | O_NONBLOCK | O_CREAT, 0644);
      if (fd < 0) {
	assert(!"Can't create the destination file");
      } else {
	rec = true;
      }
    }

    if (rec && !m_recording) {
      close(fd);
      fd = -1;
      rec = false;
    }

    if (rec) {
      FramePacket* next;
      while ((next = m_consumerBuf[id].getTail()) != nullptr) {
	ssize_t count = write(fd, next->imageBytes, frameSize());
	if (count < 0) {
	  assert(!"error writing data to disk");
	}

	total += count;
	++nimg;
	m_consumerBuf[id].advanceTail();
      }
    }
  }
}

void CameraController::startSlaveCapture() {
  auto k = 0;
  for (auto& cam : m_camera) {
    auto master = (k == std::get<1>(m_masterCamera));

    cam->toggleStrobeOut(1, false);

    if (!master) {
      cam->toggleStrobeOut(3, false);
      cam->startCapture();
    }

    ++k;
  }

  for (auto& cond : m_slaveCond) {
    cond.notify_one();
  }
}

void CameraController::startMasterCapture() {
  auto idx = std::get<1>(m_masterCamera);

  m_camera[idx]->toggleStrobeOut(m_pinStrobe, true);
  m_camera[idx]->startCapture();

  for (auto& cond : m_cond) {
    cond.notify_one();
  }
}

size_t CameraController::frameSize() const {
  return (m_width * m_height * m_bitsPerPixel) / 8;
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

void CameraController::setPath(const string& path) {
  m_dirname = path;
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
