/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */
#include <CameraControl.hpp>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <fcntl.h>
#include <getopt.h>
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

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/avfiltergraph.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavformat/avformat.h>
#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/mathematics.h>
#include <libavutil/opt.h>
#include <libavutil/timestamp.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

#include <flycapture/FlyCapture2.h>
#include <gflags/gflags.h>
#include <libusb-1.0/libusb.h>

using namespace std;
using namespace surround360;
using namespace fc;

static const int kNumPreviewCams = 4;
static const int kAlignment = 4096;
static const int kMaxDigits = 4;

#define D(x) if (FLAGS_debug) {cout << x << endl;}

DEFINE_bool(debug,          false,                    "Enable printing of debugging statements.");
DEFINE_string(dir,          to_string(time(nullptr)), "Directory to save data to.");
DEFINE_double(brightness,   10.449,                   "Set brightness value.");
DEFINE_double(exposure,     0.850,                    "Set exposure value.");
DEFINE_double(fps,          30.0,                     "Set frame rate.");
DEFINE_double(gain,         0.0,                      "Set gain.");
DEFINE_double(gamma,        1.250,                    "Set gamma.");
DEFINE_bool(list,           false,                    "List detected cameras.");
DEFINE_bool(mono,           false,                    "Capture grayscale frames.");
DEFINE_bool(nocapture,      false,                    "Configure cameras and exit without capturing anything.");
DEFINE_int32(master,        -1,                       "Master camera serial number.");
DEFINE_int32(numcams,       -1,                       "Number of cameras in the rig.");
DEFINE_int32(nbits,         8,                        "Set bit depth. Allowed values: 8, 12 or 16.");
DEFINE_int32(nframes,       0,                        "Set number of frames to capture.");
DEFINE_bool(props,          false,                    "Print camera property ranges, default values and exit.");
DEFINE_bool(raw,            true,                     "Capture RAW frames.");
DEFINE_bool(restore,        false,                    "Restore memory channels of all cameras.");
DEFINE_double(shutter,      20.0,                     "Set shutter speed.");
DEFINE_bool(stop,           false,                    "Stop capturing.");
DEFINE_string(whitebalance, "450 796",                "Set red, blue white balance values.");
DEFINE_bool(cli,            false,                    "Enable CLI mode");

typedef pair<unsigned int, unsigned int> SerialIndexPair;
typedef vector<SerialIndexPair> SerialIndexVector;
typedef SerialIndexVector::iterator SerialIndexIterator;

// The output disk prefix name. There is one per consumer thread
// (CONSUMER_COUNT).
static const string kFramesDisk = "/media/snoraid";

// Compute a time difference in seconds
static long double timeDiff(timespec start, timespec end) {
  const long double s = start.tv_sec + start.tv_nsec * 1.0e-9;
  const long double e = end.tv_sec   + end.tv_nsec   * 1.0e-9;
  return e - s;
}

// Needs to be global so the signal handler can set this variable.
static bool keepRunning = true;
static bool startRecording = false;
static bool stopRecording = false;
static bool recording = false;

// If we get a SIGINT (a ctrl-c) stop recording at the next frame
// boundary.
static void sigIntHandler(int signal) {
  keepRunning = false;
  stopRecording = true;
}

static unsigned int previewCameras[kNumPreviewCams] = { 0, 4, 8, 12 };

// Watch out when using this function! It exits on error by default

static void printAndSaveError(
  const string& errorText, const string& destDir) {
  // Print error
  cerr << errorText << endl;

  ofstream errorLogFile;
  ostringstream filename;
  filename << destDir << "/error.log";

  // Truncate error file. Apache will assume the contents of this file are
  // from the last capture
  errorLogFile.open(filename.str().c_str(), std::fstream::trunc);

  if (errorLogFile.is_open()) {
    errorLogFile << errorText << endl;
    errorLogFile.close();
  } else {
    cerr << "Warning: Unable to open file ("
         << filename.str() << ") for writing. "
         << "Please check permissions" << endl;
  }
}

static const string getOptString(PointGreyCamera::CameraProperty propType) {
  switch (propType) {
  case PointGreyCamera::CameraProperty::BRIGHTNESS: return "brightness";
  case PointGreyCamera::CameraProperty::GAMMA: return "gamma";
  case PointGreyCamera::CameraProperty::SHUTTER: return "shutter";
  case PointGreyCamera::CameraProperty::GAIN: return "gain";
  case PointGreyCamera::CameraProperty::WHITE_BALANCE: return "whitebalance";
  default:
    throw "Invalid option passed.";
  }
}

static const string getDefaultOptValue(PointGreyCamera::CameraProperty propType) {
  switch (propType) {
  case PointGreyCamera::CameraProperty::BRIGHTNESS: return "10.449";
  case PointGreyCamera::CameraProperty::GAMMA: return "1.250";
  case PointGreyCamera::CameraProperty::SHUTTER: return "20.0";
  case PointGreyCamera::CameraProperty::GAIN: return "0.0";
  case PointGreyCamera::CameraProperty::WHITE_BALANCE: return "450 796";
  default:
    throw "Invalid option passed.";
  }
}

static string getPropRange(
  PointGreyCameraPtr& pCam,
  PointGreyCamera::CameraProperty prop) {

  // Get property
  string propstr = pCam->getProperty(prop);
  ostringstream oss;

  oss << "*PROP*;"
      << getOptString(prop) << ";"
      << propstr << ";"
      << getDefaultOptValue(prop) << ";"
      << "-" << getOptString(prop)
      << endl;

  return oss.str();
}

static string getPropsRange(PointGreyCameraPtr& pCam) {
  string output;
  output += getPropRange(pCam, PointGreyCamera::CameraProperty::SHUTTER);
  output += getPropRange(pCam, PointGreyCamera::CameraProperty::GAIN);
  return output;
}

static void printAndSaveCameraProperties(PointGreyCameraPtr& pCam) {
  string camProps = getPropsRange(pCam);

  ofstream myfile;
  myfile.open("camProps_raw");

  if (myfile.is_open()) {
    myfile << camProps;
    myfile.close();
  } else {
    D("ERROR: Unable to open file for writing. Please check permissions");
    exit(EXIT_FAILURE);
  }

  D(camProps);
}

static void disconnect(
  PointGreyCameraPtr cameras[],
  int nCameras) {
  D("Disconnecting cameras...");

  for (unsigned int i = 0; i < nCameras; i++) {
    cameras[i]->detach();
    D("Camera " << i << " disconnected...");
  }
}

static void stopCapturing(
  PointGreyCameraPtr cameras[],
  int nCameras) {
  D("Stop capturing...");

  for (unsigned int i = 0; i < nCameras; i++) {
    cameras[i]->stopCapture();
    D("Camera " << i << " stopped...");
  }
}

static int isPreviewCam(int k) {
  for (int m = 0; m < kNumPreviewCams; ++m) {
    if (previewCameras[m] == k) {
      return m;
    }
  }
  return -1;
}

static int setPreviewCam(int k, int m) {
  previewCameras[k] = m;
}

void cameraProducer(
  ConsumerBuffer *consumerBuffer,
  ConsumerBuffer *previewBuffer,
  PointGreyCameraPtr ppCameras[],
  const unsigned int nCameras,
  const unsigned int nImages,
  const int pid,
  const int iCamMaster,
  const bool isDebug,
  const bool isMono,
  const bool isRaw,
  const int pinStrobe,
  const unsigned int nBits,
  unsigned int* droppedFramesCount,
  unsigned int* droppedFramesCur,
  unsigned int* droppedFramesPrev,
  stringstream *statsStream,
  const string& destDir) {

  // Create file to write heartbeats
  ostringstream filenameHeartbeat;
  filenameHeartbeat << destDir << "/heartbeat.dat";
  int fd = open(filenameHeartbeat.str().c_str(), O_WRONLY | O_CREAT, 0777);

  // Setup the SIGINT handler
  struct sigaction sa;
  sa.sa_handler = &sigIntHandler;

  // Restart the system call, if at all possible
  sa.sa_flags = SA_RESTART;

  // Block every signal during the handler
  sigfillset(&sa.sa_mask);

  // Intercept SIGINT
  if (sigaction(SIGINT, &sa, nullptr) == -1) {
    printAndSaveError("Error: cannot handle SIGHUP", destDir);
  }

  const int camerasPerProducer = nCameras / PRODUCER_COUNT;
  const int cameraOffset = pid * camerasPerProducer;
  const int lastCamera = min(cameraOffset + camerasPerProducer, int(nCameras));
  Image rawImage;

  int droppedFramesWindow[nCameras];
  for (int j = 0; j < nCameras; j++) {
    droppedFramesWindow[j] = 0;
  }

  // move outside of the region of CPUs where the kernel's MSI/MSI-X/IRQ handlers run
  cpu_set_t threadCpuAffinity;
  CPU_ZERO(&threadCpuAffinity);
  CPU_SET(10 + pid, &threadCpuAffinity);
  sched_setaffinity(0, sizeof(threadCpuAffinity), &threadCpuAffinity);

  sched_param sparam;
  sparam.sched_priority = 99; // real-timed
  sched_setscheduler(0, SCHED_RR, &sparam);

  timespec tStart, tEnd;
  long double tDiff;
  clock_gettime(CLOCK_REALTIME, &tStart);
  timespec tLast = tStart;

  int frameNumber = 0;
  int frameCount = 0;
  const int intFps = int(FLAGS_fps);

  while (keepRunning) {
    for (unsigned int i = cameraOffset; i < lastCamera; ++i) {
      const int cid = i % CONSUMER_COUNT; // ping-pong between output threads
      ++frameCount;

      if (i == 0 && startRecording) {
        startRecording = false;
        recording = true;
      }

      if (i == 0 && stopRecording) {
        stopRecording = false;
        recording = false;
      }

      // Retrieve an image from buffer
      FramePacket* nextFrame = nullptr;

      try {
        void *bytes = ppCameras[i]->getFrame();
        // loop invariant; if this fails, it means the program is not written correctly
        assert(bytes != nullptr);

        if (recording) {
          nextFrame = consumerBuffer[cid].getHead();
          nextFrame->frameNumber = frameNumber;
          nextFrame->cameraNumber = i;
          memcpy(nextFrame->imageBytes, bytes, FRAME_SIZE);

          // We're done with the head of the queue - move on...
          consumerBuffer[cid].advanceHead();
        }

        FramePacket *previewFrame = nullptr;

        int prevIdx = isPreviewCam(i);
        if (prevIdx > -1 && (frameNumber & 1)) {
          previewFrame = previewBuffer[prevIdx].getHead();
          assert(previewFrame != nullptr);
          previewFrame->imageBytes = (uint8_t*)bytes;
          //memcpy(previewFrame->imageBytes, bytes, FRAME_SIZE);
          previewFrame->frameNumber = frameNumber;
          previewFrame->cameraNumber = i;
          previewBuffer[prevIdx].advanceHead();
        }
      } catch (...) {
        cerr << "Error when grabbing a frame from the camera " << i << endl;
        stopCapturing(ppCameras, nCameras);
        ppCameras[iCamMaster]->toggleStrobeOut(pinStrobe, false);
        disconnect(ppCameras, nCameras);
        exit(EXIT_FAILURE);
      }

      droppedFramesCount[i] += (droppedFramesCur[i] == 0)
        ? 0 : ppCameras[i]->getDroppedFramesCounter() - droppedFramesCur[i] - 1;
      droppedFramesCur[i] = ppCameras[i]->getDroppedFramesCounter();
      droppedFramesPrev[i] = droppedFramesCount[i];

      ppCameras[i]->commitShutterSpeedUpdate();
      ppCameras[i]->commitGainUpdate();

      if (frameNumber > 0 && (frameNumber % intFps) == 0 && i == 0) {
        timespec tCurr;
        clock_gettime(CLOCK_REALTIME, &tCurr);
        int dpf = 0;

        for (int j = 0; j < nCameras; j++) {
          dpf += droppedFramesCount[j] - droppedFramesWindow[j];
          droppedFramesWindow[j] = droppedFramesCount[j];
        }

        // Update heartbeat
        // Multiply by 10 to keep precision. Will divide by 10 in website
        int fps = 10.0 * intFps / timeDiff(tLast, tCurr);

        lseek(fd, 0, SEEK_SET);
        const int secs = timeDiff(tStart, tCurr);
        int bytes = write(fd, &fps, sizeof(fps));
        bytes = write(fd, &secs, sizeof(secs));
        bytes = write(fd, &dpf, sizeof(dpf));

        if (frameNumber % (3 * intFps) == 0) {
          if (FLAGS_cli) {
            D("Press 'r' to start recording, 's' to stop recording, 'q' to quit.");
          }
          D("Elapsed time = " << timeDiff(tStart, tCurr)
            << " FPS = " << fps / 10.0
            << " frame # = " << frameNumber
            << " Frames captured = " << frameCount
            << " Dropped frames = " << dpf
            << " " << consumerBuffer[0].stateString()
            << " " << consumerBuffer[1].stateString());
        }
        tLast = tCurr;
      }
    }
    ++frameNumber;
  }

  for (int cid = 0; cid < CONSUMER_COUNT; ++cid) {
    consumerBuffer[cid].done();
  }

  for (int cid = 0; cid < kNumPreviewCams; ++cid) {
    previewBuffer[cid].done();
  }

  close(fd);

  clock_gettime(CLOCK_REALTIME, &tEnd);
  tDiff = timeDiff(tStart, tEnd);

  int nImagesTotal = frameNumber * nCameras;
  float frameSizeGB = (float) FRAME_SIZE / (1024 * 1024 * 1024);
  float sizeGB = (float) nImagesTotal * frameSizeGB;
  float speedTheory = (float) nCameras * 30 * 8 * frameSizeGB;
  *statsStream << "--- Producer " << pid << "---" << endl;
  *statsStream << "Data writen: " << sizeGB << " GB" << endl;
  *statsStream << "Images count: " << nImagesTotal << endl;
  *statsStream << "Images count/camera: " << nImagesTotal / nCameras << endl;
  *statsStream << "Elapsed time: " << tDiff << " s" << endl;
  *statsStream << "Producer speed: " << (8 * sizeGB / tDiff) << " Gb/s" << endl;
  *statsStream << "Producer speed (theory): " << speedTheory << " Gb/s" << endl;
}

static const char* filterGraphDesc(
  "nullsrc=size=512x512 [bg];"
  "[0:v] setpts=PTS-STARTPTS, scale=256x256 [A];"
  "[1:v] setpts=PTS-STARTPTS, scale=256x256 [B];"
  "[2:v] setpts=PTS-STARTPTS, scale=256x256 [C];"
  "[3:v] setpts=PTS-STARTPTS, scale=256x256 [D];"
  "[bg][A] overlay=shortest=1 [bgA];"
  "[bgA][B] overlay=shortest=1:x=256 [bgAB];"
  "[bgAB][C] overlay=shortest=1:y=256 [bgABC];"
  "[bgABC][D] overlay=shortest=1:x=256:y=256");

static void preview(ConsumerBuffer* previewBuffer) {
  AVFormatContext* formatCtx = nullptr;
  AVOutputFormat* fmt = nullptr;
  AVCodec* codec = nullptr;
  AVCodecContext* codecCtx = nullptr;
  AVFrame* frame[kNumPreviewCams];
  AVFrame* filtFrame = av_frame_alloc();
  AVStream* stream = nullptr;
  SwsContext* scaleCtx = nullptr;
  AVFilter* source[kNumPreviewCams] = { 0 };
  AVFilter* sink = nullptr;
  AVFilterInOut* outputs[kNumPreviewCams];
  AVFilterInOut* output = nullptr;
  AVFilterInOut* inputs = nullptr;
  AVFilterContext* sinkCtx = nullptr;
  AVFilterContext* srcCtx[kNumPreviewCams];
  AVFilterGraph* filterGraph = nullptr;
  int ret, i;
  const int kPreviewResWidth = 512;
  const int kPreviewResHeight = 512;
  const int kSubFrameResWidth = 256;
  const int kSubFrameResHeight = 256;
  const int kBitrate = 10000;
  int64_t next_pts = 0;
  char args[400];
  const char *kStreamUrl = "http://127.0.0.1:8090/preview.ffm";
  AVRational timeBase = (AVRational){ 1, static_cast<int>(FLAGS_fps) };
  cpu_set_t threadCpuAffinity;
  enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE };

  av_register_all();
  avfilter_register_all();
  avformat_network_init();

  // prepare stream and filtering pipeline
  avformat_alloc_output_context2(&formatCtx, nullptr, "ffm", kStreamUrl);
  if (formatCtx == nullptr) {
    throw "could not allocate output format context";
  }
  assert(formatCtx != nullptr);

  fmt = formatCtx->oformat;
  fmt->video_codec = AV_CODEC_ID_VP8;
  assert(fmt != nullptr);

  codec = avcodec_find_encoder(fmt->video_codec);
  if (codec == nullptr) {
    throw "could not find video encoder codec";
  }
  assert(codec != nullptr);

  stream = avformat_new_stream(formatCtx, codec);
  if (stream == nullptr) {
    throw "could not allocate new stream";
  }
  assert(stream != nullptr);

  stream->id = formatCtx->nb_streams - 1;
  codecCtx = avcodec_alloc_context3(codec);
  if (codecCtx == nullptr) {
    throw "could not allocate codec context";
  }
  assert(codecCtx != nullptr);

  codecCtx->codec_id = fmt->video_codec;
  codecCtx->bit_rate = kBitrate;
  codecCtx->width = kPreviewResWidth;
  codecCtx->height = kPreviewResHeight;
  stream->time_base = timeBase;
  codecCtx->time_base = stream->time_base;
  codecCtx->gop_size = 12;
  codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  codecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
  codecCtx->qmin = 10;
  codecCtx->qmax = 42;
  codecCtx->framerate = (AVRational){ 1, static_cast<int>(FLAGS_fps) };
  codecCtx->level = 2;
  codecCtx->profile = 1;
  codecCtx->thread_count = 4;
  codecCtx->thread_type = FF_THREAD_FRAME;

  if (fmt->flags & AVFMT_GLOBALHEADER)
    codecCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  ret = avcodec_open2(codecCtx, codec, nullptr);
  if (ret < 0) {
    throw "could not open video codec";
  }
  assert(ret == 0);

  avcodec_parameters_from_context(stream->codecpar, codecCtx);

  ret = avcodec_copy_context(stream->codec, codecCtx);

  if (ret < 0) {
    throw "could not copy stream parameters";
  }
  assert(ret == 0);

  av_dump_format(formatCtx, 0, kStreamUrl, 1);

  if (!(fmt->flags & AVFMT_NOFILE)) {
    ret = avio_open(&formatCtx->pb, kStreamUrl, AVIO_FLAG_WRITE);
    if (ret < 0) {
      throw "could not open format for streaming";
    }
    assert(ret == 0);
  }

  ret = avformat_write_header(formatCtx, nullptr);
  if (ret < 0) {
    throw "could not write header";
  }
  assert(ret == 0);

  sink = avfilter_get_by_name("buffersink");
  if (sink == nullptr) {
    throw "could not allocate buffersink filter";
  }
  assert(sink != nullptr);

  for (int i = 0; i < kNumPreviewCams; ++i) {
    source[i] = avfilter_get_by_name("buffer");
    if (source == nullptr) {
      throw "could not allocate buffer source filter";
    }
    assert(source[i] != nullptr);
  }

  filterGraph = avfilter_graph_alloc();
  if (filterGraph == nullptr) {
    throw "could not allocate filter graph";
  }
  assert(filterGraph != nullptr);

  snprintf(
    args,
    sizeof(args),
    "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
    kSubFrameResWidth,
    kSubFrameResHeight,
    AV_PIX_FMT_YUV420P,
    timeBase.num,
    timeBase.den,
    codecCtx->sample_aspect_ratio.num,
    codecCtx->sample_aspect_ratio.den);

  for (int i = 0; i < kNumPreviewCams; ++i) {
    char name[kMaxDigits];
    snprintf(name, sizeof(name), "%d:v", i);
    char *tmp = av_strdup(name);
    if (tmp == nullptr) {
      throw "could not allocate memory";
    }
    assert(tmp != nullptr);
    ret = avfilter_graph_create_filter(
      &srcCtx[i], source[i], tmp, args, nullptr, filterGraph);
    if (ret < 0) {
      throw "could not create graph filter";
    }
    assert(ret >= 0);
  }

  ret = avfilter_graph_create_filter(
    &sinkCtx, sink, "out", nullptr, nullptr, filterGraph);
  if (ret < 0) {
    throw "could not create graph filter";
  }
  assert(ret == 0);

  ret = av_opt_set_int_list(
    sinkCtx,
    "pix_fmts",
    pix_fmts,
    AV_PIX_FMT_NONE,
    AV_OPT_SEARCH_CHILDREN);
  if (ret < 0) {
    throw "failed to set sink options";
  }
  assert(ret == 0);

  for (int i = 0; i < kNumPreviewCams; ++i) {
    char name[kMaxDigits];
    snprintf(name, sizeof(name), "%d:v", i);

    outputs[i] = avfilter_inout_alloc();
    if (outputs[i] == nullptr) {
      throw "could not allocate output inout node";
    }
    assert(outputs[i] != nullptr);

    outputs[i]->name = av_strdup(name);
    outputs[i]->filter_ctx = srcCtx[i];
    outputs[i]->pad_idx = 0;
    outputs[i]->next = nullptr;
  }

  for (int i = 0; i < kNumPreviewCams - 1; ++i) {
    outputs[i]->next = outputs[i + 1];
  }
  output = outputs[0];

  inputs = avfilter_inout_alloc();
  if (inputs == nullptr) {
    throw "could not allocate input inout node";
  }
  assert(inputs != nullptr);

  inputs->name = av_strdup("out");
  inputs->filter_ctx = sinkCtx;
  inputs->pad_idx = 0;
  inputs->next = NULL;

  ret = avfilter_graph_parse_ptr(
    filterGraph, filterGraphDesc, &inputs, &output, nullptr);
  if (ret < 0) {
    throw "failed to parse graph filter";
  }
  assert(ret >= 0);

  ret = avfilter_graph_config(filterGraph, nullptr);
  if (ret < 0) {
    throw "failed to create filter complex";
  }
  assert(ret == 0);

  scaleCtx = sws_getContext(
    FRAME_W,
    FRAME_H,
    AV_PIX_FMT_BAYER_GBRG8,
    kSubFrameResWidth,
    kSubFrameResHeight,
    AV_PIX_FMT_YUV420P,
    SWS_FAST_BILINEAR,
    nullptr,
    nullptr,
    nullptr);
  if (scaleCtx == nullptr) {
    throw "failed to create scaling context";
  }
  assert(scaleCtx != nullptr);

  // allocate frames
  for (i = 0; i < kNumPreviewCams; ++i) {
    frame[i] = av_frame_alloc();
    if (frame[i] == nullptr) {
      throw "could not allocate frame";
    }
    assert(frame[i] != nullptr);

    frame[i]->format = AV_PIX_FMT_YUV420P;
    frame[i]->width = kSubFrameResWidth;
    frame[i]->height = kSubFrameResHeight;
    ret = av_frame_get_buffer(frame[i], 32);
    if (ret < 0) {
      throw "could not allocate frame";
    }
    assert(ret >= 0);
  }

  FramePacket* nextFrame = nullptr;

  while (keepRunning) {
    for (i = 0; i < kNumPreviewCams; ++i) {
      int stride[4] = { FRAME_W, 0, 0, 0 };

      nextFrame = previewBuffer[i].getTail();
      if (nextFrame == nullptr) {
        keepRunning = false;
        break;
      }

      const unsigned char* const planes[4] = {
        nextFrame->imageBytes, nullptr, nullptr, nullptr
      };

      frame[i]->pts = next_pts;
      ret = sws_scale(
        scaleCtx,
        planes,
        stride,
        0,
        FRAME_H,
        frame[i]->data,
        frame[i]->linesize);

      if (ret < 0) {
        throw "failed when scaling frame";
      }
      assert(ret >= 0);
      // frame is now down-scaled and converted from bayer to yuv420p
      // we can advance the buffer
      previewBuffer[i].advanceTail();

      ret = av_buffersrc_add_frame_flags(
        srcCtx[i], frame[i], AV_BUFFERSRC_FLAG_KEEP_REF);
      if (ret < 0) {
        throw "could not push scaled frame into a buffer source";
      }
      assert(ret == 0);
    }
    ++next_pts;

    while (keepRunning) {
      AVPacket pkt = { 0 };
      int gotPacket = 0;

      ret = av_buffersink_get_frame(sinkCtx, filtFrame);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        break;
      }
      if (ret < 0) {
        throw "error when retrieving filtered frames";
      }

      av_init_packet(&pkt);
      ret = avcodec_encode_video2(codecCtx, &pkt, filtFrame, &gotPacket);
      if (ret < 0) {
        throw "could not encode final frame";
      }
      assert(ret >= 0);

      if (gotPacket) {
        av_packet_rescale_ts(&pkt, codecCtx->time_base, stream->time_base);
        pkt.stream_index = stream->index;
        ret = av_interleaved_write_frame(formatCtx, &pkt);
        if (ret < 0) {
          throw "failed to write frame to the format output";
        }
        assert(ret >= 0);
      }

      av_frame_unref(filtFrame);
    }
  }
}

void frameConsumer(
  ConsumerBuffer consumerBuffer[],
  const int cid,
  const int nCameras,
  const int nImages,
  const string& dir,
  const string& label,
  stringstream* statsStream) {

  const string filenameFrames = dir + "/" + to_string(cid) + ".bin";
  int fd = open(filenameFrames.c_str(),
                O_WRONLY | O_NONBLOCK | O_CREAT | O_DIRECT, 0644);

  if (fd < 0) {
    printAndSaveError(strerror(errno), dir);
    exit(EXIT_FAILURE);
  }

  size_t fileSize = FRAME_SIZE * nCameras * size_t(nImages) / CONSUMER_COUNT;
  posix_fadvise(fd, 0, fileSize, POSIX_FADV_DONTNEED);
  posix_fadvise(fd, 0, fileSize, POSIX_FADV_SEQUENTIAL);

  int countImg = 0;
  size_t bytesWritten = 0;

  // move outside of the region of CPUs where the kernel's MSI/MSI-X/IRQ handlers run
  cpu_set_t threadCpuAffinity;
  CPU_ZERO(&threadCpuAffinity);
  CPU_SET(10 + cid + PRODUCER_COUNT, &threadCpuAffinity);
  sched_setaffinity(0, sizeof(threadCpuAffinity), &threadCpuAffinity);

  // Using UNIX open/write to avoid I/O buffering
  timespec tStart, tEnd;
  long double tDiff;
  clock_gettime(CLOCK_REALTIME, &tStart);

  FramePacket* nextFrame;
  while ((nextFrame = consumerBuffer[cid].getTail()) != nullptr) {
    int count = write(fd, nextFrame->imageBytes, FRAME_SIZE);
    if (count < 0) {
      printAndSaveError(strerror(errno), dir);
      exit(EXIT_FAILURE);
    }
    bytesWritten += count;
    countImg++;
    consumerBuffer[cid].advanceTail();
  }

  fsync(fd);
  close(fd);
  clock_gettime(CLOCK_REALTIME, &tEnd);

  tDiff = timeDiff(tStart, tEnd);

  float sizeGB = float(bytesWritten) / float(1024 * 1024 * 1024);
  *statsStream << "--- Consumer " << cid << "---" << endl;
  *statsStream << "Data writen: " << sizeGB << " GB" << endl;
  *statsStream << "Images count: " << countImg << endl;
  *statsStream << "Elapsed time: " << tDiff << " s" << endl;
  *statsStream << "Consumer speed: "
               << (8 * sizeGB / tDiff) << " Gb/s" << endl;
}

static bool hasEnoughDiskSpace(const string& path, const double requested) {
  struct statvfs stats;
  statvfs(path.c_str(), &stats);
  const double avail = double(stats.f_bfree) * double(stats.f_frsize);
  return avail > requested;
}

static void checkCameraSpeeds() {
  int err;
  ssize_t ndevs = 0;
  libusb_device** devices = nullptr;
  const unsigned short kVendorID = 0x1e10;
  const unsigned short kProductID = 0x3300;

  libusb_init(nullptr);

  ndevs = libusb_get_device_list(nullptr, &devices);
  for (int k = 0; k < ndevs; k++) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(devices[k], &desc);

    if ((desc.idVendor != kVendorID) && (desc.idProduct != kProductID)) {
      continue;
    }

    int speed = libusb_get_device_speed(devices[k]);
    if (speed < LIBUSB_HIGH_SPEED_OPERATION) {
      int err;
      libusb_device_handle *handle = nullptr;
      libusb_device_handle *parentHandle = nullptr;
      libusb_device *parentDev = nullptr;

      cout << "USB device " << k
           << " operating at wrong speed. Resetting camera..." << endl;

      parentDev = libusb_get_parent(devices[k]);
      if ((err = libusb_open(parentDev, &parentHandle)) != 0) {
        cerr << "Error grabbing the parent interface handle: " << err << endl;
        continue;
      }

      libusb_reset_device(parentHandle);
      libusb_close(parentHandle);
    }
  }

  libusb_free_device_list(devices, 1);
}

static void getCameraSerialNumbers(SerialIndexVector *v) {
  unsigned int ncameras = 0;
  BusManager busMgr;
  PGRGuid guid;
  CameraInfo camInfo;

  if (!v) {
    return;
  }

  ncameras = PointGreyCamera::findCameras();

  for (int k = 0; k < ncameras; ++k) {
    PointGreyCameraPtr c = PointGreyCamera::getCamera(k);
    v->push_back(make_pair(k, c->getSerialNumber()));
  }
}

static void validateWhiteBalance(const string& valuestr) {
  istringstream iss(valuestr);
  istream_iterator<string> end;
  istream_iterator<string> iter(iss);
  int idx = 0;
  int wb[2] = { 0 };

  for (; iter != end; ++iter, ++idx) {
    try {
      wb[idx] = stoi(*iter);
    } catch (...) {
      cerr << "white balance arguments need to be a pair of integers." << endl;
    }
  }
  if (idx != 2) {
    throw "whitebalance needs a pair of integer values.";
  }
}

static void saveCMDArgs(const string& destDir, int argc, char* argv[]) {
  ofstream cmdFile;
  ostringstream filename;
  filename << destDir << "/cmd.txt";
  cmdFile.open(filename.str().c_str());

  if (cmdFile.is_open()) {
    for (int i = 0; i <= argc; i++) {
      cmdFile << argv[i] << " ";
    }
    cmdFile.close();
  } else {
    cerr << "Warning: Unable to open file (cmd) for writing."
         << " Please check permissions, and make sure all intermediate "
         << "directories are created ("
         << destDir << ")"
         << endl;
  }
}

static void getPixelFormatFromBitDepth(
  PixelFormat* pf,
  unsigned int nBits) {
  switch (nBits) {
  case 8:
    *pf = PIXEL_FORMAT_RAW8;
    break;
  case 12:
    *pf = PIXEL_FORMAT_RAW12;
    break;
  case 16:
    *pf = PIXEL_FORMAT_RAW16;
    break;
  default:
    cerr << "Invalid bit depth! Falling back to RGB8" << endl;
    break;
  }
}

static void saveDroppedFrames(
  PointGreyCameraPtr& ppCam,
  unsigned int droppedFrames,
  const string& destDir) {
  ofstream skippedFramesFile;
  ostringstream filename;
  filename << destDir << "/dropped_frames.txt";
  skippedFramesFile.open(filename.str().c_str(), std::fstream::app);

  if (skippedFramesFile.is_open()) {
    skippedFramesFile << "Dropped frames in camera " << ppCam->getSerialNumber()
                      << ": " << droppedFrames << endl;
    skippedFramesFile.close();
  } else {
    cerr << "Warning: Unable to open file (dropped_frames) for writing. "
         << "Please check permissions"
         << endl;
  }
}

static int getControlQueue() {
  const char* kQueuePath = "/usr/local/bin/CameraControl";
  const int kKeyId = 83; // ascii 'S'
  key_t queueKey = ftok(kQueuePath, kKeyId);
  int queueId = msgget(queueKey, 0666 | IPC_CREAT);
  if (queueId == -1) {
    throw "can't create control message queue";
  }
  return queueId;
}

static termios origTermSettings;

static void restoreTerminalSettings() {
  tcsetattr(STDIN_FILENO, 0, &origTermSettings);
}

int main(int argc, char *argv[]) {
  PointGreyCameraPtr *ppCameras;
  Error error;
  Image rawImage;
  PixelFormat pf = PIXEL_FORMAT_RGB8;
  timespec tStart, tEnd, tDiff;
  termios tattr;

  const int pinStrobe = 2; // pin #3 (red wire)
  const int pinTrigger = 3; // pin #4 (green wire)
  // Time (ms) to wait for an image when calling RetrieveBuffer
  const int timeoutBuffer = -1;
  float sh; // Used to track if we do autoshutter (sh = 0)
  SerialIndexVector* serialNumbers = new SerialIndexVector();

  // Serial number of camera that will trigger the rest (master)
  int camStrobeOutSN = -1;
  int iCamMaster = -1;
  int ret;
  unsigned int nCameras = 0;
  string finalPath;
  struct stat dirStat;
  const mode_t kPermissions =
    S_IRUSR |
    S_IWUSR |
    S_IXUSR |
    S_IRGRP |
    S_IWGRP |
    S_IXGRP |
    S_IROTH |
    S_IXOTH;

  google::SetUsageMessage("Control capturing of image data.");
  google::ParseCommandLineFlags(&argc, &argv, false);

  if (argc == 1) {
    cerr << "No arguments. See --help for more information." << endl;
    return -1;
  }

  if (FLAGS_cli) {
    tcgetattr(STDIN_FILENO, &origTermSettings);
    tattr = origTermSettings;
    tattr.c_lflag &= ~(ECHO | ICANON);
    tattr.c_cc[VMIN] = 1;
    tattr.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, 0, &tattr);

    atexit(&restoreTerminalSettings);
  }

  getCameraSerialNumbers(serialNumbers);
  SerialIndexVector* sortedSerials =
    new SerialIndexVector(serialNumbers->size());
  copy(serialNumbers->begin(), serialNumbers->end(), sortedSerials->begin());

  sort(sortedSerials->begin(), sortedSerials->end(),
    [](const SerialIndexPair& left, const SerialIndexPair& right)
    { return left.second < right.second; });

  // list cameras and quit
  if (FLAGS_list) {
    if (serialNumbers->empty()) {
      cerr << "No cameras detected." << endl;
      return -1;
    }

    cout << "1: " << sortedSerials->at(0).second << " [master]" << endl;

    int i;
    SerialIndexVector::iterator it;

    for (i = 2, it = ++sortedSerials->begin(); it != sortedSerials->end(); ++it, ++i) {
      cout << i << ": " << (*it).second << " [slave]" << endl;
    }

    delete sortedSerials;
    return 0;
  }

  const string label = FLAGS_dir;
  string captureDir = kFramesDisk + "/" + FLAGS_dir;

  ret = stat(captureDir.c_str(), &dirStat);
  if (ret == -1 && errno == ENOENT) {
    ret = mkdir(captureDir.c_str(), kPermissions);
    if (ret == -1) {
      const string kErrString(strerror(errno));
      throw "Can't create destination directory: " + kErrString;
    }
  } else {
    // destination directory exists. Don't overwrite. Append timestamp to the name
    captureDir += "-" + to_string(time(nullptr));
    ret = mkdir(captureDir.c_str(), kPermissions);
    if (ret == -1) {
      const string kErrString(strerror(errno));
      throw "Can't create destination directory "
        + captureDir + ": " + kErrString;
    }
  }

  saveCMDArgs(captureDir, argc, argv);
  checkCameraSpeeds();

  camStrobeOutSN = FLAGS_master;
  validateWhiteBalance(FLAGS_whitebalance);

  // Check if we have enough disk space
  if ((FLAGS_nframes > 0)
      && !hasEnoughDiskSpace(
        kFramesDisk, FRAME_SIZE * FLAGS_numcams * FLAGS_nframes)) {
    cerr << "Not enough disk space to capture requested number of frames."
         << endl << "You need at least "
         << (FRAME_SIZE >> 20) * FLAGS_numcams * FLAGS_nframes
         << " MB available on " << kFramesDisk << endl;
    exit(EXIT_FAILURE);
  }

  // Create the producer/consumer buffer object
  ConsumerBuffer* consumerBuffer = new ConsumerBuffer[CONSUMER_COUNT];
  uint8_t* frameBytes[CONSUMER_COUNT];
  for (int k = 0; k < CONSUMER_COUNT; k++) {
    int ret = posix_memalign(
      (void **)&frameBytes[k],
      kAlignment,
      FRAME_SIZE * BUFFER_SIZE);
    assert(ret == 0);
    mlock(frameBytes[k], FRAME_SIZE * BUFFER_SIZE);
    memset(frameBytes[k], 0, FRAME_SIZE * BUFFER_SIZE);
    madvise(frameBytes, FRAME_SIZE * BUFFER_SIZE, MADV_SEQUENTIAL);
    consumerBuffer[k].setBuffers(frameBytes[k], FRAME_SIZE);
  }

  ConsumerBuffer* previewBuffer = new ConsumerBuffer[kNumPreviewCams];

  D("Starting process...");
  BusManager busMgr;

  D("Getting number of cameras...");

  nCameras = PointGreyCamera::findCameras();

  D("Number of cameras detected: " << nCameras);
  if (nCameras == 0 || (!FLAGS_props && FLAGS_numcams != nCameras)) {
    cerr << "Error: detected number of cameras "
         << "is different than passed in as argument." << endl;
    exit(EXIT_FAILURE);
  }

  // Check if we only want camera property info
  if (FLAGS_props) {
    PointGreyCameraPtr camera = PointGreyCamera::getCamera(0);
    try {
      camera->init(false);
    } catch (...) {
      printAndSaveError("Erorr polling for trigger ready!", captureDir);
      camera->toggleStrobeOut(pinStrobe, false);
      camera->detach();
      exit(EXIT_FAILURE);
    }
  }

  ppCameras = new PointGreyCameraPtr[nCameras];

  if (FLAGS_restore) {
    for (unsigned int i = 0; i < nCameras; i++) {
      ppCameras[i] = PointGreyCamera::getCamera(i);
      ppCameras[i]->reset();
    }
    disconnect(ppCameras, nCameras);
    exit(EXIT_SUCCESS);
  }

  if (camStrobeOutSN == -1) {
    camStrobeOutSN = sortedSerials->at(0).second;
    printAndSaveError("Master serial number missing! Using "
                      + to_string(camStrobeOutSN), captureDir);
  }

  if (FLAGS_stop) {
    CameraInfo camInfo;

    for (unsigned int i = 0; i < nCameras; i++) {
      PointGreyCameraPtr cam = PointGreyCamera::getCamera(i);
      if (cam->getSerialNumber() == camStrobeOutSN) {
        cam->toggleStrobeOut(pinStrobe, false);
      }
    }

    stopCapturing(ppCameras, nCameras);
    disconnect(ppCameras, nCameras);
    exit(EXIT_SUCCESS);
  }

  sh = FLAGS_shutter;

  if (FLAGS_nframes == -1) {
    printAndSaveError("Number of frames (n) missing!", captureDir);
    exit(EXIT_FAILURE);
  }

  // Check that we have all we need if video
  if (FLAGS_nframes > 1) {
    float fps = FLAGS_fps;
    // We cannot have shutter time longer than 1/fps
    if (sh > 1000 / fps) {
      ostringstream oss;

      printAndSaveError(
        "Shutter time (" + to_string(sh)
        + "ms) cannot be longer than 1/fps ("
        + to_string(1000.0 / fps) + ")!", captureDir);
      exit(EXIT_FAILURE);
    }
  }

  if (FLAGS_raw) {
    getPixelFormatFromBitDepth(&pf, FLAGS_nbits);
  } else if (FLAGS_mono) {
    pf = PIXEL_FORMAT_MONO8;
  }

  CameraInfo camInfo;

  ////////// START OF SETTING CAMERA PARAMETERS //////////

  // Connect to all detected cameras and start capturing (i.e. letting sensor get light != buffering)
  for (unsigned int i = 0; i < nCameras; i++) {
    D("Connecting camera " << i << "...");
    ppCameras[i] = PointGreyCamera::getCamera(i);

    // Power on the camera
    D("Powering on camera " << i << "...");
    ppCameras[i]->powerCamera(true);

    D("Getting camera " << i << " info...");

    // Print camera information
    cout << ppCameras[i] << endl;

    // We will send a software trigger to one camera, which will be configured
    // to send a strobe output to the rest of the cameras at the time of icoming
    // trigger (since they are all in external trigger mode)
    // free run)
    bool isMaster = ppCameras[i]->getSerialNumber() == camStrobeOutSN;
    if (isMaster)
      iCamMaster = i;

    ppCameras[i]->setMaster();

    // Shutter = 0 means auto shutter, which requires the camera to be in
    // free-run mode
    if (sh == 0.0f)
      isMaster = true;

    try {
      ppCameras[i]->init(isMaster);
    } catch (...) {
      printAndSaveError("Erorr polling for trigger ready!", captureDir);
      ppCameras[iCamMaster]->toggleStrobeOut(pinStrobe, false);
      disconnect(ppCameras, nCameras);
      exit(EXIT_FAILURE);
    }
  }

  ////////// END OF SETTING CAMERA PARAMETERS //////////

  if (FLAGS_nocapture) {
    ppCameras[iCamMaster]->toggleStrobeOut(pinStrobe, false);
    disconnect(ppCameras, nCameras);
    cerr << "Done." << endl;
    exit(EXIT_SUCCESS);
  }

  if (iCamMaster == -1) {
    printAndSaveError("No master camera!! Exiting...", captureDir);
    ppCameras[iCamMaster]->toggleStrobeOut(pinStrobe, false);
    disconnect(ppCameras, nCameras);
    exit(EXIT_FAILURE);
  }

  // Prepare arrays of dropped frame counts
  unsigned int droppedFramesCount[nCameras];
  unsigned int droppedFramesCur[nCameras];
  unsigned int droppedFramesPrev[nCameras];

  for (unsigned int i = 0; i < nCameras; i++) {
    droppedFramesCount[i] = 0;
    droppedFramesCur[i] = 0;
    droppedFramesPrev[i] = 0;
  }

  // Dump the camera names
  ofstream cameraNamesFile(
    string(captureDir + "/cameranames.txt"));

  for (unsigned int i = 0; i < nCameras; i++) {
    cameraNamesFile << to_string(serialNumbers->at(i).second) << "\n";
  }
  cameraNamesFile.close();

  // Write stats to file
  ofstream statsFile;
  const string filenameStats = captureDir + "/stats.txt";
  statsFile.open(filenameStats.c_str(), std::fstream::app);


  // Start capture on all slave cameras
  for (unsigned int i = 0; i < nCameras; i++) {
    if (i != iCamMaster) {
      D("Starting slave camera " << i << " capture...");

      // Start capturing images
      PointGreyCamera::printError(ppCameras[i]->startCapture());
    }
  }

  // Start capture on master camera
  D("Starting master camera " << iCamMaster << " capture...");
  PointGreyCamera::printError(ppCameras[iCamMaster]->startCapture());

  // If we do it before calling StartCapture cameras will start sending strobe
  // pulses to before we want them to
  D("Setting camera strobes...");

  // Sometimes pin #1 is enabled for strobe out. Disable it.
  for (unsigned int i = 0; i < nCameras; i++) {
    ppCameras[i]->toggleStrobeOut(1, false);
  }

  // Set master camera strobe. This will send the pulses that slaves are expecting
  ppCameras[iCamMaster]->toggleStrobeOut(pinStrobe, true);

  if (sh == 0.0f) {
    D("Sleeping 2 seconds to let auto shutter stabilize...");
    sleep(2);
  }

  D("Grabbing " << FLAGS_nframes << " image(s) from each camera");

  ////////// START OF FRAME CAPTURE //////////
  clock_gettime(CLOCK_REALTIME, &tStart);

  // create preview threads
  thread* previewThread = new std::thread(preview, previewBuffer);

  // Create producer thread
  stringstream* producerStatsString[PRODUCER_COUNT];
  thread* cameraProducerThread[PRODUCER_COUNT];

  for (int pid = 0; pid < PRODUCER_COUNT; ++pid) {
    producerStatsString[pid] = new stringstream;
    cameraProducerThread[pid] =
      new std::thread(
        cameraProducer,
        consumerBuffer,
        previewBuffer,
        ppCameras,
        FLAGS_numcams,
        FLAGS_nframes,
        pid,
        iCamMaster,
        FLAGS_debug,
        FLAGS_mono,
        FLAGS_raw,
        pinStrobe,
        FLAGS_nbits,
        &droppedFramesCount[pid],
        &droppedFramesCur[pid],
        &droppedFramesPrev[pid],
        producerStatsString[pid],
        captureDir);
  }

  // Create consumer threads
  thread* frameConsumerThread[CONSUMER_COUNT];
  stringstream* consumerStatsString[CONSUMER_COUNT];
  int fp[CONSUMER_COUNT];

  for (int cid = 0; cid < CONSUMER_COUNT; ++cid) {
    consumerStatsString[cid] = new stringstream;

    frameConsumerThread[cid] =
      new std::thread(
        frameConsumer,
        consumerBuffer,
        cid,
        FLAGS_numcams,
        FLAGS_nframes,
        captureDir,
        label,
        consumerStatsString[cid]);
  }

  int queueId = getControlQueue();

  while (keepRunning) {
    static const int kCmdSize = 60;

    struct cmd_msgbuf {
      long mtype;
      char cmd[kCmdSize];
    } msg;

    if (FLAGS_cli) {
      char c;
      cin >> c;

      if (c == 'r') {
        startRecording = true;
        stopRecording = false;
      } else if (c == 's') {
        stopRecording = true;
        startRecording = false;
      } else if (c == 'q') {
        stopRecording = true;
        startRecording = false;
        keepRunning = false;
      }
    } else {
      ssize_t size = msgrcv(queueId, &msg, sizeof(msg.cmd), 0, IPC_NOWAIT);
      if (size > 0) {
        if (strncmp(msg.cmd, "cam", min(strlen("cam"), sizeof(msg.cmd))) == 0) {
          int k = -1;
          int m = -1;
          sscanf(msg.cmd, "cam %d %d", &k, &m);
          cout << "command was: " << msg.cmd << endl;
          if (k >= 0 && k < kNumPreviewCams) {
            // set preview slot K to show camera M
            setPreviewCam(k, sortedSerials->at(m).first);
          }
        } else if (strncmp(msg.cmd, "shutter", min(strlen("shutter"), sizeof(msg.cmd))) == 0) {
          double shutter = 0.0;
          sscanf(msg.cmd, "shutter %lf", &shutter);
          // set new shutter value
          for (int k = 0; k < FLAGS_numcams; ++k) {
            ppCameras[k]->prepareShutterSpeedUpdate(shutter);
          }
        } else if (strncmp(msg.cmd, "gain", strlen("gain")) == 0) {
          double gain = 0.0;
          sscanf(msg.cmd, "gain %lf", &gain);
          for (int k = 0; k < FLAGS_numcams; ++k) {
            ppCameras[k]->prepareGainUpdate(gain);
          }
        } else if (strncmp(msg.cmd, "record", strlen("record")) == 0) {
          startRecording = true;
          stopRecording = false;
        } else if (strncmp(msg.cmd, "stop", strlen("stop")) == 0) {
          stopRecording = true;
          startRecording = false;
        } else if (strncmp(msg.cmd, "quit", strlen("quit")) == 0) {
          stopRecording = true;
          startRecording = false;
          keepRunning = false;
        }
      }
    }
  }

  // Join frame producer threads
  for (int pid = 0; pid < PRODUCER_COUNT; ++pid) {
    cameraProducerThread[pid]->join();
    statsFile << producerStatsString[pid]->str();
    delete producerStatsString[pid];
  }

  D("Done");

  // Join frame consumer threads
  for (int cid = 0; cid < CONSUMER_COUNT; ++cid) {
    frameConsumerThread[cid]->join();
    statsFile << consumerStatsString[cid]->str();
    delete consumerStatsString[cid];
  }

  clock_gettime(CLOCK_REALTIME, &tEnd);

  delete [] consumerBuffer;
  delete serialNumbers;
  ////////// END OF FRAME CAPTURE //////////

  // Close files
  statsFile.close();
  stopCapturing(ppCameras, nCameras);

  for (unsigned int i = 0; i < nCameras; i++) {
    D("Camera " << ppCameras[i]->getSerialNumber()
      << " dropped " << droppedFramesCount[i] << " frame(s).");
    saveDroppedFrames(ppCameras[i], droppedFramesCount[i], captureDir);
  }

  // KP: Master camera will continue to send strobe pulses after stop capturing,
  // so we manually disable it
  ppCameras[iCamMaster]->toggleStrobeOut(pinStrobe, false);
  disconnect(ppCameras, nCameras);

  D("Done capturing");
  exit(EXIT_SUCCESS);
}

//////////////////////// END OF MAIN ////////////////////////
