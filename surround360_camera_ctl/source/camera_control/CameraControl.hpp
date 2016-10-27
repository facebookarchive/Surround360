/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */

#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "PointGrey.hpp"
#include "ProducerConsumer.h"

#define PRODUCER_COUNT 1
#define CONSUMER_COUNT 2
#define FRAME_H        2048ULL
#define FRAME_W        2048ULL
#define FRAME_SIZE     (FRAME_H * FRAME_W)
#define BUFFER_SIZE    1000ULL

namespace surround360 {
  namespace fc = FlyCapture2;

  struct FramePacket{
    int frameNumber;
    int cameraNumber;
    uint8_t* imageBytes;
  };

  // What we pass between the producer and consumer.
  typedef ProducerConsumer<FramePacket, BUFFER_SIZE> ConsumerBuffer;

}
