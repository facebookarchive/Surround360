/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */
#pragma once

#include <vector>
#include <memory>
#include <string>

namespace surround360 {
  class Camera {
  public:
    virtual ~Camera() {}
    virtual int attach() = 0;
    virtual int detach() = 0;
    virtual int init(bool isMaster) = 0;
    virtual int startCapture() = 0;
    virtual int stopCapture() = 0;
    virtual void* getFrame() = 0;
    virtual unsigned int getDroppedFramesCounter() const = 0;
    virtual int powerCamera(bool onOff) = 0;

  protected:
    Camera() {}
    Camera(const Camera& c) = delete;
  };
}
