/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <memory>
#include <vector>

#include "BinaryFootageFile.h"

namespace surround360 {

class RawConverter {
 public:
  static std::unique_ptr<std::vector<uint16_t>> convert8Frame(
    const void* rawFrame,
    const size_t width,
    const size_t height);

  static std::unique_ptr<std::vector<uint16_t>> convert12Frame(
    const void* rawFrame,
    const size_t width,
    const size_t height);
};

}  //end namespace surround360
