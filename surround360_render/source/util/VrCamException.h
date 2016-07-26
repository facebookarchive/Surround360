/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <exception>
#include <iostream>
#include <string>

namespace surround360 {

struct VrCamException : public std::exception {
  std::string msg;
  VrCamException() {}
  VrCamException(const std::string& msg) : msg(msg) {}
  const char* what() const noexcept { return msg.c_str(); }
};

} // namespace surround360
