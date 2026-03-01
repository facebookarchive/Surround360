/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "RawConverter.h"

using namespace surround360;
using namespace std;

unique_ptr<vector<uint16_t>> RawConverter::convert8Frame(
    const void* rawFrame,
    const size_t width,
    const size_t height) {

  auto frame = reinterpret_cast<const uint8_t*>(rawFrame);
  auto result = make_unique<vector<uint16_t>>(width * height);

  uint32_t p = 0;
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      uint32_t pixval = frame[p];
      (*result)[y * width + x] = pixval * 0x101;
      ++p;
    }
  }
  return result;
}

unique_ptr<vector<uint16_t>> RawConverter::convert12Frame(
    const void* rawFrame,
    const size_t width,
    const size_t height) {

  auto frame = reinterpret_cast<const uint8_t*>(rawFrame);
  auto result = make_unique<vector<uint16_t>>(width * height);

  uint32_t p = 0;
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      uint16_t lo = frame[p];
      uint16_t hi = frame[p + 1];
      uint16_t unswizzled, rep;
      if (x & 1) {
        p += 2;
        unswizzled = hi << 4 | lo >> 4;
      } else {
        p += 1;
        unswizzled = lo << 4 | (hi & 0xF);
      }
      (*result)[y * width + x] = unswizzled << 4 | unswizzled >> 8;
    }
  }
  return result;
}
