#include "Raw12Converter.hpp"

using namespace surround360;
using namespace std;

unique_ptr<vector<uint16_t>> Raw12Converter::convertFrame(
  const void* rawFrame, const size_t width, const size_t height) {
  auto frame = reinterpret_cast<const uint8_t*>(rawFrame);
  auto result = make_unique<vector<uint16_t>>(width * height);

  uint32_t index = 0;

  for (size_t y = 0; y < height; ++y) {
    for (size_t x = 0; x < width; ++x) {
      const uint16_t lo = frame[index];
      const uint16_t hi = frame[index + 1];
      uint16_t unswizzled;

      if (x & 1) {
        index += 2;
        unswizzled = hi << 4 | lo >> 4;
      } else {
        ++index;
        unswizzled = lo << 4 | (hi & 0xF);
      }

      (*result)[y * width + x] = unswizzled << 4 | unswizzled >> 8;
    }
  }

  return result;
}
