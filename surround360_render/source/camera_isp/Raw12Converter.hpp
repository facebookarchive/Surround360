#pragma once

#include <vector>
#include <memory>
#include "BinaryFootageFile.hpp"

namespace surround360 {
class Raw12Converter {
 public:
  static std::unique_ptr<std::vector<uint16_t>> convertFrame(
    const void* rawFrame, const size_t width, const size_t height);
};
}
