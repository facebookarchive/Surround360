/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl_ui file in the root directory of this subproject.
*/

#pragma once

extern "C" {
#include <cstddef>
#include <sys/mman.h>
}

namespace surround360 {
namespace util {
template <typename T, int ALIGNMENT = 64>
class aligned_allocator {
public:
  using value_type = T;
  using pointer = value_type*;
  using const_pointer = const value_type*;
  using reference = value_type&;
  using const_reference = const value_type&;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  template<typename U>
  struct rebind {
    using other = aligned_allocator<U, ALIGNMENT>;
  };

  inline explicit aligned_allocator() = default;
  inline ~aligned_allocator() = default;
  inline explicit aligned_allocator(const aligned_allocator& a) = default;

  inline pointer address(reference r) { return &r; }
  inline const_pointer address(const_reference r) { return &r; }

  inline pointer allocate(size_type sz, typename std::allocator<void>::const_pointer = 0) {
    auto x = memalign(ALIGNMENT, sizeof(T) * sz);
    mlock(x, sizeof(T) * sz); // opportunistically mlock as much as possible
    return reinterpret_cast<pointer>(x);
  }

  void deallocate(pointer p, size_type sz) {
    munlock(p, sizeof(T) * sz);
    free(p);
  }
};
}
}
