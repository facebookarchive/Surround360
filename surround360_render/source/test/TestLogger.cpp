/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <iostream>
#include <string>
#include <vector>

#include "SystemUtil.h"

#include <glog/logging.h>

using namespace std;
using namespace surround360::util;

int main(int argc, char** argv) {
  initSurround360(argc, argv);

  LOG(INFO) << "something at info level";
  LOG(WARNING) << "something at warn level";
  LOG(ERROR) << "something at error level";
  LOG(FATAL) << "something at fatal level";

  return EXIT_SUCCESS;
}
