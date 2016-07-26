/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "thirdparty/supereasyjson/json.h"

#include "SystemUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace surround360::util;

// tests of the SuperEasy JSON library
int main(int argc, char** argv) {
  initSurround360(argc, argv);

  LOG(INFO) << "json<-->string deserialize, serialize";

  // only double quotes are supported as string keys
  static const string kTestStr = "[{\"foo\":1, \"bar\":2}, 123, true, 3.1415926535897]";
  json::Value testList = json::Deserialize(kTestStr);
  LOG(INFO) << json::Serialize(testList);

  // array access
  json::Array a = testList.ToArray();
  json::Object a0 = a[0];
  int a1 = a[1].ToInt();
  bool a2 = a[2].ToBool();
  double a3 = a[3].ToDouble();
  LOG(INFO) << "array size=" << a.size();
  LOG(INFO) << json::Serialize(a0);
  LOG(INFO) << a1;
  LOG(INFO) << a2;
  LOG(INFO) << setprecision(15) << a3;

  // map access by key
  int foo = a0["foo"].ToInt();
  int bar = a0["bar"].ToInt();
  LOG(INFO) << "foo=" << foo;
  LOG(INFO) << "bar=" << bar;

  // test a case where we try to use something as the wrong type
  json::Value testBadType = json::Deserialize("12345");
  testBadType.ToArray(); // (it's an int, not an array)

  return EXIT_SUCCESS;
}
