/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <string>
#include <vector>

#include "thirdparty/supereasyjson/json.h"

#include "CvUtil.h"

namespace surround360 {

using namespace cv;
using namespace std;

// verifies that we can index 2 levels down with objKey, then key
void exceptionIfMissingKeys(
  const json::Object& obj, const string& objKey, const string& key);

// all of the get* functions below read a json object representing a 2-level dictionary
// with 2 string keys, and expect a particular type of object to be stored in the
// dictionary under those keys. if the keys are missing, or the type is unexpected, an
// exception will be thrown.
const json::Array getArray(
  const json::Object& obj, const string& objKey, const string& key);

const string getString(
  const json::Object& obj, const string& objKey, const string& key);

const double getDouble(
  const json::Object& obj, const string& objKey, const string& key);

const int getInteger(
  const json::Object& obj, const string& objKey, const string& key);

Point3f getVector(
  const json::Object& obj, const string& objKey, const string& key);

Mat getMatrix(
  const json::Object& obj, const string& objKey, const string& key);

vector<Point3f> getCoordList(
  const json::Object& obj, const string& objKey, const string& key);

} // end namespace surround360
