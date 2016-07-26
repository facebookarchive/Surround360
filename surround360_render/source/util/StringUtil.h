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

namespace surround360 {
namespace util {

using namespace std;

// takes a string and a delimiter character, and returns a vector of tokens
vector<string> stringSplit(const string& s, const char delim);

// concatenates all strings in the input vector into a single string, separated by delim's
string stringJoin(const string& delim, const vector<string>& stringsToJoin);

} // namespace util
} // namespace surround360
