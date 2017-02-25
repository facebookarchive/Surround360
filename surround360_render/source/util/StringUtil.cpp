/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "StringUtil.h"

#include <string>
#include <vector>

namespace surround360 {
namespace util {

using namespace std;

vector<string> stringSplit(const string& s, const char delim) {
  vector<string> parts;
  string part = "";
  for(unsigned int i = 0; i < s.length(); ++i) {
    if(s[i] == delim) {
      parts.push_back(part);
      part = "";
    } else {
      part += s[i];
    }
  }
  parts.push_back(part);
  return parts;
}

string stringJoin(const string& delim, const vector<string>& stringsToJoin) {
  string s = "";
  for (int i = 0; i < stringsToJoin.size(); ++i) {
    s += stringsToJoin[i];
    if (i != stringsToJoin.size() - 1) {
      s += delim;
    }
  }
  return s;
}

string intToStringZeroPad(const int x, const int padlen) {
  ostringstream ss;
  ss << setw(padlen) << setfill('0') << x;
  return ss.str();
}

} // namespace util
} // namespace surround360
