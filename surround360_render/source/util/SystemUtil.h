/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <assert.h>
#include <dirent.h>
#include <math.h>

#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "CvUtil.h"
#include "StringUtil.h"
#include "VrCamException.h"

namespace surround360 {
namespace util {

using namespace std;
using namespace std::chrono;

// this should be the first line of most main() function in this project. sets up glog,
// gflags, and enables stack traces to be triggered when the program stops due to an
// exception
void initSurround360(int argc, char** argv);

void printStacktrace();

// example use:
//
//DEFINE_string(foo, "", "foo is a required string");
//...
//requireArg(FLAGS_foo, "foo");
static void requireArg(const string& argValue, const string& argName) {
  if (argValue.empty()) {
    throw VrCamException("missing required command line argument: " + argName);
  }
}

// example use:
//
//DEFINE_int32(foo, -1, "foo is required positive integer");
//...
//requireArgGeqZero(FLAGS_foo, "foo");
static void requireArgGeqZero(const int& argValue, const string& argName) {
  if (argValue < 0) {
    throw VrCamException("missing required arg or must be >= 0: " + argName);
  }
}

// return the current system time in seconds. reasonably high precision.
static double getCurrTimeSec() {
  return (double)(system_clock::now().time_since_epoch().count()) * system_clock::period::num / system_clock::period::den;
}

// scans srcDir for all files/folders, and return a vector of filenames (or full file
// paths if fullPath is true)
static vector<string> getFilesInDir(
    const string& srcDir,
    const bool fullPath,
    int numFilesToReturn = -1) {

  DIR* dir = opendir(srcDir.c_str());
  if (!dir) { return vector<string>(); }

  vector<string> out_file_names;
  dirent* dent;
  while (true) {
    dent = readdir(dir);
    if (!dent) break;
    // skip hidden files and/or links to parent dir
    if (string(dent->d_name)[0] == '.') continue;
    if (fullPath) {
      out_file_names.push_back(srcDir + "/" + string(dent->d_name));
    } else {
      out_file_names.push_back(string(dent->d_name));
    }
    if (--numFilesToReturn == 0) {
      break;
    }
  }
  return out_file_names;
}

static string getImageFileExtension(const string& imageDir) {
  // figure out the file extension used by the images. This is complicated but
  // it's all so we can iterate over the images in the right order.
  // assumption: no weird mixtures of file extension
  vector<string> imageFilenames = getFilesInDir(imageDir, false, 1);
  assert(imageFilenames.size() > 0);
  vector<string> firstFilenameParts = stringSplit(imageFilenames[0], '.');
  assert(firstFilenameParts.size() == 2);
  return firstFilenameParts[1];
}

// Thread functor wrapper.
template <typename T>
struct Threadable {
  shared_ptr<thread> runningThread;
  shared_ptr<T> threadFunctor;

  Threadable(T* threadFunctor) :
      threadFunctor(threadFunctor),
      runningThread(new thread(*threadFunctor)) {
  }
};

// wrapper for imreadExceptionOnFail( for use in std::threads
static void imreadInStdThread(string path, int flags, Mat* dest) {
  *dest = imreadExceptionOnFail(path, flags);
}

} // namespace util
} // namespace surround360
