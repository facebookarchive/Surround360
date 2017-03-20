/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "SystemUtil.h"

#include <execinfo.h>
#include <signal.h>

#include <exception>
#include <stdexcept>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace fLB {
extern bool FLAGS_help;
extern bool FLAGS_helpshort;
}

namespace surround360 {
namespace util {

using namespace std;

void printStacktrace() {
  const size_t maxStackDepth = 128;
  void* stack[maxStackDepth];
  size_t stackDepth = backtrace(stack, maxStackDepth);
  char** stackStrings = backtrace_symbols(stack, stackDepth);
  for (size_t i = 0; i < stackDepth; ++i) {
    LOG(ERROR) << stackStrings[i];
  }
  free(stackStrings);
}

void terminateHandler() {
  exception_ptr exptr = current_exception();
  if (exptr != 0) {
    try {
      rethrow_exception(exptr);
    } catch (VrCamException &ex) {
      LOG(ERROR) << "Terminated with VrCamException: " << ex.what();
    } catch (exception &ex) {
      LOG(ERROR) << "Terminated with exception: " << ex.what();
      printStacktrace();
    } catch (...) {
      LOG(ERROR) << "Terminated with unknown exception";
      printStacktrace();
    }
  } else {
    LOG(ERROR) << "Terminated due to unknown reason";
    printStacktrace();
  }
  abort();
}

void sigHandler(int signal) {
  LOG(ERROR) << strsignal(signal);
  printStacktrace();
  abort();
}

void initSurround360(int argc, char** argv) {
  // Initialize Google's logging library
  google::InitGoogleLogging(argv[0]);

  // GFlags
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  fLB::FLAGS_helpshort = fLB::FLAGS_help;
  fLB::FLAGS_help = false;
  gflags::HandleCommandLineHelpFlags();

  // setup signal and termination handlers
  set_terminate(terminateHandler);

  // terminate process: terminal line hangup
  signal(SIGHUP, sigHandler);

  // terminate process: interrupt program
  signal(SIGINT, sigHandler);

  // create core image: quit program
  signal(SIGQUIT, sigHandler);

  // create core image: illegal instruction
  signal(SIGILL, sigHandler);

  // create core image: trace trap
  signal(SIGTRAP, sigHandler);

  // create core image: floating-point exception
  signal(SIGFPE, sigHandler);

  // terminate process: kill program
  signal(SIGKILL, sigHandler);

  // create core image: bus error
  signal(SIGBUS, sigHandler);

  // create core image: segmentation violation
  signal(SIGSEGV, sigHandler);

  // create core image: non-existent system call invoked
  signal(SIGSYS, sigHandler);

  // terminate process: write on a pipe with no reader
  signal(SIGPIPE, sigHandler);

  // terminate process: software termination signal
  signal(SIGTERM, sigHandler);
}

} // namespace util
} // namespace surround360
