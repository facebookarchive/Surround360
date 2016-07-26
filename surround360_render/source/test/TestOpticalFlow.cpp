/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "CvUtil.h"
#include "NovelView.h"
#include "OpticalFlowVisualization.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;
using namespace surround360::optical_flow;

DEFINE_string(mode,                       "",     "command line mode");
DEFINE_string(test_dir,                   "",     "path to dir with test files");
DEFINE_string(left_img,                   "",     "path to left image (relative to test_dir)");
DEFINE_string(right_img,                  "",     "path to right image  (relative to test_dir)");
DEFINE_int32(num_intermediate_views,      11,     "number of views to make");
DEFINE_string(flow_alg,                   "",     "optical flow algorithm to use");
DEFINE_int32(repetitions,                 1,      "number of times to repeat the flow calculation");
DEFINE_bool(save_asymmetric_novel_views,  false,  "if true, we will save the non-merged novel views that are obtained by warping the left/right images, in addition to the combined novel view");

// reads a pair of images specified by --left_img and --right_img. applies an optical flow
// algorithm specified by --flow_alg. generates a visualization of the flow field, and a
// sequence of novel views morphing between the two images. results are saved to
// subdirectories of --test_dir: /disparity and /novel_view
void testDisparity(const string imagePathL, const string imagePathR) {
  requireArg(FLAGS_test_dir, "test_dir");
  requireArg(FLAGS_left_img, "left_img");
  requireArg(FLAGS_right_img, "right_img");
  requireArg(FLAGS_flow_alg, "flow_alg");

  imwriteExceptionOnFail(FLAGS_test_dir + "/colorwheel.png", testColorWheel());

  LOG(INFO) << "reading opencv image matrices";
  Mat colorImageL = imreadExceptionOnFail(imagePathL, -1); // -1 = load RGBA
  Mat colorImageR = imreadExceptionOnFail(imagePathR, -1);

  // we need alpha channels for flow. if they are missing, convert
  if (colorImageL.type() == CV_8UC3) {
    cvtColor(colorImageL, colorImageL, CV_BGR2BGRA);
  }
  if (colorImageR.type() == CV_8UC3) {
    cvtColor(colorImageR, colorImageR, CV_BGR2BGRA);
  }

  for (int rep = 0; rep < FLAGS_repetitions; ++rep) {
    LOG(INFO) << "---- repetition " << rep;

    LOG(INFO) << "constructing novel view generator";
    NovelViewGenerator* novelViewGen =
      new NovelViewGeneratorAsymmetricFlow(FLAGS_flow_alg);

    LOG(INFO) << "calling prepare";
    double prepareStartTime = getCurrTimeSec();
    const Mat prevFlowLtoR = Mat();
    const Mat prevFlowRtoL = Mat();
    const Mat prevColorImageL = Mat();
    const Mat prevColorImageR = Mat();

    novelViewGen->prepare(
      colorImageL,
      colorImageR,
      prevFlowLtoR,
      prevFlowRtoL,
      prevColorImageL,
      prevColorImageR);
    double prepareEndTime = getCurrTimeSec();
    LOG(INFO) << "RUNTIME (sec) = " << (prepareEndTime - prepareStartTime);

    LOG(INFO) << "building visualizations";

    Mat flowVisLtoR                = visualizeFlowAsGreyDisparity(novelViewGen->getFlowLtoR());
    Mat flowVisRtoL                = visualizeFlowAsGreyDisparity(novelViewGen->getFlowRtoL());
    Mat flowVisLtoRColorWheel      = visualizeFlowColorWheel(novelViewGen->getFlowLtoR());
    Mat flowVisRtoLColorWheel      = visualizeFlowColorWheel(novelViewGen->getFlowRtoL());
    Mat flowVisLtoRColorWithLines  = visualizeFlowAsVectorField(novelViewGen->getFlowLtoR(), colorImageL);
    Mat flowVisRtoLColorWithLines  = visualizeFlowAsVectorField(novelViewGen->getFlowRtoL(), colorImageR);

    cvtColor(flowVisRtoL,                 flowVisRtoL,                CV_GRAY2BGRA);
    cvtColor(flowVisLtoR,                 flowVisLtoR,                CV_GRAY2BGRA);
    cvtColor(flowVisLtoRColorWheel,       flowVisLtoRColorWheel,      CV_BGR2BGRA);
    cvtColor(flowVisRtoLColorWheel,       flowVisRtoLColorWheel,      CV_BGR2BGRA);

    Mat horizontalVisLtoR = stackHorizontal(
      vector<Mat>({flowVisLtoR, flowVisLtoRColorWheel, flowVisLtoRColorWithLines}));
    Mat horizontalVisRtoL = stackHorizontal(
      vector<Mat>({flowVisRtoL, flowVisRtoLColorWheel, flowVisRtoLColorWithLines}));

    imwriteExceptionOnFail(
      FLAGS_test_dir + "/disparity/LtoR_" + FLAGS_flow_alg + ".png",
      horizontalVisLtoR);

    imwriteExceptionOnFail(
      FLAGS_test_dir + "/disparity/RtoL_" + FLAGS_flow_alg +".png",
      horizontalVisRtoL);

    system(string("rm " + FLAGS_test_dir + "/novel_view/*").c_str());

    for (int v = 0; v < FLAGS_num_intermediate_views; ++v) {
      const double shiftFromLeft =
        double(v) / double(FLAGS_num_intermediate_views - 1);

      Mat novelViewMerged = Mat(); // init here so we don't crash if nothing
      Mat novelViewFromL = Mat();  // is written
      Mat novelViewFromR = Mat();
      novelViewGen->generateNovelView(
        shiftFromLeft, novelViewMerged, novelViewFromL, novelViewFromR);

      stringstream ss;
      ss << std::setfill('0') << std::setw(6) << v;
      const string frameIdxPadded = ss.str();

      imwriteExceptionOnFail(
        FLAGS_test_dir + "/novel_view/" + frameIdxPadded + ".png",
        novelViewMerged);

      if (FLAGS_save_asymmetric_novel_views) {
        imwriteExceptionOnFail(
          FLAGS_test_dir + "/novel_view/novelFromL_" + frameIdxPadded + ".png",
          novelViewFromL);

        imwriteExceptionOnFail(
          FLAGS_test_dir + "/novel_view/novelFromR_" + frameIdxPadded + ".png",
          novelViewFromR);
      }
    }

    delete novelViewGen;
  }
}

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_mode, "mode");

  if (FLAGS_mode == "test") {
    testDisparity(
      FLAGS_test_dir + "/" + FLAGS_left_img,
      FLAGS_test_dir + "/" + FLAGS_right_img);
  } else {
    throw VrCamException("unrecongized mode: " + FLAGS_mode);
  }
  return EXIT_SUCCESS;
}
