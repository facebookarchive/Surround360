/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "CameraIsp.h"
#ifdef USE_HALIDE
#include "CameraIspPipe.h"
#endif
#include "ColorCalibration.h"
#include "CvUtil.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include "ceres/ceres.h"
#include <folly/FileUtil.h>
#include <folly/json.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace surround360;
using namespace surround360::util;
using namespace surround360::color_calibration;

DEFINE_string(data_path,            "",     "path to json file with locations and RGB medians");
DEFINE_string(output_dir,           "",     "path to write data for debugging");
DEFINE_int32(image_width,           2048,   "image width");
DEFINE_int32(image_height,          2048,   "image height");
DEFINE_string(test_image_path,      "",     "path to test image");
DEFINE_string(test_isp_path,        "",     "passthrough ISP for test image");
DEFINE_bool(save_debug_images,      false,  "save intermediate images");

template<int TBezierX, int TBezierY>
struct BezierFunctor {
  static ceres::CostFunction* addResidual(
      ceres::Problem& problem,
      vector<double>& paramsX,
      vector<double>& paramsY,
      const double x,
      const double y,
      const double intensity) {

    CHECK_EQ(TBezierX + 1, paramsX.size());
    CHECK_EQ(TBezierY + 1, paramsY.size());

    auto* cost = new CostFunction(new BezierFunctor(x, y, intensity));
    problem.AddResidualBlock(
      cost,
      nullptr, // loss
      paramsX.data(),
      paramsY.data());
    return cost;
  }

  bool operator()(
      double const* const paramsX,
      double const* const paramsY,
      double* residuals) const {

    BezierCurve<float, double> bezierX;
    BezierCurve<float, double> bezierY;

    for (int i = 0; i <= TBezierX; ++i) {
      bezierX.addPoint(paramsX[i]);
    }
    for (int i = 0; i <= TBezierY; ++i) {
      bezierY.addPoint(paramsY[i]);
    }

    const double vx = bezierX(x);
    const double vy = bezierY(y);

    *residuals = vx * vy - intensity;
    CHECK(std::isfinite(*residuals));
    return true;
  }

 private:
  using CostFunction = ceres::NumericDiffCostFunction<
    BezierFunctor,
    ceres::CENTRAL,
    1, // residuals
    TBezierX + 1,
    TBezierY + 1>;

  BezierFunctor(const double x, const double y, const double intensity) :
      x(x),
      y(y),
      intensity(intensity) {
  }

  const double x;
  const double y;
  const double intensity;
};

void putTextShift(Mat& image, string text, Point p, Scalar color) {
  static const double kTextFontScale = 0.8;
  static const Point kShift = Point(10, 0);
  putText(
    image,
    text,
    p + kShift,
    FONT_HERSHEY_SIMPLEX,
    kTextFontScale,
    color);
}

void putCircleAndText(Mat& image, Point p, Scalar color) {
  static const int kRadius = 5;
  static const int kThickness = 2;
  circle(image, p, kRadius, color, kThickness);
  string text = "(" + to_string(p.x) + ", " + to_string(p.y) + ")";
  putTextShift(image, text, p, color);
}

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  requireArg(FLAGS_data_path, "data_path");

  if (FLAGS_test_image_path != "" && FLAGS_test_isp_path == "") {
    throw VrCamException("Need ISP file for test image");
  }

  string dataDir = FLAGS_data_path.substr(0, FLAGS_data_path.find_last_of('/'));
  const string outputDir =
    FLAGS_output_dir.empty() ? dataDir + "/output" : FLAGS_output_dir;
  system(string("mkdir -p \"" + outputDir + "\"").c_str());

  LOG(INFO) << "Loading data...";

  // Assuming JSON file with format
  // [
  //   {
  //     "location" : [x1, y1],
  //     "rgbmedian" : [r1, g1, b1]
  //   },
  //   ...
  //   {
  //     "location" : [xN, yN],
  //     "rgbmedian" : [rN, gN, bN]
  //   }
  //  ]

  string json;
  folly::readFile(FLAGS_data_path.c_str(), json);
  if (json.empty()) {
    throw VrCamException("Failed to load JSON file: " + FLAGS_data_path);
  }

  vector<Point2f> vignetteMapLocs;
  vector<Vec3f> vignetteMapRGBValues;
  folly::dynamic data = folly::parseJson(json);
  static const int kNumChannels = 3;
  vector<float> minRGB(kNumChannels, INT_MAX);
  vector<float> maxRGB(kNumChannels, 0);
  for (const auto& sample : data) {
    const folly::dynamic loc = sample["location"];
    const folly::dynamic median = sample["rgbmedian"];
    vignetteMapLocs.push_back(Point2f(loc[0].asDouble(), loc[1].asDouble()));
    const Vec3f v = Vec3f(
      median[0].asDouble(),
      median[1].asDouble(),
      median[2].asDouble());
    vignetteMapRGBValues.push_back(v);

    // Find min and max of each channel as we load data
    for (int ch = 0; ch < kNumChannels; ++ch) {
      minRGB[ch] = std::min(minRGB[ch], v[ch]);
      maxRGB[ch] = std::max(maxRGB[ch], v[ch]);
    }
  }

  if (FLAGS_save_debug_images) {
    vector<Mat> vignetteMapPlotRGB;
    for (int i = 0; i < kNumChannels; ++i) {
      Mat ch = Mat::zeros(FLAGS_image_height, FLAGS_image_width, CV_8UC1);
      vignetteMapPlotRGB.push_back(ch);
    }

    // Stretch data for display purposes
    for (int i = 0; i < vignetteMapLocs.size(); ++i) {
      for (int ch = 0; ch < kNumChannels; ++ch) {
        const float v = vignetteMapRGBValues[i][ch];
        const float valNorm =
          255.0f * (v - minRGB[ch]) / (maxRGB[ch] - minRGB[ch]);
        vignetteMapPlotRGB[ch].at<uchar>(vignetteMapLocs[i]) = valNorm;
        static const int kRadius = 5;
        static const int kThicknessFill = -1;
        circle(
          vignetteMapPlotRGB[ch],
          vignetteMapLocs[i],
          kRadius,
          valNorm,
          kThicknessFill);
        circle(
          vignetteMapPlotRGB[ch],
          vignetteMapLocs[i],
          kRadius,
          255);
      }
    }

    for (int ch = 0; ch < kNumChannels; ++ch) {
      Mat vignetteMapPlotJet;
      applyColorMap(vignetteMapPlotRGB[ch], vignetteMapPlotJet, COLORMAP_JET);
      const char chName = "rgb"[ch];
      const string vignetteMapPlotFilename =
        outputDir + "/sample_map_" + chName + "_stretched.png";
      imwriteExceptionOnFail(vignetteMapPlotFilename, vignetteMapPlotJet);
    }
  }

  // Approximating falloff using Nth order Bezier surface
  // p(u, v) = (sum_i=0^N b_i * B_i^N(u)) * (sum_j=0^N b_j * B_j^N(v))
  static const int kBezierX = 4;
  static const int kBezierY = 4;

  vector<Point3f> vignetteRollOffH(kBezierX + 1);
  vector<Point3f> vignetteRollOffV(kBezierY + 1);
  const int maxDimension = std::max(FLAGS_image_width, FLAGS_image_height);
  for (int ch = 0; ch < kNumChannels; ++ch) {
    ceres::Problem problem;

    // Initialize estimates
    vector<double> paramsX(kBezierX + 1, 0.5);
    vector<double> paramsY(kBezierY + 1, 0.5);

    LOG(INFO) << "Creating residuals...";

    for (int i = 0; i < vignetteMapLocs.size(); ++i) {
      const double x = double(vignetteMapLocs[i].x) / double(maxDimension);
      const double y = double(vignetteMapLocs[i].y) / double(maxDimension);
      const float v = vignetteMapRGBValues[i][ch] / maxRGB[ch];

      // Use inverse to directly get vignetting correction surface
      BezierFunctor<kBezierX, kBezierX>::addResidual(
        problem, paramsX, paramsY, x, y, 1.0f / v);
    }

    LOG(INFO) << "Solving...";

    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = FLAGS_save_debug_images;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    LOG(INFO) << summary.FullReport();

    LOG(INFO) << "curveFit parameters...";

    std::ostringstream paramsStreamX;
    for (int i = 0; i <= kBezierX; ++i) {
      if (ch == 0) {
        vignetteRollOffH[i].x = paramsX[i];
      } else if (ch == 1) {
        vignetteRollOffH[i].y = paramsX[i];
      } else {
        vignetteRollOffH[i].z = paramsX[i];
      }
      paramsStreamX << "X" << i << ":" << paramsX[i] << " ";
    }
    std::ostringstream paramsStreamY;
    for (int i = 0; i <= kBezierY; ++i) {
      if (ch == 0) {
        vignetteRollOffV[i].x = paramsY[i];
      } else if (ch == 1) {
        vignetteRollOffV[i].y = paramsY[i];
      } else {
        vignetteRollOffV[i].z = paramsY[i];
      }
      paramsStreamY << "Y" << i << ":" << paramsY[i] << " ";
    }
    LOG(INFO) << paramsStreamX.str();
    LOG(INFO) << paramsStreamY.str();

    // Save report to file
    const char chName = "rgb"[ch];
    const string ceresFilename = outputDir + "/ceres_report_" + chName + ".txt";
    ofstream ceresStream(ceresFilename, ios::out);
    if (!ceresStream) {
      throw VrCamException("file open failed: " + ceresFilename);
    }
    ceresStream << summary.FullReport();
    ceresStream << paramsStreamX.str() << "\n" << paramsStreamY.str();
    ceresStream.close();

    if (FLAGS_save_debug_images) {
      LOG(INFO) << "Creating surface fit for channel " << ch << "...";

      BezierCurve<float, double> bezierX(paramsX);
      BezierCurve<float, double> bezierY(paramsY);

      Mat surfaceFit = Mat::zeros(FLAGS_image_height, FLAGS_image_width, CV_32FC1);
      for (int x = 0; x < FLAGS_image_width; ++x) {
        const float vx = bezierX(float(x) / float(maxDimension));
        for (int y = 0; y < FLAGS_image_height; ++y) {
          const float vy = bezierY(float(y) / float(maxDimension));
          surfaceFit.at<float>(Point(x, y)) = vx * vy;
        }
      }

      double minVal;
      double maxVal;
      Point maxLoc;
      Point minLoc;
      minMaxLoc(surfaceFit, &minVal, &maxVal, &minLoc, &maxLoc);
      LOG(INFO) << "surfaceFit min: " << minVal << ", at " << minLoc;
      LOG(INFO) << "surfaceFit max: " << maxVal << ", at " << maxLoc;

      Mat surfacePlot = 255.0f * surfaceFit / maxVal;
      surfacePlot.convertTo(surfacePlot, CV_8UC1);

      // Plot parameters
      static const Point kTextLoc = Point(0, 50);
      static const Scalar kColor = 0;
      putTextShift(surfacePlot, paramsStreamX.str(), kTextLoc, kColor);
      putTextShift(surfacePlot, paramsStreamY.str(), kTextLoc * 2, kColor);

      // Plot image center and surface minima location
      putCircleAndText(surfacePlot, minLoc, kColor);
      Point imageCenter = Point(FLAGS_image_width / 2, FLAGS_image_height / 2);
      putCircleAndText(surfacePlot, imageCenter, kColor);

      Mat surfacePlotJet;
      applyColorMap(surfacePlot, surfacePlotJet, COLORMAP_JET);
      const string surfaceFitPlotFilename =
        outputDir + "/curveFit_fit_" + chName + "_stretched.png";
      imwriteExceptionOnFail(surfaceFitPlotFilename, surfacePlotJet);
    }
  }

  if (FLAGS_test_image_path == "") {
    return EXIT_SUCCESS;
  }

  Mat rawTest = imreadExceptionOnFail(
    FLAGS_test_image_path, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
  const uint8_t rawDepth = rawTest.type() & CV_MAT_DEPTH_MASK;
  if (rawDepth == CV_8U) {
    rawTest = convert8bitTo16bit(rawTest);
  } else if (rawDepth != CV_16U) {
    throw VrCamException("Test image is not 8-bit or 16-bit");
  }

  LOG(INFO) << "Updating ISP config file...";

  string ispOutFilename = outputDir + "/isp_out.json";
  CameraIsp cameraIsp(getJson(FLAGS_test_isp_path), getBitsPerPixel(rawTest));
  cameraIsp.setVignetteRollOffH(vignetteRollOffH);
  cameraIsp.setVignetteRollOffV(vignetteRollOffV);
  cameraIsp.setup();
  cameraIsp.loadImage(rawTest.clone());
  cameraIsp.dumpConfigFile(ispOutFilename);

  if (FLAGS_save_debug_images) {
    LOG(INFO) << "Testing vignetting...";

    const string rawTestFilename = outputDir + "/test_in.png";
    imwriteExceptionOnFail(rawTestFilename, 255.0f * cameraIsp.getRawImage());

    static const int kOutputBpp = 16;
    Mat rawTestIspOut(rawTest.size(), CV_16UC3);

  #ifdef USE_HALIDE
    static const bool kFast = false;
    CameraIspPipe cameraIspTest(getJson(ispOutFilename), kFast, kOutputBpp);
  #else
    CameraIsp cameraIspTest(getJson(ispOutFilename), kOutputBpp);
  #endif

    cameraIspTest.setup();
    cameraIspTest.loadImage(rawTest);
    cameraIspTest.getImage(rawTestIspOut);
    const string rawTestIspOutFilename = outputDir + "/test_out_rgb.png";
    imwriteExceptionOnFail(rawTestIspOutFilename, rawTestIspOut);
  }

  return EXIT_SUCCESS;
}
