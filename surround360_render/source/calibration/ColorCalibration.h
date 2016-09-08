/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <list>
#include <vector>

#include "opencv2/imgproc.hpp"

namespace surround360 {
namespace color_calibration {

using namespace std;
using namespace cv;

struct ColorPatch {
  int width, height, area;
  Point2f centroid;
  Vec3f rgbMedian;
  Mat mask;
};

static bool sortPointsX(const Point pt1, const Point pt2) {
  return pt1.x < pt2.x;
}

// MacBeth Linear RGB patches from http://www.babelcolor.com/colorchecker-2.htm#CCP2_data
// Un-gamma corrected sRGB: INT(255 * (sRGB/255)^2.2)
const vector<vector<int>> rgbLinearMacbeth = {
  {44, 20, 13},
  {147, 76, 56},
  {26, 50, 86},
  {25, 38, 12},
  {57, 55, 112},
  {27, 133, 107},
  {192, 51, 6},
  {13, 26, 104},
  {146, 20, 30},
  {28, 9, 36},
  {89, 131, 11},
  {204, 93, 4},
  {3, 11, 76},
  {13, 78, 16},
  {118, 6, 9},
  {219, 146, 0},
  {138, 22, 80},
  {0, 64, 104},
  {233, 233, 228},
  {150, 153, 153},
  {92, 95, 95},
  {49, 49, 49},
  {21, 22, 23},
  {6, 6, 7}
};

// Converts raw image to RGB using our ISP pipeline. It takes an input ISP
// config file, and accepts custom black level, white balance gains, CCM, and
// gamma values. It can save a new ISP config file with the given parameters
Mat raw2rgb(
  const string& ispConfigFileIn,
  const Mat& raw,
  const Point3f blackLevel = Point3f(0.0, 0.0, 0.0),
  const Vec3f& whiteBalanceGain = Vec3f(1.0, 1.0, 1.0),
  const Mat& ccm = Mat::eye(3, 3, CV_32F),
  const Point3f gamma = Point3f(1.0, 1.0, 1.0),
  const string& ispConfigFileOut = "");

// Loads the given JSON file into a usable string
string getJson(const string& filename);

// Finds the black level on the input grayscale image. Assumes color chart is
// on the center 50% of the image
Point3f findBlackPoint(
  const Mat& image,
  const string& ispConfigFile,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Creates a rectangular mask from image center, of width and height given by
// the input percentage (in range 0.0 to 1.0)
Mat createMaskFromCenter(const Mat& image, const float percentage);

// Gets closest R, G and B values from Bayer pattern, given by the input ISP
// config file
Point3f getClosestRGB(
  const Mat& image,
  const Point minLoc,
  const string& ispConfigFile);

// Detects color chart patches on the input image. Returns a list of color
// patches containing location, shape and color information
vector<ColorPatch> detectColorChart(
  const Mat& image,
  const int numSquaresW,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Applies a morphological opening followed by a morphological closing
Mat morphOpeningAndClosing(
  const Mat& imageIn,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Finds straight contours on input image
vector<vector<Point>> findContours(
  const Mat& image,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Removes outliers from given color patch list
vector<ColorPatch> removeContourOutliers(vector<ColorPatch> colorPatchList);

// Sorts patches from top left to bottom right
vector<ColorPatch> sortPatches(
  const vector<ColorPatch> colorPatchList,
  const int numSquaresW,
  const Size imageSize);

// Finds the point closest to the top-left corner of the image
Point2f findTopLeft(const vector<Point2f> points);

// Finds the point closest to the top-right corner of the image
Point2f findTopRight(const vector<Point2f> points, const int imageWidth);

// Finds the distance from a point to a line defined by two points
float pointToLineDistance(
  const Point2f p,
  const Point2f pLine1,
  const Point2f pLine2);

// Compute RGB medians of each given color patch
void computeRGBMedians(
  vector<ColorPatch>& colorPatches,
  const Mat& bgr,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Compute channel ratios for white balance
Vec3f computeChannelRatios(
  const vector<ColorPatch>& colorPatches,
  const int numPatchesRow);

// Plots white balance histograms for debugging purposes
void plotWhiteBalanceHistogram(
  const vector<ColorPatch> colorPatches,
  const Vec3f channelRatios,
  const int numPatchesRow,
  const string& outputDir,
  int& stepDebugImages);

// Compute color correction matrix from patch info
Mat computeCCM(const vector<ColorPatch> colorPatches);

// Compute L2 errors between corrected color patches and MacBeth ground truth
// The output is a pair of vectors such that
// Vector 1: errRGB, errB, errG, errR BEFORE correction
// Vector 2: errRGB, errB, errG, errR AFTER correction
pair<Vec4f, Vec4f> computeColorPatchErrors(
  const Mat& imBefore,
  const Mat& imAfter,
  const vector<ColorPatch> colorPatches);

} // namespace color_calibration
} // namespace surround360
