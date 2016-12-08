/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <vector>

#include "opencv2/imgproc.hpp"

namespace surround360 {
namespace color_calibration {

using namespace std;
using namespace cv;

struct ColorPatch {
  Point2f centroid;
  Mat mask;
  Vec3f rgbMedian;
};

struct ColorResponse {
  Vec3f rgbInterceptXMax;
  Vec3f rgbInterceptXMin;
  Vec3f rgbInterceptY;
  Vec3f rgbSlope;
};

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

// Get image bit depth
int getBitsPerPixel(const Mat& image);

// Loads the given JSON file into a usable string
string getJson(const string& filename);

// Gets ground truth values for each gray patch on the MacBeth chart
vector<int> getMacBethGrays();

// Loads raw image using the ISP setup, so output is 16-bit and [0..1]
Mat getRaw(const string& ispConfigFile, const Mat& image);

// Created a grayscale map where crushed pixels are set to 0 and saturated
// pixels are set to 1. Assuming 8-bit input (it's just for visualization)
Mat findClampedPixels(const Mat& image8);

// Computes color channel responses from the grayscale patches on the
// colorchecker
ColorResponse computeRGBResponse(
  const Mat& raw,
  const bool isRaw,
  vector<ColorPatch>& colorPatches,
  const string& ispConfigFile,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages,
  const string& titleExtra);

// Saves black level of each channel to text file
void saveBlackLevel(const Vec3f& blackLevel, const string& outputDir);

// Saves X-intercepts of each channel for the given RGB response. Saves values
// to text file
void saveXIntercepts(const ColorResponse& colorResponse, const string& outputDir);

// Applies ISP black level adjustment to input raw image
Mat adjustBlackLevel(
  const string& ispConfigFile,
  const Mat& rawRef,
  const Mat& raw,
  const Point3f& blackLevel);

// Applies ISP white balance gains to input raw image
Mat whiteBalance(
  const string& ispConfigFile,
  const Mat& rawRef,
  const Mat& raw,
  const Vec3f& whiteBalanceGain);

// Applies ISP clamping and contrast stretching
// contrast
Mat clampAndStretch(
  const string& ispConfigFile,
  const Mat& rawRef,
  const Mat& raw,
  const ColorResponse& colorResponse,
  Vec3f& rgbClampMin,
  Vec3f& rgbClampMax);

// Applies ISP demosaicing step
Mat demosaic(const string& ispConfigFile, const Mat& rawRef, const Mat& raw);

// Applies ISP color correction
Mat colorCorrect(
  const string& ispConfigFile,
  const Mat& rawRef,
  const Mat& rgb,
  const Mat& ccm,
  const Vec3f& gamma = Vec3f(1.0f, 1.0f, 1.0f));

// Generates ISP config file with all the given parameters
void writeIspConfigFile(
  const string& ispConfigFileIn,
  const string& ispConfigFileOut,
  const Mat& raw,
  const Vec3f& blackLevel = Vec3f(0.0f, 0.0f, 0.0f),
  const Vec3f& whiteBalanceGain = Vec3f(1.0f, 1.0f, 1.0f),
  const Vec3f& clampMin = Vec3f(0.0f, 0.0f, 0.0f),
  const Vec3f& clampMax = Vec3f(1.0f, 1.0f, 1.0f),
  const Mat& ccm = Mat::eye(3, 3, CV_32F),
  const Vec3f& gamma = Vec3f(1.0f, 1.0f, 1.0f));

// Finds black level. Assumes there's a black hole in the input image
Vec3f findBlackLevel(
  const Mat& raw16,
  const string& ispConfigFile,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Computes one dimensional histogram of input image
Mat computeHistogram(const Mat& image, const Mat& mask);

// Detects color chart patches on the input image. Returns a list of color
// patches containing location, shape and color information
vector<ColorPatch> detectColorChart(
  const Mat& image,
  const int numSquaresW,
  const int numSquaresH,
  const float minAreaChart,
  const float maxAreaChart,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Fills gaps in input binary image
Mat fillGaps(
  const Mat& imageBw,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Dilates gaps to avoid outliers on contour detection
Mat dilateGaps(
  const Mat& imageBw,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Creates structuring element of given shape for morphological operations
Mat createMorphElement(const Size imageSize, const int shape);

// Removes small objects
Mat removeSmallObjects(
  const Mat& imageBw,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Finds straight contours on input image
vector<vector<Point>> findContours(
  const Mat& image,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages,
  const float straightenFactor);

// Removes outliers from given color patch list
vector<ColorPatch> removeContourOutliers(vector<ColorPatch> colorPatchList);

// Sorts patches from top left to bottom right
vector<ColorPatch> sortPatches(
  const vector<ColorPatch>& colorPatchList,
  const int numSquaresW,
  const Size imageSize);

// Finds the point closest to the top-left corner of the image
Point2f findTopLeft(const vector<Point2f>& points);

// Finds the point closest to the top-right corner of the image
Point2f findTopRight(const vector<Point2f>& points, const int imageWidth);

// Finds the distance from a point to a line defined by two points
float pointToLineDistance(
  const Point2f p,
  const Point2f pLine1,
  const Point2f pLine2);

// Draws color patches on top of given image
Mat drawPatches(const Mat& image, vector<ColorPatch>& colorPatches);

// Computes RGB medians of each given color patch
void computeRGBMedians(
  vector<ColorPatch>& colorPatches,
  const Mat& bgr,
  const bool isRaw,
  const string& ispConfigFile);

// Computes RGB medians on given mask
Vec3f getRgbMedianMask(
  const Mat& image,
  const Mat& mask,
  const string& ispConfigFile,
  const bool isRaw);

Vec3f plotGrayPatchResponse(
  vector<ColorPatch>& colorPatches,
  const Mat& rgb,
  const bool isRaw,
  const string& ispConfigFile,
  const string& titleExtra,
  const string& outputDir,
  int& stepDebugImages);

// Computes white balance gains so that color response matches ground truth
Vec3f computeWhiteBalanceGains(const ColorResponse& colorResponse);

// Compute color correction matrix from patch info
Mat computeCCM(const vector<ColorPatch>& colorPatches);

// Compute L2 errors between corrected color patches and MacBeth ground truth
// The output is a pair of vectors such that
// Vector 1: errRGB, errB, errG, errR BEFORE correction
// Vector 2: errRGB, errB, errG, errR AFTER correction
pair<Vec4f, Vec4f> computeColorPatchErrors(
  const Mat& imBefore,
  const Mat& imAfter,
  const vector<ColorPatch>& colorPatches);

} // namespace color_calibration
} // namespace surround360
