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
#include "CameraIsp.h"

#include "opencv2/imgproc.hpp"

namespace surround360 {
namespace color_calibration {

using namespace std;
using namespace cv;

struct ColorPatch {
  Point2f centroid;
  Mat mask;
  Vec3f rgbMedian;
  Vec3f labMedian;
};

struct ColorResponse {
  Vec3f rgbInterceptXMax;
  Vec3f rgbInterceptXMin;
  Vec3f rgbInterceptY;
  Vec3f rgbSlope;
};

// Reference grayscale values for plotting purposes
const vector<int> rgbGrayLinearMacbeth = {6, 21, 49, 92, 150, 233};

// MacBeth patches from X-Rite website
// http://xritephoto.com/ph_product_overview.aspx?ID=938&Action=Support&SupportID=5884
const vector<vector<float>> labMacbeth = {
  {37.54, 14.37, 14.92},
  {64.66, 19.27, 17.5},
  {49.32, -3.82, -22.54},
  {43.46, -12.74, 22.72},
  {54.94, 9.61, -24.79},
  {70.48, -32.26, -0.37},
  {62.73, 35.83, 56.5},
  {39.43, 10.75, -45.17},
  {50.57, 48.64, 16.67},
  {30.1, 22.54, -20.87},
  {71.77, -24.13, 58.19},
  {71.51, 18.24, 67.37},
  {28.37, 15.42, -49.8},
  {54.38, -39.72, 32.27},
  {42.43, 51.05, 28.62},
  {81.8, 2.67, 80.41},
  {50.63, 51.28, -14.12},
  {49.57, -29.71, -28.32},
  {95.19, -1.03, 2.93},
  {81.29, -0.57, 0.44},
  {66.89, -0.75, -0.06},
  {50.76, -0.13, 0.14},
  {35.63, -0.46, -0.48},
  {20.64, 0.07, -0.46}
};

// Get image bit depth
int getBitsPerPixel(const Mat& image);

// Loads the given JSON file into a usable string
string getJson(const string& filename);

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

// Generates ISP config file with all the given parameters
void writeIspConfigFile(
  const string& ispConfigFileOut,
  CameraIsp& cameraIsp,
  const Vec3f& blackLevel,
  const Vec3f& whiteBalanceGain,
  const Mat& ccm,
  const Vec3f& gamma);

// Updates input ISP config file with clamp values
void updateIspWithClamps(
  const string& ispConfigFilePath,
  const int bpp,
  const Vec3f& clampMin,
  const Vec3f& clampMax);

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
  const float elementSize,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Dilates gaps to avoid outliers on contour detection
Mat dilateGaps(
  const Mat& imageBw,
  const float elementSize,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages);

// Creates structuring element of given shape for morphological operations
Mat createMorphElement(
  const Size imageSize,
  const float elementSize,
  const int shape);

// Removes small objects
Mat removeSmallObjects(
  const Mat& imageBw,
  const float smallestObjectSize,
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

// Calculates black level, white balance and CCM from given color patches
void obtainIspParams(
  vector<ColorPatch>& colorPatches,
  const Size& imageSize,
  const bool isBlackLevelSet,
  const bool saveDebugImages,
  const string& outputDir,
  int& stepDebugImages,
  Vec3f& blackLevel,
  Vec3f& whiteBalance,
  Mat& ccm);

// Compute DeltaE errors between corrected color patches and MacBeth ground
// truth (Lab)
void computeColorPatchErrors(
  const vector<ColorPatch>& colorPatches,
  const string& outputDir,
  const string& titleExtra);

} // namespace color_calibration
} // namespace surround360
