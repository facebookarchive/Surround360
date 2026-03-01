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

// MacBeth patches from
// Danny Pascale, "RGB coordinates of the Macbeth ColorChecker", The BabelColor Company, June 2006
map<string, vector<vector<float> > > const labMacbeth {
  {"D50", {
    {37.99, 13.56, 14.06},
    {65.71, 18.13, 17.81},
    {49.93, -4.88, -21.93},
    {43.14, -13.10, 21.91},
    {55.11, 8.84, -25.40},
    {70.72, -33.40, -0.199},
    {62.66, 36.07, 57.10},
    {40.02, 10.41, -45.96},
    {51.12, 48.24, 16.25},
    {30.33, 22.98, -21.59},
    {72.53, -23.71, 57.26},
    {71.94, 19.36, 67.86},
    {28.78, 14.18, -50.30},
    {55.26, -38.34, 31.37},
    {42.10, 53.38, 28.19},
    {81.73, 4.04, 79.82},
    {51.94, 49.99, -14.57},
    {51.04, -28.63, -28.64},
    {96.54, -0.425, 1.186},
    {81.26, -0.638, -0.335},
    {66.77, -0.734, -0.504},
    {50.87, -0.153, -0.270},
    {35.66, -0.421, -1.231},
    {20.46, -0.079, -0.973}}},
  {"D65", {
    {37.85, 12.72, 14.07},
    {65.43, 17.18, 17.21},
    {50.15, -1.91, -21.79},
    {43.17, -15.08, 22.44},
    {55.40, 11.58, -25.06},
    {70.92, -33.22, 0.29},
    {62.06, 33.37, 56.24},
    {40.59, 16.15, -45.14},
    {50.58, 47.55, 15.17},
    {30.51, 25.11, -21.74},
    {72.31, -27.84, 57.83},
    {71.43, 15.50, 67.80},
    {29.46, 20.74, -49.34},
    {55.26, -41.23, 32.03},
    {41.53, 52.67, 26.92},
    {81.08, -0.33, 80.10},
    {51.74, 51.26, -15.48},
    {52.41, -18.46, -26.64},
    {96.49, -0.35, 0.96},
    {81.17, -0.69, -0.24},
    {66.84, -0.71, -0.25},
    {50.86, 0.20, -0.55},
    {35.61, -0.36, -1.44},
    {20.40, 0.47, -1.27}}
  }
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
  const int minNumPixels,
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
  const string& illuminant,
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
  const string& illuminant,
  const string& outputDir,
  const string& titleExtra);

} // namespace color_calibration
} // namespace surround360
