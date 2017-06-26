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
    {37.99f, 13.56f, 14.06f},
    {65.71f, 18.13f, 17.81f},
    {49.93f, -4.88f, -21.93f},
    {43.14f, -13.10f, 21.91f},
    {55.11f, 8.84f, -25.40f},
    {70.72f, -33.40f, -0.199f},
    {62.66f, 36.07f, 57.10f},
    {40.02f, 10.41f, -45.96f},
    {51.12f, 48.24f, 16.25f},
    {30.33f, 22.98f, -21.59f},
    {72.53f, -23.71f, 57.26f},
    {71.94f, 19.36f, 67.86f},
    {28.78f, 14.18f, -50.30f},
    {55.26f, -38.34f, 31.37f},
    {42.10f, 53.38f, 28.19f},
    {81.73f, 4.04f, 79.82f},
    {51.94f, 49.99f, -14.57f},
    {51.04f, -28.63f, -28.64f},
    {96.54f, -0.425f, 1.186f},
    {81.26f, -0.638f, -0.335f},
    {66.77f, -0.734f, -0.504f},
    {50.87f, -0.153f, -0.270f},
    {35.66f, -0.421f, -1.231f},
    {20.46f, -0.079f, -0.973f}}},
  {"D65", {
    {37.85f, 12.72f, 14.07f},
    {65.43f, 17.18f, 17.21f},
    {50.15f, -1.91f, -21.79f},
    {43.17f, -15.08f, 22.44f},
    {55.40f, 11.58f, -25.06f},
    {70.92f, -33.22f, 0.29f},
    {62.06f, 33.37f, 56.24f},
    {40.59f, 16.15f, -45.14f},
    {50.58f, 47.55f, 15.17f},
    {30.51f, 25.11f, -21.74f},
    {72.31f, -27.84f, 57.83f},
    {71.43f, 15.50f, 67.80f},
    {29.46f, 20.74f, -49.34f},
    {55.26f, -41.23f, 32.03f},
    {41.53f, 52.67f, 26.92f},
    {81.08f, -0.33f, 80.10f},
    {51.74f, 51.26f, -15.48f},
    {52.41f, -18.46f, -26.64f},
    {96.49f, -0.35f, 0.96f},
    {81.17f, -0.69f, -0.24f},
    {66.84f, -0.71f, -0.25f},
    {50.86f, 0.20f, -0.55f},
    {35.61f, -0.36f, -1.44f},
    {20.40f, 0.47f, -1.27f}}
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
