/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "ColorCalibration.h"

#include <iomanip>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "ColorspaceConversion.h"
#include "CvUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include "ceres/ceres.h"

namespace surround360 {
namespace color_calibration {

using namespace std;
using namespace cv;
using namespace util;

vector<double> applyColorParams(
    const vector<double>& rgb,
    const string& illuminant,
    const double illumScale,
    const double* const bl,
    const double* const wbAndCcm) {

  // Black level
  vector<double> rgbBL(3, 0.0);
  const int numChannels = rgbBL.size();
  for (int i = 0; i < numChannels; ++i) {
    const double eps = std::numeric_limits<double>::epsilon();
    rgbBL[i] = (rgb[i] - bl[i]) / (1.0 - bl[i] + eps);
    rgbBL[i] *= illumScale;
  }

  // WB * CCM * rgb
  vector<double> rgbWbAndCcm(numChannels, 0.0);
  for (int x = 0; x < numChannels; ++x) {
    for (int y = 0; y < numChannels; ++y) {
      rgbWbAndCcm[x] += wbAndCcm[x * numChannels + y] * rgbBL[y];
    }
  }

  vector<double> lab(numChannels, 0.0);
  toLab(
    illuminant,
    rgbWbAndCcm[0],
    rgbWbAndCcm[1],
    rgbWbAndCcm[2],
    lab[0],
    lab[1],
    lab[2]);
  return lab;
}

// We want to minimize
// sum_i=1^N ||LABgt_i - LAB(M * (s_i * RGBin_i - BL) / (1 - BL))||^2
// where
// N: number of color patches
// LABgt_i: lab ground truth for i-th color patch
// M: 3x3 matrix (can be interpreted as WB * CCM)
// s_i: non-uniform illumination on i-th patch
// RGBin_i: RGB value of color patch (input)
// BL: black level
// To force the non-uniform illumination to be a smooth surface, we treat the
// color chart as a Bezier surface
template<int TBezierX, int TBezierY>
struct IspFunctor {
  static ceres::CostFunction* addResidual(
      ceres::Problem& problem,
      vector<double>& bezierX,
      vector<double>& bezierY,
      vector<double>& bl,
      vector<double>& wbAndCcm,
      const double x,
      const double y,
      const vector<double> rgb,
      const string illuminant,
      const vector<double> labRef) {

    CHECK_EQ(TBezierX + 1, bezierX.size());
    CHECK_EQ(TBezierY + 1, bezierY.size());

    auto* cost = new CostFunction(new IspFunctor(x, y, rgb, illuminant, labRef));
    problem.AddResidualBlock(
      cost,
      nullptr,  // loss
      bezierX.data(),
      bezierY.data(),
      bl.data(),
      wbAndCcm.data());
    return cost;
  }

  bool operator()(
      const double* const bezierX,
      const double* const bezierY,
      const double* const bl,
      const double* const wbAndCcm,
      double* residuals) const {

    BezierCurve<float, double> bX;
    BezierCurve<float, double> bY;

    for (int i = 0; i <= TBezierX; ++i) {
      bX.addPoint(bezierX[i]);
    }
    for (int i = 0; i <= TBezierY; ++i) {
      bY.addPoint(bezierY[i]);
    }

    const double illumScale = bX(x) * bY(y);
    const vector<double> lab =
      applyColorParams(rgb, illuminant, illumScale, bl, wbAndCcm);

    for (int i = 0; i < lab.size(); ++i) {
      residuals[i] = labRef[i] - lab[i];
    }

    CHECK(std::isfinite(*residuals));
    return true;
  }

 private:
  using CostFunction = ceres::NumericDiffCostFunction<
    IspFunctor,
    ceres::CENTRAL,
    3,  // residuals
    TBezierX + 1,  // bezier X
    TBezierY + 1,  // bezier Y
    3,  // bl
    9>;  // wb and ccm

  IspFunctor(
      const double x,
      const double y,
      const vector<double> rgb,
      const string& illuminant,
      const vector<double> labRef) :

    x(x),
    y(y),
    rgb(rgb),
    illuminant(illuminant),
    labRef(labRef) {
  }

  const double x;
  const double y;
  const vector<double> rgb;
  const string illuminant;
  const vector<double> labRef;
};

int getBitsPerPixel(const Mat& image) {
  uint8_t depth = image.type() & CV_MAT_DEPTH_MASK;
  return depth == CV_8U ? 8 : 16;
}

string getJson(const string& filename) {
  ifstream ifs(filename, ios::in);
  if (!ifs) {
    throw VrCamException("file read failed: " + filename);
  }

  string json(
    (istreambuf_iterator<char>(ifs)),
    istreambuf_iterator<char>());

  return json;
}

Mat getRaw(const string& ispConfigFile, const Mat& image) {
  CameraIsp cameraIsp(getJson(ispConfigFile), getBitsPerPixel(image));
  cameraIsp.loadImage(image);
  return cameraIsp.getRawImage();
}

Mat findClampedPixels(const Mat& image8) {
  Mat clamped(image8.size(), image8.type(), Scalar::all(128));
  for (int y = 0; y < image8.rows; ++y) {
    for (int x = 0; x < image8.cols; ++x) {
      const int pixelVal = image8.at<uchar>(y, x);
      if (pixelVal == 0 || pixelVal == 255) {
        clamped.at<uchar>(y, x) = pixelVal;
      }
    }
  }
  return clamped;
}

ColorResponse computeRGBResponse(
    const Mat& raw,
    const bool isRaw,
    vector<ColorPatch>& colorPatches,
    const string& ispConfigFile,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages,
    const string& titleExtra) {

  ColorResponse colorResponse;
  Vec3f rgbSlope = Vec3f(0.0f, 0.0f, 0.0f);
  Vec3f rgbInterceptY = Vec3f(0.0f, 0.0f, 0.0f);
  Vec3f rgbInterceptXMin = Vec3f(0.0f, 0.0f, 0.0f);
  Vec3f rgbInterceptXMax = Vec3f(0.0f, 0.0f, 0.0f);

  // Get RGB medians in raw image
  computeRGBMedians(colorPatches, raw, isRaw, ispConfigFile);

  // const vector<int> macBethGrayValues = getMacBethGrays();
  const int iStart = colorPatches.size() - 1;

  // Line between second darkest and second brightest medians
  static const int kBrightIdx = 4;
  static const int kDarkIdx = 1;
  const float xDark = float(rgbGrayLinearMacbeth[kDarkIdx]) / 255.0f;
  const float xBright = float(rgbGrayLinearMacbeth[kBrightIdx]) / 255.0f;
  const Vec3f& yDark = colorPatches[iStart - kDarkIdx].rgbMedian;
  const Vec3f& yBright = colorPatches[iStart - kBrightIdx].rgbMedian;

  // Each channel response is of the form y = mx + b
  static const int kNumChannels = 3;
  for (int ch = 0; ch < kNumChannels; ++ch) {
    rgbSlope[ch] = (yBright[ch] - yDark[ch]) / (xBright - xDark);
    rgbInterceptY[ch] = -rgbSlope[ch] * xDark + yDark[ch];
    rgbInterceptXMin[ch] = -rgbInterceptY[ch] / rgbSlope[ch];
    rgbInterceptXMax[ch] = (1.0 - rgbInterceptY[ch]) / rgbSlope[ch];
  }

  colorResponse.rgbSlope = rgbSlope;
  colorResponse.rgbInterceptY = rgbInterceptY;
  colorResponse.rgbInterceptXMin = rgbInterceptXMin;
  colorResponse.rgbInterceptXMax = rgbInterceptXMax;

  if (saveDebugImages) {
    plotGrayPatchResponse(
      colorPatches,
      raw,
      isRaw,
      ispConfigFile,
      titleExtra,
      outputDir,
      stepDebugImages);
  }

  return colorResponse;
}

void saveBlackLevel(const Vec3f& blackLevel, const string& outputDir) {
  const string blackLevelFilename = outputDir + "/black_level.txt";
  ofstream blackLevelStream(blackLevelFilename, ios::out);

  if (!blackLevelStream) {
    throw VrCamException("file open failed: " + blackLevelFilename);
  }

  blackLevelStream << blackLevel;
  blackLevelStream.close();
}

void saveXIntercepts(
    const ColorResponse& colorResponse,
    const string& outputDir) {

  const string interceptXFilename = outputDir + "/intercept_x.txt";
  ofstream interceptXStream(interceptXFilename, ios::out);

  if (!interceptXStream) {
    throw VrCamException("file open failed: " + interceptXFilename);
  }

  interceptXStream << "[";
  interceptXStream << colorResponse.rgbInterceptXMin;
  interceptXStream << ",";
  interceptXStream << colorResponse.rgbInterceptXMax;
  interceptXStream << "]";
  interceptXStream.close();
}

void writeIspConfigFile(
    const string& ispConfigFileOut,
    CameraIsp& cameraIsp,
    const Vec3f& blackLevel,
    const Vec3f& whiteBalanceGain,
    const Mat& ccm,
    const Vec3f& gamma) {

  cameraIsp.setBlackLevel(blackLevel);
  cameraIsp.setWhiteBalance(whiteBalanceGain);
  cameraIsp.setCCM(ccm);
  cameraIsp.setGamma(gamma);
  cameraIsp.setup();
  cameraIsp.dumpConfigFile(ispConfigFileOut);
}

void updateIspWithClamps(
    const string& ispConfigFilePath,
    const int bpp,
    const Vec3f& rgbClampMin,
    const Vec3f& rgbClampMax) {

  CameraIsp cameraIsp(getJson(ispConfigFilePath), bpp);
  cameraIsp.setClampMin(rgbClampMin);
  cameraIsp.setClampMax(rgbClampMax);
  cameraIsp.setup();
  cameraIsp.dumpConfigFile(ispConfigFilePath);
}

Vec3f findBlackLevel(
    const Mat& raw16,
    const int minNumPixels,
    const string& ispConfigFile,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  // Divide raw into R, G and B channels
  const int bitsPerPixel = getBitsPerPixel(raw16);
  const int maxPixelValue = (1 << bitsPerPixel) - 1;
  static const int kNumChannels = 3;
  vector<Mat> RGBs;
  for (int i = 0; i < kNumChannels; ++i) {
    // Initialize channel to max value. Unused pixels will be at the high end of
    // the histogram, which will not be reached
    RGBs.push_back(Mat(raw16.size(), CV_32F, Scalar(maxPixelValue)));
  }

  CameraIsp cameraIsp(getJson(ispConfigFile), bitsPerPixel);
  for (int i = 0; i < raw16.rows; ++i) {
    for (int j = 0; j < raw16.cols; j++) {
      const int channelIdx =
        cameraIsp.redPixel(i, j) ? 0 : (cameraIsp.greenPixel(i, j) ? 1 : 2);
      RGBs[channelIdx].at<float>(i, j) = raw16.at<uint16_t>(i, j);
    }
  }

  // Calculate per channel histograms
  Mat blackHoleMask = Mat::zeros(raw16.size(), CV_8UC1);
  static const int kNumPixelsMin = minNumPixels / kNumChannels;
  double blackLevelThreshold = 0.0;
  for (int i = 0; i < kNumChannels; ++i) {
    // Black level threshold is the lowest non-zero value with enough pixel
    // count (to avoid noise and dead pixels)
    Mat hist = computeHistogram(RGBs[i], Mat());
    int countChannel = 0;
    for (int h = 1; h < maxPixelValue; ++h) {
      countChannel += hist.at<float>(h);
      if (countChannel > kNumPixelsMin) {
        blackLevelThreshold = h;
        break;
      }
    }

    // Create mask with all pixels below threshold
    Mat mask(RGBs[i].size(), RGBs[i].type(), Scalar::all(0));
    inRange(RGBs[i], 0.0f, blackLevelThreshold, mask);
    blackHoleMask = (blackHoleMask | mask);
  }

  // Dilate black hole mask to help contour detection
  static const int kDilationSize = 5;
  Mat element = getStructuringElement(
    MORPH_RECT,
    Size(2 * kDilationSize + 1, 2 * kDilationSize + 1),
    Point2f(kDilationSize, kDilationSize));
  dilate(blackHoleMask, blackHoleMask, element);

  if (saveDebugImages) {
    const string blackHoleDilatedImageFilename = outputDir + "/" +
      to_string(++stepDebugImages) + "_black_hole_mask_dilated.png";
    imwriteExceptionOnFail(blackHoleDilatedImageFilename, 255.0f * blackHoleMask);
  }

  // Black hole mask can contain outliers and pixels outside black hole. We need
  // to filter it
  vector<vector<Point>> contours;
  static const float kStraightenFactor = 0.01f;
  contours = findContours(
    blackHoleMask, false, outputDir, stepDebugImages, kStraightenFactor);

  // Filter contours
  vector<vector<Point>> contoursFiltered;
  vector<Point2f> circleCenters;
  vector<float> circleRadii;
  for (int i = 0; i < contours.size(); ++i) {
    vector<Point2i> cont = contours[i];
    const int contArea = contourArea(cont);

    Point2f circleCenter;
    float circleRadius;
    minEnclosingCircle(cont, circleCenter, circleRadius);
    const float circleArea = M_PI * circleRadius * circleRadius;

    // Discard contours that are too small and non-circular
    static const int kMinNumVertices = 10;
    static const float kMaxRatioAreas = 0.3f;
    if (contArea < kNumPixelsMin ||
        cont.size() < kMinNumVertices ||
        contArea / circleArea < kMaxRatioAreas) {
      continue;
    }

    circleCenters.push_back(circleCenter);
    circleRadii.push_back(circleRadius);
    contoursFiltered.push_back(contours[i]);
  }

  if (contoursFiltered.size() == 0) {
    throw VrCamException("Cannot find black hole!");
  }

  if (saveDebugImages) {
    Mat contoursPlot = Mat::zeros(blackHoleMask.size(), CV_8UC3);
    RNG rng(12345);
    for (int i = 0; i < contoursFiltered.size(); ++i) {
      Scalar color =
        Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      drawContours(contoursPlot, contoursFiltered, i, color);
      circle(contoursPlot, circleCenters[i], (int)circleRadii[i], color);
    }
    const string contoursImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_contours_filtered.png";
    imwriteExceptionOnFail(contoursImageFilename, contoursPlot);
  }

  // Get RGB median of each filtered contour
  vector<Mat> blackHoleMasks(contoursFiltered.size());
  vector<Vec3f> blackLevels(contoursFiltered.size());
  double minNorm = DBL_MAX;
  int minNormIdx = 0;
  for (int i = 0; i < contoursFiltered.size(); ++i) {
    // Create contour mask
    blackHoleMasks[i] = Mat::zeros(blackHoleMask.size(), CV_8UC1);
    drawContours(blackHoleMasks[i], contoursFiltered, i, Scalar(255), CV_FILLED);

    Mat rawNormalized = getRaw(ispConfigFile, raw16);
    static const bool kIsRaw = true;
    blackLevels[i] =
      getRgbMedianMask(rawNormalized, blackHoleMasks[i], ispConfigFile, kIsRaw);

    // Find distance to [0, 0, 0]
    double blackLevelNorm = norm(blackLevels[i]);
    if (blackLevelNorm < minNorm) {
      minNorm = blackLevelNorm;
      minNormIdx = i;
    }
  }

  // Black level is the one closest to origin
  Vec3f blackLevel = blackLevels[minNormIdx];

  if (saveDebugImages) {
    Mat rawRGB(raw16.size(), CV_8UC3);
    cvtColor(raw16, rawRGB, CV_GRAY2RGB);
    blackHoleMask = blackHoleMasks[minNormIdx];
    rawRGB.setTo(Scalar(0, maxPixelValue, 0), blackHoleMask);

    const string blackHoleMaskImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_black_hole.png";
    imwriteExceptionOnFail(blackHoleMaskImageFilename, rawRGB);
  }

  const Vec3f blackLevelScaled = blackLevel * maxPixelValue;
  LOG(INFO) << "Black level (" << bitsPerPixel << "-bit): " << blackLevelScaled;

  return blackLevel;
}

Mat computeHistogram(const Mat& image, const Mat& mask) {
  static const int kNumImages = 1;
  static const int* kChannelsAuto = 0;
  static const int kNumDims = 1;
  const int bitsPerPixel = getBitsPerPixel(image);
  const float maxPixelValue = float((1 << bitsPerPixel) - 1);
  const int histSize = maxPixelValue + 1;
  float range[] = {0, maxPixelValue};
  const float *ranges[] = {range};
  Mat hist;
  calcHist(
    &image,
    kNumImages,
    kChannelsAuto,
    mask,
    hist,
    kNumDims,
    &histSize,
    ranges);
  return hist;
}

vector<ColorPatch> detectColorChart(
    const Mat& image,
    const int numSquaresW,
    const int numSquaresH,
    const float minAreaChart,
    const float maxAreaChart,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  // Scale image to make patches brighter
  static const float kScale = 2.0f;
  Mat imageScaled = kScale * image;

  // Smooth image
  Mat imageBlur;
  static const Size kBlurSize = Size(15, 15);
  static const double kSigmaAuto = 0;
  GaussianBlur(imageScaled, imageBlur, kBlurSize, kSigmaAuto);

  if (saveDebugImages) {
    const string blurImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_scaled_blurred.png";
    imwriteExceptionOnFail(blurImageFilename, imageBlur);
  }

  // Adaptive thresholding
  Mat bw;
  static const double kMaxValue = 255.0;
  static const int kBlockSize = 19;
  static const int kWeightedSub = 2;
  adaptiveThreshold(
    imageBlur,
    bw,
    kMaxValue,
    ADAPTIVE_THRESH_MEAN_C,
    THRESH_BINARY_INV,
    kBlockSize,
    kWeightedSub);

  if (saveDebugImages) {
    const string adaptiveThreshImageFilename =
      outputDir + "/" + to_string(++stepDebugImages)
      + "_adaptive_threshold.png";
    imwriteExceptionOnFail(adaptiveThreshImageFilename, bw);
  }

  const int numPatches = numSquaresW * numSquaresH;
  const float minAreaPatch = minAreaChart / numPatches;
  const float maxAreaPatch = maxAreaChart / numPatches;
  const float bwArea = bw.rows * bw.cols;
  static const float kScaleElement = 10.0f;
  const float morphElementSize = kScaleElement * minAreaPatch / bwArea;

  // Morphological closing to reattach patches
  bw = fillGaps(bw, morphElementSize, saveDebugImages, outputDir, stepDebugImages);

  // Remove small objects
  static const float kScaleSmallestObject = 0.3f;
  const float smallestObjectSize = kScaleSmallestObject * minAreaPatch;
  bw = removeSmallObjects(bw, smallestObjectSize, saveDebugImages, outputDir, stepDebugImages);

  // Dilate gaps so contours don't contain pixels outside patch
  bw = dilateGaps(bw, morphElementSize, saveDebugImages, outputDir, stepDebugImages);

  // Connected components
  Mat labels;
  Mat stats;
  Mat centroids;
  int la = connectedComponentsWithStats(bw, labels, stats, centroids, 8);

  // Filter components
  vector<vector<Point>> contours;
  Mat bwLabel(bw.size(), CV_8UC1, Scalar::all(0));
  const Point center = Point(bw.cols / 2, bw.rows / 2);
  for (int label = 1; label < la; ++label) {
    const int numPixels = stats.at<int>(label, CC_STAT_AREA);
    const int top = stats.at<int>(label, CC_STAT_TOP);
    const int left = stats.at<int>(label, CC_STAT_LEFT);
    const int width = stats.at<int>(label, CC_STAT_WIDTH);
    const int height = stats.at<int>(label, CC_STAT_HEIGHT);

    // Assuming chart doesn't take too much of the image
    if (numPixels < minAreaChart || width * height > maxAreaChart) {
      continue;
    }

    // Get contours for current label
    bwLabel.setTo(Scalar::all(0));
    inRange(labels, label, label, bwLabel);
    static const float kStraightenFactor = 0.08f;
    vector<vector<Point>> contoursI = findContours(
      bwLabel, saveDebugImages, outputDir, stepDebugImages, kStraightenFactor);

    // Check if we have at least as many contours as number of patches
    // (+1 to account for chart border)
    if (contoursI.size() >= numPatches + 1) {
      for (const auto& contour : contoursI) {
        contours.push_back(contour);
      }
    }
  }

  vector<ColorPatch> colorPatchList;

  // Morphological constraints for patch filtering
  static const float kMaxAspectRatio = 2.0f;
  static const int kNumEdges = 4;

  // Filter contours
  int countPatches = 0;
  for (int i = 0; i < contours.size(); ++i) {
    vector<Point2i> cont = contours[i];
    RotatedRect boundingBox = minAreaRect(cont);
    Moments mu = moments(cont, false);

    Point2f centroid = boundingBox.center;

    const int width = boundingBox.size.width;
    const int height = boundingBox.size.height;
    const int area = mu.m00;
    const int aspectRatio =
      1.0f * std::max(width, height) / (1.0f * std::min(width, height));

    // Discard contours that are too small/large, non-square and non-convex
    if (area < minAreaPatch || area > maxAreaPatch ||
        cont.size() != kNumEdges ||
        aspectRatio > kMaxAspectRatio ||
        !isContourConvex(cont)) {
      continue;
    }

    LOG(INFO) << "Patch found (" << countPatches++ << ")!";

    // Create patch mask
    Mat patchMask(bw.size(), CV_8UC1, Scalar::all(0));
    drawContours(patchMask, contours, i, 255, CV_FILLED);

    // Add patch to list
    ColorPatch colorPatch;
    colorPatch.centroid = centroid;
    colorPatch.mask = patchMask;

    colorPatchList.push_back(colorPatch);
  }

  if (colorPatchList.size() == 0) {
    return colorPatchList;
  }

  vector<ColorPatch> colorPatchListClean =
    removeContourOutliers(colorPatchList);

  vector<ColorPatch> colorPatchListSorted =
    sortPatches(colorPatchListClean, numSquaresW, image.size());

  LOG(INFO) << "Number of patches found: " << colorPatchListSorted.size();

  if (saveDebugImages) {
    Mat rgbDraw = image;
    cvtColor(rgbDraw, rgbDraw, CV_GRAY2RGB);
    rgbDraw = drawPatches(rgbDraw, colorPatchListSorted);
    const string patchesImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_detected_patches.png";
    imwriteExceptionOnFail(patchesImageFilename, rgbDraw);
  }

  return colorPatchListSorted;
}

Mat fillGaps(
    const Mat& imageBwIn,
    const float elementSize,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  Mat imageBwOut;
  Mat element = createMorphElement(imageBwIn.size(), elementSize, MORPH_CROSS);
  morphologyEx(imageBwIn, imageBwOut, MORPH_CLOSE, element);

  if (saveDebugImages) {
    const string morphImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_fill_gaps.png";
    imwriteExceptionOnFail(morphImageFilename, imageBwOut);
  }

  return imageBwOut;
}

Mat dilateGaps(
    const Mat& imageBwIn,
    const float elementSize,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  Mat imageBwOut;
  Mat element = createMorphElement(imageBwIn.size(), elementSize, MORPH_RECT);
  dilate(imageBwIn, imageBwOut, element);

  if (saveDebugImages) {
    const string morphImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_dilate.png";
    imwriteExceptionOnFail(morphImageFilename, imageBwOut);
  }

  return imageBwOut;
}

Mat createMorphElement(
    const Size imageSize,
    const float elementSize,
    const int shape) {

  const int morphRadius =
    elementSize * std::min(imageSize.width, imageSize.height);
  Size morphSize = Size(2 * morphRadius + 1, 2 * morphRadius + 1);
  return getStructuringElement(
    shape,
    morphSize,
    Point2f(morphRadius, morphRadius));
}

Mat removeSmallObjects(
    const Mat& imageBwIn,
    const float smallestObjectSize,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  Mat labels;
  Mat stats;
  Mat centroids;
  std::set<int> labelsSmall;
  int numConnectedComponents =
    connectedComponentsWithStats(imageBwIn, labels, stats, centroids);

  for (int label = 0; label < numConnectedComponents; ++label) {
    if (stats.at<int>(label, CC_STAT_AREA) < smallestObjectSize) {
      labelsSmall.insert(label);
    }
  }

  Mat imageBwOut = imageBwIn.clone();
  for (int y = 0; y < imageBwIn.rows; ++y) {
    for (int x = 0; x < imageBwIn.cols; ++x) {
      const int v = labels.at<int>(y, x);
      if (labelsSmall.find(v) != labelsSmall.end()) {
        imageBwOut.at<uchar>(y, x) = 0;
      }
    }
  }

  if (saveDebugImages) {
    const string morphImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_no_small_objects.png";
    imwriteExceptionOnFail(morphImageFilename, imageBwOut);
  }

  return imageBwOut;
}

vector<vector<Point>> findContours(
    const Mat& image,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages,
    const float straightenFactor) {

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  static const Point kOffset = Point(0, 0);
  findContours(
    image,
    contours,
    hierarchy,
    CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE,
    kOffset);

  // Straighten contours to minimize number of vertices
  for (int i = 0; i < contours.size(); ++i) {
    const double epsilonPolyDP =
      straightenFactor * arcLength(contours[i], true);
    approxPolyDP(contours[i], contours[i], epsilonPolyDP, true);
  }

  if (saveDebugImages) {
    Mat contoursPlot = Mat::zeros(image.size(), CV_8UC3);
    RNG rng(12345);
    for (int i = 0; i < contours.size(); ++i) {
      Scalar color =
        Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      drawContours(contoursPlot, contours, i, color);
    }
    const string contoursImageFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_contours.png";
    imwriteExceptionOnFail(contoursImageFilename, contoursPlot);
  }

  return contours;
}

vector<ColorPatch> removeContourOutliers(vector<ColorPatch> colorPatchList) {
  // Store min distance between patches, for each patch
  vector<float> minDistances(colorPatchList.size(), FLT_MAX);
  for (int iPatch = 0; iPatch < colorPatchList.size(); ++iPatch) {
    for (int jPatch = 0; jPatch < colorPatchList.size(); ++jPatch) {
      if (iPatch == jPatch) {
        continue;
      }
      const Point2f ci = colorPatchList[iPatch].centroid;
      const Point2f cj = colorPatchList[jPatch].centroid;
      const float distance = norm(ci - cj);
      if (distance < minDistances[iPatch]) {
        minDistances[iPatch] = distance;
      }
    }
  }

  // Find median minimum distance between patches
  vector<float> minDistancesSorted = minDistances;
  sort(minDistancesSorted.begin(), minDistancesSorted.end());
  float minDistanceMedian = minDistancesSorted[minDistancesSorted.size() / 2];

  // Discard patches with too large of a minimum distance
  const float maxDistanceThreshold = 2.0f * minDistanceMedian;
  vector<ColorPatch> colorPatchListClean;
  for (int iPatch = 0; iPatch < colorPatchList.size(); ++iPatch) {
    if (minDistances[iPatch] < maxDistanceThreshold) {
      colorPatchListClean.push_back(colorPatchList[iPatch]);
    }
  }

  return colorPatchListClean;
}

vector<ColorPatch> sortPatches(
    const vector<ColorPatch>& colorPatchList,
    const int numSquaresW,
    const Size imageSize) {

  const Point2f topLeftRef = Point2f(0.0f, 0.0f);
  const Point2f topRightRef = Point2f(imageSize.width, 0.0f);
  Point2f topLeft = Point2f(FLT_MAX, FLT_MAX);
  Point2f topRight = Point2f(-1.0f, FLT_MAX);

  // Assuming top left is (0, 0)
  vector<Point2f> centroids;
  for (ColorPatch patch : colorPatchList) {
    Point2f centroid = patch.centroid;
    centroids.push_back(centroid);

    if (norm(topLeftRef - centroid) < norm(topLeftRef - topLeft)) {
      topLeft = centroid;
    }
    if (norm(topRightRef - centroid) < norm(topRightRef - topRight)) {
      topRight = centroid;
    }
  }

  vector<Point2f> centroidsSorted;

  while (centroids.size() > 0) {
    // Get points in current row, i.e. closest to line between top-left and
    // top-right patches
    vector<Point2f> centroidsDistances;
    const Point2f pLine1 = findTopLeft(centroids);
    const Point2f pLine2 = findTopRight(centroids, imageSize.width);

    // Sort centroids by their distance to the line
    sort(centroids.begin(), centroids.end(),
      [pLine1, pLine2] (const Point2f& p1, const Point2f& p2) {
        float d1 = pointToLineDistance(p1, pLine1, pLine2);
        float d2 = pointToLineDistance(p2, pLine1, pLine2);
        return d1 < d2;
      });

    // Get top numSquaresW
    vector<Point2f> centroidsRow;
    for (int i = 0; i < numSquaresW && i < centroids.size(); ++i) {
      centroidsRow.push_back(centroids[i]);
    }

    // Sort row by X coordinate
    sort(centroidsRow.begin(), centroidsRow.end(), [](Point pt1, Point pt2) {
      return pt1.x < pt2.x;
    });

    // Add row to parent vector
    for (Point2f& centroid : centroidsRow) {
      centroidsSorted.push_back(centroid);
    }

    // Remove row from vector
    centroids.erase(
      centroids.begin(),
      centroids.begin() + centroidsRow.size());
  }

  // Re-order vector
  vector<ColorPatch> colorPatchListSorted;
  for (const Point2f& c : centroidsSorted) {
    for (const ColorPatch cp : colorPatchList) {
      if (cp.centroid == c) {
        colorPatchListSorted.push_back(cp);
        break;
      }
    }
  }

  return colorPatchListSorted;
}

Point2f findTopLeft(const vector<Point2f>& points) {
  static const Point2f kTopLeftRef = Point2f(0.0f, 0.0f);
  Point2f topLeft = Point2f(FLT_MAX, FLT_MAX);
  for (const Point2f& p : points) {
    if (norm(kTopLeftRef - p) < norm(kTopLeftRef - topLeft)) {
      topLeft = p;
    }
  }
  return topLeft;
}

Point2f findTopRight(const vector<Point2f>& points, int imageWidth) {
  const Point2f topRightRef = Point2f(imageWidth, 0.0f);
  Point2f topRight = Point2f(-1.0f, FLT_MAX);
  for (const Point2f& p : points) {
    if (norm(topRightRef - p) < norm(topRightRef - topRight)) {
      topRight = p;
    }
  }
  return topRight;
}

float pointToLineDistance(
    const Point2f p,
    const Point2f pLine1,
    const Point2f pLine2) {

  // Numerator: height of the triangle defined by the three points
  // Denominator: distance between two points in line
  const float n1 = (pLine2.y - pLine1.y) * p.x;
  const float n2 = (pLine2.x - pLine1.x) * p.y;
  const float n3 = pLine2.x * pLine1.y;
  const float n4 = pLine2.y * pLine1.x;
  return fabs(n1 - n2 + n3 - n4) / norm(pLine1 - pLine2);
}

Mat drawPatches(const Mat& image, vector<ColorPatch>& colorPatches) {
  Mat imageDraw = image.clone();
  for (int i = 0; i < colorPatches.size(); ++i) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    static const Point kOffset = Point(0, 0);
    findContours(
      colorPatches[i].mask,
      contours,
      hierarchy,
      CV_RETR_TREE,
      CV_CHAIN_APPROX_SIMPLE,
      kOffset);
    static const int kContourIdx = 0;
    static const Scalar kColorG = Scalar(0, 255, 0);
    drawContours(imageDraw, contours, kContourIdx, kColorG);

    const Point2f center = colorPatches[i].centroid;
    static const double kTextFontScale = 0.4;
    const string text = to_string(i);
    putText(
      imageDraw,
      text,
      center,
      FONT_HERSHEY_SIMPLEX,
      kTextFontScale,
      kColorG);
  }

  return imageDraw;
}

void computeRGBMedians(
    vector<ColorPatch>& colorPatches,
    const Mat& image,
    const bool isRaw,
    const string& ispConfigFile) {

  for (int i = 0; i < colorPatches.size(); ++i) {
    colorPatches[i].rgbMedian =
      getRgbMedianMask(image, colorPatches[i].mask, ispConfigFile, isRaw);
    LOG(INFO) << "Patch " << i << " RGB median: " << colorPatches[i].rgbMedian;
  }
}

Vec3f getRgbMedianMask(
    const Mat& image,
    const Mat& mask,
    const string& ispConfigFile,
    const bool isRaw) {

  // Allocate space for mask values on each channel
  static const int kNumChannels = 3;
  vector<vector<float>> RGBs;
  for (int ch = 0; ch < kNumChannels; ++ch) {
    vector<float> channel;
    RGBs.push_back(channel);
  }

  // Populate color channels
  Mat locs;
  findNonZero(mask, locs);
  CameraIsp cameraIsp(getJson(ispConfigFile), getBitsPerPixel(image));
  for (int ip = 0; ip < locs.rows; ++ip) {
    Point p = locs.at<Point>(ip);

    if (isRaw) {
      const float valRaw = image.at<float>(p);
      const int channelRaw = cameraIsp.redPixel(p.y, p.x)
        ? 0 : (cameraIsp.greenPixel(p.y, p.x) ? 1 : 2);
      RGBs[channelRaw].push_back(valRaw);
    } else {
      const Vec3f& val = image.at<Vec3f>(p);
      for (int ch = 0; ch < kNumChannels; ++ch) {
        RGBs[ch].push_back(val[ch]);
      }
    }
  }

  // Use partial sort to get median
  Vec3f rgbMedian = Vec3f(-1.0f, -1.0f, -1.0f);
  for (int ch = 0; ch < kNumChannels; ++ch) {
    vector<float>::iterator itMedian = RGBs[ch].begin() + RGBs[ch].size() / 2;
    std::nth_element(RGBs[ch].begin(), itMedian, RGBs[ch].end());
    rgbMedian[ch] = RGBs[ch][RGBs[ch].size() / 2];
  }

  return rgbMedian;

}

Vec3f plotGrayPatchResponse(
    vector<ColorPatch>& colorPatches,
    const Mat& image,
    const bool isRaw,
    const string& ispConfigFile,
    const string& titleExtra,
    const string& outputDir,
    int& stepDebugImages) {

  static const Scalar kRgbColors[] =
    {Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0)};
  const int bitsPerPixel = getBitsPerPixel(image);
  const float maxPixelValue = float((1 << bitsPerPixel) - 1);
  static const int kScalePlot = 10;
  static const float maxScaled = 255.0f * kScalePlot;
  static const float maxRow = 1.5f * maxScaled;
  static const float maxCol = maxRow;

  Point2f textCenter;
  string text;
  const int textFont = FONT_HERSHEY_SIMPLEX;
  static const float textSize = kScalePlot * 0.2f;
  static const int kTextThickness = 3;

  // Get RGB medians
  LOG(INFO) << "RGB medians (" << titleExtra << ")...";
  vector<ColorPatch> colorPatchesPlot = colorPatches;
  computeRGBMedians(colorPatchesPlot, image, isRaw, ispConfigFile);

  CameraIsp cameraIsp(getJson(ispConfigFile), bitsPerPixel);
  // const vector<int> macBethGrayValues = getMacBethGrays();
  const int iStart = colorPatchesPlot.size() - 1;
  const Point2f pShift = Point2f(5.0f * kScalePlot, 0.0f);
  const Point2f pShiftText = Point2f(pShift.x, 0.0f);
  static const int kNumChannels = 3;
  Mat scatterImage(maxRow, maxCol, CV_8UC3, Scalar::all(255));

  static const int kNumGreyPatches = 5;
  static const int kRadiusCircle = 3;
  for (int i = iStart; i >= iStart - kNumGreyPatches; --i) {
    const float xCoord =
      maxScaled * float(rgbGrayLinearMacbeth[iStart - i]) / 255.0f;

    // Only consider pixels inside patch mask
    Mat locs;
    findNonZero(colorPatchesPlot[i].mask, locs);

    // Plot all values
    for (int ip = 0; ip < locs.rows; ++ip) {
      Point p = locs.at<Point>(ip);
      if (isRaw) {
        const float patchValRaw = maxScaled * image.at<float>(p);
        const Point2f center = Point2f(xCoord, maxRow - patchValRaw);
        const int colorIdx = cameraIsp.redPixel(p.y, p.x)
          ? 0 : (cameraIsp.greenPixel(p.y, p.x) ? 1 : 2);
        circle(scatterImage, center, kRadiusCircle, kRgbColors[colorIdx], -1);
      } else {
        const Vec3f& patchVal = maxScaled * image.at<Vec3f>(p);
        for (int ch = 0; ch < kNumChannels; ++ch) {
          const Point2f center = Point2f(xCoord, maxRow - patchVal[ch]);
          circle(scatterImage, center, kRadiusCircle, kRgbColors[ch], -1);
        }
      }
    }

    // Plot medians
    const Vec3f& median = maxScaled * colorPatchesPlot[i].rgbMedian;
    static const int kLineThickness = 3;
    for (int ch = 0; ch < kNumChannels; ++ch) {
      const Point2f center = Point2f(xCoord, maxRow - median[ch]);
      line(
        scatterImage,
        center - pShift,
        center + pShift,
        kRgbColors[ch],
        kLineThickness);

      textCenter = center + pShiftText;
      const float medianReal = maxPixelValue * colorPatchesPlot[i].rgbMedian[ch];
      std::ostringstream textStream;
      textStream << fixed << std::setprecision(2) << medianReal;
      text = textStream.str();
      putText(
        scatterImage,
        text,
        textCenter,
        textFont,
        textSize * 0.8,
        kRgbColors[ch],
        kTextThickness);
    }
  }

  // Line between second darkest and second brightest medians
  static const int kBrightIdx = 4;
  static const int kDarkIdx = 1;
  const float xDark = maxScaled * float(rgbGrayLinearMacbeth[kDarkIdx]) / 255.0f;
  const float xBright =
    maxScaled * float(rgbGrayLinearMacbeth[kBrightIdx]) / 255.0f;
  const Vec3f& yDark = maxScaled * colorPatchesPlot[iStart - kDarkIdx].rgbMedian;
  const Vec3f& yBright =
    maxScaled * colorPatchesPlot[iStart - kBrightIdx].rgbMedian;

  textCenter = Point2f(50.0f, 0.0f);
  Vec3f yIntercepts = Vec3f(-1.0f, -1.0f, -1.0f);
  static const int kLineThickness = 3;
  for (int j = 0; j < kNumChannels; ++j) {
    const float slope = (yBright[j] - yDark[j]) / (xBright - xDark);
    const float yIntercept = -slope * xDark + yDark[j];
    const float xIntercept = -yIntercept / slope;
    const Point2f centerDark = Point2f(0.0f, maxRow - yIntercept);

    yIntercepts[j] = yIntercept / maxScaled * maxPixelValue;

    std::ostringstream textYIntercept;
    textYIntercept << fixed << std::setprecision(2) << yIntercepts[j];
    std::ostringstream textXIntercept;
    textXIntercept << fixed << std::setprecision(2) <<
      xIntercept / maxScaled * maxPixelValue;

    std::ostringstream textSlope;
    textSlope << fixed << std::setprecision(3) << slope;

    textCenter.y = 100.0f * (j + 1);
    text =
      "xIntercept: " + textXIntercept.str() +
      ", yIntercept: " + textYIntercept.str() +
      ", slope: " + textSlope.str();
    putText(
      scatterImage,
      text,
      textCenter,
      textFont,
      textSize,
      kRgbColors[j],
      kTextThickness);

    LOG(INFO) << (j == 0 ? "R" : (j == 1 ? "G" : "B")) << ": " << text;

    const float dest = slope * maxCol + yIntercept;
    const Point2f centerBright = Point2f(maxCol, maxRow - dest);
    line(scatterImage, centerDark, centerBright, kRgbColors[j], kLineThickness);
  }

  const string plotGrayImageFilename =
    outputDir + "/" + to_string(++stepDebugImages) + "_gray_patches_" +
    titleExtra + ".png";
  imwriteExceptionOnFail(plotGrayImageFilename, scatterImage);

  return yIntercepts;
}

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
    Mat& ccm) {

  LOG(INFO) << "Illuminant: " << illuminant;

  ceres::Problem problem;

  // Initialize estimates
  static const int kNumChannels = 3;
  vector<double> bl(kNumChannels, 0.0);

  if (isBlackLevelSet) {
    for (int i = 0; i < kNumChannels; ++i) {
      bl[i] = blackLevel[i];
    }
  }
  vector<double> wbAndCcm(kNumChannels * kNumChannels, 0.0);
  for (int i = 0; i < kNumChannels; ++i) {
    wbAndCcm[i * (kNumChannels + 1)] = 1.0;
  }
  static const int kBezierOrderX = 4;
  static const int kBezierOrderY = 4;
  vector<double> bezierX(kBezierOrderX + 1, 1.0);
  vector<double> bezierY(kBezierOrderY + 1, 1.0);

  // Assuming raster scan order
  // Assuming color patch medians are [0..1]
  float xMin = FLT_MAX;
  float xMax = 0.0f;
  float yMin = FLT_MAX;
  float yMax = 0.0f;
  for (int i = 0; i < colorPatches.size(); ++i) {
    Point2f centroid = colorPatches[i].centroid;
    xMin = std::min(xMin, centroid.x);
    xMax = std::max(xMax, centroid.x);
    yMin = std::min(yMin, centroid.y);
    yMax = std::max(yMax, centroid.y);
  }
  const Point2f bezierTL = colorPatches[0].centroid;
  const float width = xMax - xMin;
  const float height = yMax - yMin;

  // Create residuals
  vector<vector<double>> rgbsRef(colorPatches.size());
  for (int i = 0; i < colorPatches.size(); ++i) {
    Vec3f patchRGB = colorPatches[i].rgbMedian;
    rgbsRef[i] = {patchRGB[0], patchRGB[1], patchRGB[2]};
    const vector<double> labRef(labMacbeth.at(illuminant)[i].begin(), labMacbeth.at(illuminant)[i].end());
    const Point2f centroid = colorPatches[i].centroid - bezierTL;
    IspFunctor<kBezierOrderX, kBezierOrderY>::addResidual(
      problem,
      bezierX,
      bezierY,
      bl,
      wbAndCcm,
      centroid.x / width,
      centroid.y / height,
      rgbsRef[i],
      illuminant,
      labRef);
  }

  // Lock black level if known
  if (isBlackLevelSet) {
    problem.SetParameterBlockConstant(&bl[0]);
  } else {
    // Set black level bounds
    for (int i = 0; i < kNumChannels; ++i) {
      problem.SetParameterLowerBound(&bl[0], i, 0.0);
      problem.SetParameterUpperBound(&bl[0], i, 1.0);
    }
  }

  // Lock first Bezier control points to 1 to give it less weight in the overall
  // solution
  std::vector<int> constantControlPoints;
  constantControlPoints.push_back(0);
  ceres::SubsetParameterization* subsetParameterizationX =
    new ceres::SubsetParameterization(bezierX.size(), constantControlPoints);
  ceres::SubsetParameterization* subsetParameterizationY =
    new ceres::SubsetParameterization(bezierY.size(), constantControlPoints);
  problem.SetParameterization(&bezierX[0], subsetParameterizationX);
  problem.SetParameterization(&bezierY[0], subsetParameterizationY);

  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = saveDebugImages;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport();

  blackLevel = Vec3f(bl[0], bl[1], bl[2]);

  LOG(INFO) << "Black level: " << blackLevel;

  Mat wbAndCcmMat(kNumChannels, kNumChannels, CV_32FC1);
  for (int y = 0; y < kNumChannels; ++y) {
    for (int x = 0; x < kNumChannels; ++x) {
      wbAndCcmMat.at<float>(y, x) = wbAndCcm[y * kNumChannels + x];
    }
  }

  LOG(INFO) << "WB and CCM: " << wbAndCcmMat;

  // M = CCM * WB
  // If GT is [1, 1, 1] and input is x = [x1, x2, x3], we want M * x = [a, a, a]
  // (= keep grays gray)
  // => x = M^-1 * [a, a, a]
  // => WB = diag(1 / N(x)), where N(xi) = xi / max(x)
  // (= scaling 2 channels to the most sensitive one)
  // => WB * x = [b, b, b]
  // CCM * WB * x = [a, a, a]
  // => CCM * [b, b, b] = [a, a, a]
  // => sum(CCM ith row) = a / b, for all i
  // => CCM /= sum(CCM row) so each row sums to one

  Mat balanced = (wbAndCcmMat.inv() * Mat::ones(3, 1, CV_32FC1)).t();

  double balancedMax;
  minMaxLoc(balanced, nullptr, &balancedMax);

  whiteBalance = Vec3f(
    balancedMax / balanced.at<float>(0),
    balancedMax / balanced.at<float>(1),
    balancedMax / balanced.at<float>(2));

  LOG(INFO) << "White balance: " << whiteBalance;

  // Divide each column by WB
  ccm = Mat::zeros(kNumChannels, kNumChannels, CV_32FC1);
  ccm.col(0) = wbAndCcmMat.col(0) / whiteBalance[0];
  ccm.col(1) = wbAndCcmMat.col(1) / whiteBalance[1];
  ccm.col(2) = wbAndCcmMat.col(2) / whiteBalance[2];

  // Force CCM rows to sum to 1
  ccm /= sum(ccm.row(0))[0];

  LOG(INFO) << "CCM: " << ccm;

  for (int i = 0; i <= kBezierOrderX; ++i) {
    LOG(INFO) << "bezierX[" << i << "]: " << bezierX[i];
  }
  for (int i = 0; i <= kBezierOrderY; ++i) {
    LOG(INFO) << "bezierY[" << i << "]: " << bezierY[i];
  }

  BezierCurve<float, double> bX(bezierX);
  BezierCurve<float, double> bY(bezierY);

  vector<float> illuminationScales(colorPatches.size());
  for (int i = 0; i < colorPatches.size(); ++i) {
    Point2f centroid = colorPatches[i].centroid - bezierTL;
    const float bezier = bX(centroid.x / width) * bY(centroid.y / height);
    illuminationScales[i] = bezier;
  }

  if (saveDebugImages) {
    // Plot inverse illumination scales
    Mat illum(imageSize, CV_32FC1, Scalar::all(1.0f));

    for (int i = 0; i < colorPatches.size(); ++i) {
      LOG(INFO) << "patch " << i << ": " << colorPatches[i].centroid
                << ", illum: " << illuminationScales[i];
      const float bezierStretched =
        illuminationScales[i] / illuminationScales[1];
      static const int kRadius = 30;
      static const int kThickness = -1;  // filled
      const Point center = colorPatches[i].centroid;
      circle(illum, center, kRadius, 1.0f / bezierStretched, kThickness);
    }

    const string illumFilename =
      outputDir + "/" + to_string(++stepDebugImages) + "_illumination.png";
    static const float kAverageIllumination = 128.0f;
    imwriteExceptionOnFail(illumFilename, kAverageIllumination * illum);
  }

  // Save illumination scales to file
  const string illumsFilename = outputDir + "/illumination_scales.txt";
  ofstream illumStream(illumsFilename, ios::out);
  if (!illumStream) {
    throw VrCamException("file open failed: " + illumsFilename);
  }
  for (auto illumination : illuminationScales) {
    illumStream << illumination;
  }
  illumStream.close();

  LOG(INFO) << "Computing errors...";

  // Get Lab for rach color patch
  for (int i = 0; i < colorPatches.size(); ++i) {
    vector<double> labOut = applyColorParams(
      rgbsRef[i],
      illuminant,
      illuminationScales[i],
      bl.data(),
      wbAndCcm.data());
    colorPatches[i].labMedian = Vec3f(labOut[0], labOut[1], labOut[2]);
  }
  computeColorPatchErrors(colorPatches, illuminant, outputDir, "ceres");
}

void computeColorPatchErrors(
    const vector<ColorPatch>& colorPatches,
    const string& illuminant,
    const string& outputDir,
    const string& titleExtra) {

  vector<float> deltaE(colorPatches.size(), 0.0f);
  for (int i = 0; i < colorPatches.size(); ++i) {
    vector<double> labRef(labMacbeth.at(illuminant)[i].begin(), labMacbeth.at(illuminant)[i].end());
    Vec3f labOut3f = colorPatches[i].labMedian;
    const vector<double> labOut = {labOut3f[0], labOut3f[1], labOut3f[2]};

    for (int j = 0; j < labOut.size(); ++j) {
      deltaE[i] += square(labRef[j] - labOut[j]);
    }
    deltaE[i] = std::sqrt(deltaE[i]);

    LOG(INFO) << "Patch " << i
              << ": Lab out: [" << labOut[0] << ", " << labOut[1] << ", " << labOut[2]
              << "], Lab ref: [" << labRef[0] << ", " << labRef[1] << ", " << labRef[2]
              << "] DeltaE: " << deltaE[i];
  }

  const float deltaSum = accumulate(deltaE.begin(), deltaE.end(), 0.0);
  LOG(INFO) << "DeltaE mean: " << (deltaSum / deltaE.size());

  // Save deltas to file
  const string deltaEFilename = outputDir + "/deltaE_" + titleExtra + ".txt";
  ofstream deltaEStream(deltaEFilename, ios::out);
  if (!deltaEStream) {
    throw VrCamException("file open failed: " + deltaEFilename);
  }
  for (int i = 0; i < deltaE.size(); ++i) {
    deltaEStream << deltaE[i] << "\n";
  }
  deltaEStream.close();
}

}  // namespace color_calibration
}  // namespace surround360
