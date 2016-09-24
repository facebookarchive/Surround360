/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "ColorCalibration.h"

#include <iostream>
#include <string>
#include <vector>

#include "CameraIsp.h"
#include "CvUtil.h"
#include "LinearRegression.h"
#include "SystemUtil.h"
#include "VrCamException.h"

namespace surround360 {
namespace color_calibration {

using namespace std;
using namespace cv;
using namespace linear_regression;
using namespace util;

Mat raw2rgb(
    const string& ispConfigFile,
    const Mat& raw,
    const Point3f blackLevel,
    const Vec3f& whiteBalanceGain,
    const Mat& ccm,
    const Point3f gamma,
    const string& ispConfigFileOut) {

  // Load camera ISP configuration
  CameraIsp cameraIsp(getJson(ispConfigFile), 8);

  // Imaging pipeline
  cameraIsp.setBlackLevel(blackLevel);
  cameraIsp.setWhiteBalance(whiteBalanceGain);
  cameraIsp.setDemosaicFilter(EDGE_AWARE_DM_FILTER);
  cameraIsp.setCCM(ccm);
  cameraIsp.setGamma(gamma);
  cameraIsp.loadImage(raw);

  if (!ispConfigFileOut.empty()) {
    cameraIsp.dumpConfigFile(ispConfigFileOut);
  }

  Mat outputImage(raw.rows, raw.cols, CV_8UC3);
  cameraIsp.getImage(outputImage);

  // We want the RGB unclamped [0..1] version
  return cameraIsp.getDemosaicedImage();
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

Point3f findBlackPoint(
    const Mat& image,
    const string& ispConfigFile,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  // Ignore 0-valued pixels (could be dead pixels)
  Mat maskNonZero;
  threshold(image, maskNonZero, 0, 255, THRESH_BINARY);

  // Ignore borders of image (e.g. embedded info)
  Mat maskCenter = createMaskFromCenter(image, 0.9f);

  double blackLevel;
  Point minLoc;
  minMaxLoc(
    image, &blackLevel, nullptr, &minLoc, nullptr, maskNonZero & maskCenter);

  if (saveDebugImages) {
    Mat rawRGB(image.size(), CV_8UC3);
    cvtColor(image, rawRGB, CV_GRAY2RGB);
    const int radius = 5;
    circle(rawRGB, minLoc, radius, Scalar(0, 255, 0), 2);

    const string rawBlurImageFilename = outputDir +
          "/" + to_string(++stepDebugImages) + "_black_point.png";
    imwriteExceptionOnFail(rawBlurImageFilename, rawRGB);
  }

  // Get closest R, G and B values from Bayer pattern
  return getClosestRGB(image, minLoc, ispConfigFile);
}

Mat createMaskFromCenter(const Mat& image, const float percentage) {
  const int topRow = (1.0f - percentage) * image.rows / 2.0f;
  const int topCol = (1.0f - percentage) * image.cols / 2.0f;
  Mat mask(image.size(), CV_8UC1, Scalar::all(0));
  Rect roi(topRow, topCol, percentage * image.rows, percentage * image.cols);
  mask(roi).setTo(Scalar::all(255));
  return mask;
}

Point3f getClosestRGB(
    const Mat& image,
    const Point center,
    const string& ispConfigFile) {

  // Load camera ISP configuration
  CameraIsp cameraIsp(getJson(ispConfigFile), 8);

  // Search in a 3x3 window surrounding pixel location
  Point3f rgb;
  for (int rowOffset = -1; rowOffset < 2; ++rowOffset) {
    for (int colOffset = -1; colOffset < 2; ++colOffset) {
      const int row = center.y + rowOffset;
      const int col = center.x + colOffset;
      const int val = image.at<uchar>(row, col);

      if (cameraIsp.redPixel(row, col)) {
        rgb.x = val;
      } else if (cameraIsp.greenPixel(row, col)) {
        rgb.y = val;
      } else {
        rgb.z = val;
      }
    }
  }

  return rgb;
}

vector<ColorPatch> detectColorChart(
    const Mat& image,
    const int numSquaresW,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  // Adaptive thresholding
  Mat bw;
  const double maxValue = 255.0;
  const int blockSize = 19;
  const int weightedSub = 2;
  adaptiveThreshold(
    image,
    bw,
    maxValue,
    ADAPTIVE_THRESH_MEAN_C,
    THRESH_BINARY_INV,
    blockSize,
    weightedSub);

  if (saveDebugImages) {
    const string adaptiveThreshImageFilename = outputDir +
          "/" + to_string(++stepDebugImages) + "_adaptive_threshold.png";
    imwriteExceptionOnFail(adaptiveThreshImageFilename, bw);
  }

  // Morphological opening and closing
  bw = morphOpeningAndClosing(bw, saveDebugImages, outputDir, stepDebugImages);

  // Find contours
  vector<vector<Point>> contours =
    findContours(bw, saveDebugImages, outputDir, stepDebugImages);

  // Morphological constraints
  const float patchMinAreaPercentage = 0.02f;
  const float patchMaxAreaPercentage = 0.25f;
  const int minArea = patchMinAreaPercentage / 100 * bw.cols * bw.rows;
  const int maxArea = patchMaxAreaPercentage / 100 * bw.cols * bw.rows;
  const float maxAspectRatio = 1.2f;

  vector<ColorPatch> colorPatchList;

  int countPatches = 0;
  for (int nL = 0; nL < contours.size(); ++nL) {
    vector<Point2i> cont = contours[nL];
    RotatedRect boundingBox = minAreaRect(cont);
    Moments mu = moments(cont, false);

    Point2f centroid = boundingBox.center;

    const int width = boundingBox.size.width;
    const int height = boundingBox.size.height;
    const int area = mu.m00;
    const int aspectRatio =
      1.0f * std::max(width, height) / (1.0f * std::min(width, height));

    // Discard contours that are too small/large, non-square and non-convex
    if (
      (area < minArea || area > maxArea) ||
      (cont.size() != 4 || aspectRatio > maxAspectRatio) ||
      !isContourConvex(cont)) {
      continue;
    }

    LOG(INFO) << "Patch found (" << countPatches++ << ")!";

    // Create patch mask
    Mat patchMask(bw.size(), CV_8UC1, Scalar::all(0));
    patchMask(boundingRect(cont)).setTo(Scalar::all(255));

    // Add patch to list
    ColorPatch colorPatch;
    colorPatch.width = width;
    colorPatch.height = height;
    colorPatch.area = area;
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

  return colorPatchListSorted;
}

Mat morphOpeningAndClosing(
    const Mat& imageIn,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  Mat imageOut;
  const float morphPercentage = 0.1f;
  const int morphType = MORPH_ELLIPSE;
  const int ellipseRadius =
    morphPercentage * 0.01f * std::min(imageIn.cols, imageIn.rows);
  Size morphSize = Size(2 * ellipseRadius + 1, 2 * ellipseRadius + 1);
  Mat element = getStructuringElement(
    MORPH_ELLIPSE,
    morphSize,
    Point2f(ellipseRadius, ellipseRadius));
  morphologyEx(imageIn, imageOut, MORPH_CLOSE, element);
  morphologyEx(imageIn, imageOut, MORPH_OPEN, element);

  if (saveDebugImages) {
    const string morphOpeningImageFilename = outputDir +
          "/" + to_string(++stepDebugImages) + "_morph_opening_closing.png";
    imwriteExceptionOnFail(morphOpeningImageFilename, imageOut);
  }

  return imageOut;
}

vector<vector<Point>> findContours(
    const Mat& image,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(
    image,
    contours,
    hierarchy,
    CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE,
    Point(0, 0));

  // Straighten contours to minimize number of vertices
  for(int nC = 0; nC < contours.size(); ++nC) {
    const double epsilonPolyDP = 0.12 * arcLength(contours[nC], true);
    approxPolyDP(contours[nC], contours[nC], epsilonPolyDP, true);
  }

  if (saveDebugImages) {
    Mat contoursPlot = Mat::zeros(image.size(), CV_8UC3);
    RNG rng(12345);
    for(int nC = 0; nC < contours.size(); ++nC) {
      Scalar color =
        Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
      drawContours(
        contoursPlot, contours, nC, color, 2, 8, hierarchy, 0, Point());
    }
    const string contoursImageFilename = outputDir +
          "/" + to_string(++stepDebugImages) + "_contours.png";
    imwriteExceptionOnFail(contoursImageFilename, contoursPlot);
   }

  return contours;
}

vector<ColorPatch> removeContourOutliers(vector<ColorPatch> colorPatchList) {
  // Store min distance between patches, for each patch
  vector<float> minDistances;
  for (int iPatch = 0; iPatch < colorPatchList.size(); ++iPatch) {
    minDistances.push_back(FLT_MAX);
    Point2f ci = colorPatchList[iPatch].centroid;
    for (int jPatch = 0; jPatch < colorPatchList.size(); ++jPatch) {
      if (iPatch == jPatch) {
        continue;
      }

      Point2f cj = colorPatchList[jPatch].centroid;
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
    const vector<ColorPatch> colorPatchList,
    const int numSquaresW,
    const Size imageSize) {

  Point2f refTL = Point2f {0.0f, 0.0f};
  Point2f refTR = Point2f {static_cast<float>(imageSize.width), 0.0f};
  Point2f topleft = Point2f {FLT_MAX, FLT_MAX};
  Point2f topright = Point2f {-1, FLT_MAX};

  // Assuming top left is (0, 0)
  vector<Point2f> centroids;
  for (ColorPatch patch : colorPatchList) {
    Point2f centroid = patch.centroid;
    centroids.push_back(centroid);

    if (norm(refTL - centroid) < norm(refTL - topleft)) {
      topleft = centroid;
    }
    if (norm(refTR - centroid) < norm(refTR - topright)) {
      topright = centroid;
    }
  }

  vector<Point2f> centroidsSorted;

  while (centroids.size() > 0) {
    // Get points in current row, i.e. closest to line TL-TR
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
    for (int nC = 0; nC < numSquaresW && nC < centroids.size(); ++nC) {
      centroidsRow.push_back(centroids[nC]);
    }

    // Sort row by X coordinate
    sort(centroidsRow.begin(), centroidsRow.end(), sortPointsX);

    // Add row to parent vector
    for (int nC = 0; nC < numSquaresW; ++nC) {
      centroidsSorted.push_back(centroidsRow[nC]);
    }

    // Remove row from vector
    centroids.erase(
      centroids.begin(),
      centroids.size() > numSquaresW
        ? centroids.begin() + numSquaresW
        : centroids.end());
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

Point2f findTopLeft(const vector<Point2f> points) {
  const Point2f refTL = Point2f {0, 0};
  Point2f topleft = Point2f {FLT_MAX, FLT_MAX};
  for (const Point2f& p : points) {
    if (norm(refTL - p) < norm(refTL - topleft)) {
      topleft = p;
    }
  }
  return topleft;
}

Point2f findTopRight(const vector<Point2f> points, const int imageWidth) {
  const Point2f refTR = Point2f {static_cast<float>(imageWidth), 0};
  Point2f topright = Point2f {-1, FLT_MAX};
  for (const Point2f& p : points) {
    if (norm(refTR - p) < norm(refTR - topright)) {
      topright = p;
    }
  }
  return topright;
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

void computeRGBMedians(
    vector<ColorPatch>& colorPatches,
    const Mat& rgb,
    const bool saveDebugImages,
    const string& outputDir,
    int& stepDebugImages) {

  // Rescale input image to [0, 255] to make histogram calculations easier
  Mat rgb255(rgb.size(), CV_8UC3);
  rgb255 = rgb * 255.0f;

  const int numChannels = rgb255.channels();
  const int kHistNumImages = 1;
  const int kHistChannels = 0;
  const int kHistDim = 1;
  const int kHistSize = 256;
  const float range[] = {0, kHistSize};
  const float* kHistRange = {range};

  // Split RGB into channels
  vector<Mat> rgbChannels;
  split(rgb255, rgbChannels);

  // Compute medians through histograms
  for (int i = 0; i < colorPatches.size(); ++i) {
    // Get histogram for each RGB channel
    vector<Mat> histRGB(numChannels, Mat());
    for (int j = 0; j < numChannels; ++j) {
      calcHist(
        &rgbChannels[j],
        kHistNumImages,
        &kHistChannels,
        colorPatches[i].mask,
        histRGB[j],
        kHistDim,
        &kHistSize,
        &kHistRange);
    }

    Vec3d binRGB = {0, 0, 0};
    Vec3f rgbMedian {-1.0f, -1.0f, -1.0f};
    const double midPoint = colorPatches[i].area / 2;

    // Get index with half of the counts on each side
    for (int j = 0; j < kHistSize; ++j) {
      if (rgbMedian[0] >= 0 && rgbMedian[1] > 0 && rgbMedian[2] > 0) {
        break;
      }
      for (int k = 0; k < numChannels; ++k) {
        binRGB[k] += round(histRGB[k].at<float>(j));

        if (binRGB[k] > midPoint && rgbMedian[k] < 0) {
          rgbMedian[k] = j;
        }
      }
    }

    colorPatches[i].rgbMedian = rgbMedian / 255.0f;

    LOG(INFO)
      << "Patch " << i << " (" << colorPatches[i].centroid << ") "
      << "RGB median: " << colorPatches[i].rgbMedian;

    // Draw circle on top of patch filled with computed RGB median
    if (saveDebugImages) {
      const int radius =
        round(
          1.0 * std::max(colorPatches[i].width, colorPatches[i].height) / 3.0);
      circle(
        rgb255,
        colorPatches[i].centroid,
        radius,
        Scalar(rgbMedian),
        -1);
      circle(rgb255, colorPatches[i].centroid, radius, Scalar(0, 255, 0));

      const double kTextFontScale = 0.3;
      putText(
        rgb255,
        to_string(i),
        colorPatches[i].centroid,
        FONT_HERSHEY_SIMPLEX,
        kTextFontScale,
        Scalar(0, 255, 0));
    }
  }

  if (saveDebugImages) {
    Mat bgr255;
    cvtColor(rgb255, bgr255, CV_RGB2BGR);
    const string rgbPacthesImageFilename = outputDir +
          "/" + to_string(++stepDebugImages) + "_color_patches.png";
    imwriteExceptionOnFail(rgbPacthesImageFilename, bgr255);
  }
}

// Compute channel ratios for white balance from second to last brightest patch.
// Sensors assumed to be linear, but darkest/brightest patch could be under/over
// saturated
Vec3f computeChannelRatios(
    const vector<ColorPatch>& colorPatches,
    const int numPatchesRow) {

  double maxMedian;
  const int iPatchRatios = colorPatches.size() - numPatchesRow;
  const Vec3f median = colorPatches[iPatchRatios].rgbMedian;

  minMaxLoc(median, nullptr, &maxMedian);

  return Vec3f {
    float(maxMedian) / median[0],
    float(maxMedian) / median[1],
    float(maxMedian) / median[2] };
}

void plotWhiteBalanceHistogram(
    const vector<ColorPatch> colorPatches,
    const Vec3f channelRatios,
    const int numPatchesRow,
    const string& outputDir,
    int& stepDebugImages) {

  // X-axis will have (int) Y values in from
  // http://www.babelcolor.com/colorchecker-2.htm#CCP2_data
  // (scaled by a factor of 2 for scale)
  const int macBethGrayValues[] = {6, 18, 36, 72, 118, 182};

  const int kHistSize = 256;
  const int kHistWidth = 200;
  const int kHistHeight = kHistSize - 1;
  const int kBinWidth = cvRound((double) kHistWidth / kHistSize);

  Mat histImage(kHistHeight, kHistWidth, CV_8UC3, Scalar::all(255));
  Mat histImageWB(kHistHeight, kHistWidth, CV_8UC3, Scalar::all(255));
  Point2f startingPoint = Point2f(0, kHistHeight);
  vector<Point2f> prevPoints = {startingPoint, startingPoint, startingPoint};
  vector<Point2f> prevPointsWB = {startingPoint, startingPoint, startingPoint};

  // Draw the histogram for grayscale patches (bottom row)
  const int nPatches = colorPatches.size();
  for (int i = nPatches - 1; i > nPatches - numPatchesRow - 1; --i) {
    const int iGray = colorPatches.size() - i - 1;

    Vec3f rgbMedian = 255.0f * colorPatches[i].rgbMedian;

    if (iGray < numPatchesRow) {
      const int xCoord = macBethGrayValues[iGray];
      for (int iP = 0; iP < prevPoints.size(); ++iP) {
        Scalar color = iP == 0
          ? Scalar(0, 0, 255)
          : (iP == 1 ? Scalar(0, 255, 0) : Scalar(255, 0, 0));

        Point2f p = Point2f(xCoord, kHistHeight - rgbMedian[iP]);
        line(histImage, prevPoints[iP], p, color, 2, 8, 0);
        prevPoints[iP] = p;

        // Scale channels and plot again
        Point2f pWB =
          Point2f(xCoord, kHistHeight - rgbMedian[iP] * channelRatios[iP]);

        line(histImageWB, prevPointsWB[iP], pWB, color, 2, 8, 0);
        prevPointsWB[iP] = pWB;
      }
    }
  }

  const string histOriginalImageFilename = outputDir +
        "/" + to_string(++stepDebugImages) + "_histogram_original.png";
  imwriteExceptionOnFail(histOriginalImageFilename, histImage);

  const string histWBImageFilename = outputDir +
        "/" + to_string(++stepDebugImages) + "_histogram_white_balance.png";
  imwriteExceptionOnFail(histWBImageFilename, histImageWB);
}

Mat computeCCM(const vector<ColorPatch> colorPatches) {
  vector<vector<float>> inputs;
  vector<vector<float>> outputs;

  // Assuming raster scan order
  // Assuming color patch medians are [0..1]
  for (int i = 0; i < colorPatches.size(); ++i) {
    Vec3f patchRGB = colorPatches[i].rgbMedian;

    vector<float> patchRGBv {patchRGB[0], patchRGB[1], patchRGB[2]};
    inputs.push_back(patchRGBv);

    // Convert RGB to float
    vector<float> rgbLinearMacbethOut(
      rgbLinearMacbeth[i].begin(),
      rgbLinearMacbeth[i].end());

    // Normalize ground truth color patch to [0..1]
    transform(
      rgbLinearMacbethOut.begin(),
      rgbLinearMacbethOut.end(),
      rgbLinearMacbethOut.begin(),
      bind1st(multiplies<float>(), 1.0f / 255.0f));

    outputs.push_back(rgbLinearMacbethOut);
  }

  static const int kInputDim = 3;
  static const int kOutputDim = 3;
  static const int kNumIterations = 100000;
  static const float kStepSize = 0.1f;
  static const bool kPrintObjective = true;
  vector<vector<float>> ccm = solveLinearRegressionRdToRk(
    kInputDim,
    kOutputDim,
    inputs,
    outputs,
    kNumIterations,
    kStepSize,
    kPrintObjective);

  // Make sure we have a 3x3 matrix
  assert(ccm.size() == 3 && ccm.size() == ccm[0].size());

  // Convert to Mat
  Mat ccmMat(ccm.size(), ccm.at(0).size(), CV_32FC1);
  for(int i = 0; i < ccmMat.rows; ++i) {
    for(int j = 0; j < ccmMat.cols; ++j) {
      ccmMat.at<float>(i, j) = ccm.at(i).at(j);
    }
  }

  return ccmMat;
}

pair<Vec4f, Vec4f> computeColorPatchErrors(
    const Mat& imBefore,
    const Mat& imAfter,
    const vector<ColorPatch> colorPatches) {

  // Compute errors in [0..255]
  imBefore *= 255.0f;
  imAfter *= 255.0f;

  Vec4f errBefore {0.0f, 0.0f, 0.0f, 0.0f};
  Vec4f errAfter {0.0f, 0.0f, 0.0f, 0.0f};
  const int numPatches =  colorPatches.size();

  for (int i = 0; i < numPatches; ++i) {
    Vec3f medianBefore = imBefore.at<Vec3f>(colorPatches[i].centroid);
    Vec3f medianAfter = imAfter.at<Vec3f>(colorPatches[i].centroid);
    Vec3f macbethPatchVal = Vec3i {
      rgbLinearMacbeth[i][0],
      rgbLinearMacbeth[i][1],
      rgbLinearMacbeth[i][2] };

    Vec3f diffBefore = colorPatches[i].rgbMedian  - macbethPatchVal;
    Vec3f diffAfter = medianAfter - macbethPatchVal;

    // RGB error
    errBefore[0] += norm(diffBefore, NORM_L2) / numPatches;
    errAfter[0] += norm(diffAfter, NORM_L2) / numPatches;

    // B, G, R errors
    for (int iErr = 1; iErr < 4; ++iErr) {
      errBefore[iErr] += fabs(diffBefore[iErr - 1]) / numPatches;
      errAfter[iErr] += fabs(diffAfter[iErr - 1]) / numPatches;
    }
  }

  return make_pair(errBefore, errAfter);
}

} // namespace color_calibration
} // namespace surround360
