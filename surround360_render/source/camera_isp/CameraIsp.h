/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include "ColorspaceConversion.h"
#include "CvUtil.h"
#include "Filter.h"
#include "JsonUtil.h"
#include "MathUtil.h"
#include "MonotonicTable.h"
#include "VrCamException.h"

#include <glog/logging.h>

namespace surround360 {

using namespace std;
using namespace cv;
using namespace surround360::color;
using namespace surround360::util;

enum DemosaicFilter {
  BILINEAR_DM_FILTER = 0,
  FREQUENCY_DM_FILTER,
  EDGE_AWARE_DM_FILTER,
  LAST_DM_FILTER
};

const int kToneCurveLutSize = 4096;

class CameraIsp {
 protected:
  string bayerPattern;
  vector<cv::Point3f> compandingLut;
  cv::Point3f blackLevel;
  cv::Point3f clampMin;
  cv::Point3f clampMax;
  vector<cv::Point3f> vignetteRollOffH;
  vector<cv::Point3f> vignetteRollOffV;

  int stuckPixelThreshold;
  float stuckPixelDarknessThreshold;
  int stuckPixelRadius;
  int bitsPerPixel;
  int maxPixelValue;
  cv::Point3f whiteBalanceGain;
  Mat ccm; // 3x3
  Mat compositeCCM;
  float saturation;
  cv::Point3f gamma;
  cv::Point3f lowKeyBoost;
  cv::Point3f highKeyBoost;
  float contrast;
  cv::Point3f sharpening;
  float sharpeningSupport;
  float noiseCore;
  Mat rawImage;
  bool redBayerPixel[2][2];
  bool greenBayerPixel[2][2];
  Mat  demosaicedImage;
  uint32_t filters;
  DemosaicFilter demosaicFilter;
  int resize;
  bool disableToneCurve;
  vector<Vec3f> toneCurveLut;
  BezierCurve<float, Vec3f> vignetteCurveH;
  BezierCurve<float, Vec3f> vignetteCurveV;

  const int outputBpp;
  const int width;
  const int height;
  const int maxDimension;
  const float maxD; // max diagonal distance
  const float sqrtMaxD; // max diagonal distance

  void demosaicBilinearFilter(Mat& r,  Mat& g, Mat& b) const {
    for (int i = 0; i < height; ++i) {
      const int i_1 = reflect(i - 1, height);
      const int i1  = reflect(i + 1, height);

      const bool redGreenRow =
        (redPixel(i, 0) && greenPixel(i, 1)) ||
        (redPixel(i, 1) && greenPixel(i, 0));

      for (int j = 0; j < width; ++j) {
        const int j_1 = reflect(j - 1, width);
        const int j1  = reflect(j + 1, width);

        if (redPixel(i, j)) {
          g.at<float>(i, j) =
            bilerp(g.at<float>(i_1, j),
                   g.at<float>(i1, j),
                   g.at<float>(i, j_1),
                   g.at<float>(i, j1),
                   0.5f, 0.5f);

          b.at<float>(i, j) =
            bilerp(b.at<float>(i_1, j_1),
                   b.at<float>(i1, j_1),
                   b.at<float>(i_1, j1),
                   b.at<float>(i1, j1),
                   0.5f, 0.5f);

        } else if (greenPixel(i, j)) {
          if (redGreenRow) {
            b.at<float>(i, j) = (b.at<float>(i_1, j) +
                                 b.at<float>(i1, j)) / 2.0f;

            r.at<float>(i, j) = (r.at<float>(i, j_1) +
                                 r.at<float>(i, j1)) / 2.0f;
          } else {
            r.at<float>(i, j) = (r.at<float>(i_1, j) +
                                 r.at<float>(i1, j)) / 2.0f;

            b.at<float>(i, j) = (b.at<float>(i, j_1) +
                                 b.at<float>(i, j1)) / 2.0f;
          }
        } else {
          g.at<float>(i, j) =
            bilerp(g.at<float>(i_1, j),
                   g.at<float>(i1, j),
                   g.at<float>(i, j_1),
                   g.at<float>(i, j1),
                   0.5f, 0.5f);

          r.at<float>(i, j) =
            bilerp(r.at<float>(i_1, j_1),
                   r.at<float>(i1, j_1),
                   r.at<float>(i_1, j1),
                   r.at<float>(i1, j1),
                   0.5f, 0.5f);
        }
      }
    }
  }

  void demosaicFrequencyFilter(Mat& r, Mat& g, Mat& b) const {
    // Green/luma 4-th order Butterworth lp filter
    const Butterworth dFilter(0.0f, 2.0f, width + height, 1.0f, 4);
    // Chrome cross over filter
    const Butterworth dcFilter(0.0f, 2.0f, width + height, 1.0f, 2.0f);

    //  Do a per pixel filtering in DCT space
    for (int i = 0; i < height; ++i) {
      const float y = float(i) / float(height - 1);
      for (int j = 0; j < width; ++j) {
        const float x = float(j) / float(width - 1);
        // Diagonal distance and half
        static const float kDScale = 1.2f;
        const float d = (x + y) * kDScale;
        const float kSharpen = d / 2.5f + 1.0f;
        const float gGain  = 2.0f * dFilter(d) * kSharpen;
        const float rbGain = 4.0f * dFilter(d);
        g.at<float>(i, j) *= gGain;

        const float kCrossoverCutoff = 3.0f;
        const float d2 = d * 2 * kCrossoverCutoff;

        // Cross over blend value
        const float alpha = dcFilter(d2);
        r.at<float>(i, j) = lerp(g.at<float>(i, j), r.at<float>(i, j) * rbGain, alpha);
        b.at<float>(i, j) = lerp(g.at<float>(i, j), b.at<float>(i, j) * rbGain, alpha);
      }
    }
  }


  void demosaicEdgeAware(Mat& red, Mat& green, Mat& blue) const {
    // Horizontal and vertical green values
    Mat gV(height, width, CV_32F);
    Mat gH(height, width, CV_32F);

    // And their first and second order derivatives
    Mat dV(height, width, CV_32F);
    Mat dH(height, width, CV_32F);

    // Compute green gradients
    for (int i = 0; i < height; ++i) {
      const int i_1 = reflect(i - 1, height);
      const int i1  = reflect(i + 1, height);
      const int i_2 = reflect(i - 2, height);
      const int i2  = reflect(i + 2, height);

      for (int j = 0; j < width; ++j) {
        const int j_1 = reflect(j - 1, width);
        const int j1  = reflect(j + 1, width);
        const int j_2 = reflect(j - 2, width);
        const int j2  = reflect(j + 2, width);
        if (greenPixel(i, j)) {
          gV.at<float>(i, j) = green.at<float>(i, j);
          gH.at<float>(i, j) = green.at<float>(i, j);

          dV.at<float>(i, j) =
            (fabsf(green.at<float>(i2, j) - green.at<float>(i, j)) +
             fabsf(green.at<float>(i, j) - green.at<float>(i_2, j))) / 2.0f;

          dH.at<float>(i, j) =
            (fabsf(green.at<float>(i,  j2) - green.at<float>(i, j)) +
             fabsf(green.at<float>(i, j) - green.at<float>(i,  j_2))) / 2.0f;
        } else {
          gV.at<float>(i, j) = (green.at<float>(i_1, j) + green.at<float>(i1, j)) / 2.0f;
          gH.at<float>(i, j) = (green.at<float>(i, j_1) + green.at<float>(i, j1)) / 2.0f;
          dV.at<float>(i, j) = (fabsf(green.at<float>(i_1, j) - green.at<float>(i1, j))) / 2.0f;
          dH.at<float>(i, j) = (fabsf(green.at<float>(i, j_1) - green.at<float>(i, j1))) / 2.0f;

          Mat& ch = redPixel(i, j) ? red : blue;
          gV.at<float>(i, j) += (2.0f * ch.at<float>(i, j) - ch.at<float>(i_2, j) - ch.at<float>(i2, j)) / 4.0f;
          gH.at<float>(i, j) += (2.0f * ch.at<float>(i, j) - ch.at<float>(i, j_2) - ch.at<float>(i, j2)) / 4.0f;
          dV.at<float>(i, j) += fabsf(-2.0f * ch.at<float>(i, j) + ch.at<float>(i_2, j) + ch.at<float>(i2, j)) / 2.0f;
          dH.at<float>(i, j) += fabsf(-2.0f * ch.at<float>(i, j) + ch.at<float>(i, j_2) + ch.at<float>(i, j2)) / 2.0f;
        }
      }
    }
    const int w = 4;
    const int diameter = 2 * w + 1;
    const int diameterSquared = square(diameter);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        // Homogenity test
        int hCount = 0;
        for (int l = -w; l <= w; ++l) {
          const int il = reflect(i + l, height);
          for (int k = -w; k <= w; ++k) {
            const int jk = reflect(j + k, width);
            hCount += (dH.at<float>(il, jk) <= dV.at<float>(il, jk));
          }
        }
        green.at<float>(i, j) = hCount < diameterSquared / 2 ? gV.at<float>(i, j) : gH.at<float>(i, j);
      }
    }

    // compute r-b
    Mat redMinusGreen(height, width, CV_32F);
    Mat blueMinusGreen(height, width, CV_32F);
    Mat pGreen(height, width, CV_32F);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        pGreen.at<float>(i, j) = green.at<float>(i, j);
        if (redPixel(i, j)) {
          redMinusGreen.at<float>(i, j) = red.at<float>(i, j) - pGreen.at<float>(i, j);
        } else if (!greenPixel(i, j)) {
          blueMinusGreen.at<float>(i, j) = blue.at<float>(i, j) - pGreen.at<float>(i, j);
        }
      }
    }
    // Now use a constant hue based red/blue bilinear interpolation
    for (int i = 0; i < height; ++i) {
      const int i_1 = reflect(i - 1, height);
      const int i1  = reflect(i + 1, height);
      const int i_2 = reflect(i - 2, height);
      const int i2  = reflect(i + 2, height);

      const bool redGreenRow =
        (redPixel(i, 0) && greenPixel(i, 1)) ||
        (redPixel(i, 1) && greenPixel(i, 0));

      for (int j = 0; j < width; ++j) {
        const int j_1 = reflect(j - 1, width);
        const int j1  = reflect(j + 1, width);
        const int j_2 = reflect(j - 2, width);
        const int j2  = reflect(j + 2, width);

        if (redPixel(i, j)) {
          blue.at<float>(i, j) =
            (blueMinusGreen.at<float>(i_1, j_1) +
             blueMinusGreen.at<float>(i1, j_1) +
             blueMinusGreen.at<float>(i_1, j1) +
             blueMinusGreen.at<float>(i1, j1)) / 4.0f +
            pGreen.at<float>(i, j);

          red.at<float>(i, j) =
            (redMinusGreen.at<float>(i, j) +
             redMinusGreen.at<float>(i_2, j) +
             redMinusGreen.at<float>(i2, j) +
             redMinusGreen.at<float>(i, j_2) +
             redMinusGreen.at<float>(i, j2)) / 5.0f +
            pGreen.at<float>(i, j);
        } else if (greenPixel(i, j)) {
          Mat& diffCh1 = redGreenRow ? blueMinusGreen : redMinusGreen;
          Mat& diffCh2 = redGreenRow ? redMinusGreen :  blueMinusGreen;

          Mat& ch1 = redGreenRow ? blue : red;
          Mat& ch2 = redGreenRow ? red :  blue;

          ch1.at<float>(i, j) =
            (diffCh1.at<float>(i_1, j_2) +
             diffCh1.at<float>(i_1, j) +
             diffCh1.at<float>(i_1, j2) +
             diffCh1.at<float>(i1,  j_2) +
             diffCh1.at<float>(i1,  j2) +
             diffCh1.at<float>(i1,  j2)) / 6.0f +
            pGreen.at<float>(i, j);

          ch2.at<float>(i, j) =
            (diffCh2.at<float>(i_2, j_1) +
             diffCh2.at<float>(i,   j_1) +
             diffCh2.at<float>(i2,  j_1) +
             diffCh2.at<float>(i_2, j1) +
             diffCh2.at<float>(i,   j1) +
             diffCh2.at<float>(i2,  j1)) / 6.0f +
            pGreen.at<float>(i, j);
        } else {
          red.at<float>(i, j) =
            (redMinusGreen.at<float>(i_1, j_1) +
             redMinusGreen.at<float>(i1, j_1) +
             redMinusGreen.at<float>(i_1, j1) +
             redMinusGreen.at<float>(i1, j1)) / 4.0f +
            pGreen.at<float>(i, j);

          blue.at<float>(i, j) =
            (blueMinusGreen.at<float>(i,  j) +
             blueMinusGreen.at<float>(i_2, j) +
             blueMinusGreen.at<float>(i2, j) +
             blueMinusGreen.at<float>(i, j_2) +
             blueMinusGreen.at<float>(i, j2)) / 5.0f +
            pGreen.at<float>(i, j);
        }
      }
    }
  }


  template <class T>
  void resizeInput(const Mat& inputImage) {
    const float areaRecip = 1.0f / (maxPixelValue * float(square(resize)));
    const int r = resize > 1 ? 2 : 1;

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        float sum = 0.0f;
        for (int k = 0; k < resize; ++k) {
          const int ip = i*resize + k*2;
          const int ipp = reflect(ip + (i % r), inputImage.rows);
          for (int l = 0; l < resize; ++l) {
            const int jp = j*resize + l*2;
            const int jpp = reflect(jp + (j % r), inputImage.cols);
            sum += float(inputImage.at<T>(ipp, jpp));
          }
        }
        rawImage.at<float>(i, j) = sum * areaRecip;
      }
    }
  }

  // Used to build up the low and high key parts of the tone curve
  inline float bezier(float a, float b, float c, float d, float t) {
    // Four point DeCasteljau's Algorithm
    return
      lerp(
        lerp(
            lerp(a, b, t),
            lerp(b, c, t), t),
        lerp(
            lerp(b, c, t),
            lerp(c, d, t), t), t);
  }

  inline float highKey(float highKeyBoost, float x) {
    const float a = 0.5f;
    const float b = clamp(0.6666f, 0.0f, 1.0f);
    const float c = clamp(0.8333f + highKeyBoost, 0.0f, 1.0f);
    const float d = 1.0f;
    return x > 0.5f ? bezier(a, b, c, d, (x - 0.5f) * 2.0f) : 0;
  }

  inline float lowKey(float lowKeyBoost, float x) {
    const float a = 0.0f;
    const float b = clamp(0.1666f + lowKeyBoost, 0.0f, 1.0f);
    const float c = clamp(0.3333f, 0.0f, 1.0f);
    const float d = 0.5f;
    return x <= 0.5f ? bezier(a, b, c, d, x * 2.0f) : 0;
  }

  // Build the composite tone curve
  void buildToneCurveLut() {
    toneCurveLut.clear();
    const float range = float((1 << outputBpp) - 1);
    const float dx = 1.0f / float(kToneCurveLutSize - 1);

    // Contrast angle constants
    const float angle = M_PI * 0.25f * contrast;
    const float slope = tanf(angle);
    const float bias = 0.5f * (1.0f - slope);

    for (int i = 0; i < kToneCurveLutSize; ++i) {
      const float x = dx * i;
      if (disableToneCurve) {
        // Just a linear ramp ==> no-op
        const float y = x * range;
        toneCurveLut.push_back(Vec3f(y, y, y));
      } else {
        // Apply gamma correction
        float r = powf(x, gamma.x);
        float g = powf(x, gamma.y);
        float b = powf(x, gamma.z);

        // Then low/high key boost
        r = lowKey(lowKeyBoost.x, r) + highKey(highKeyBoost.x, r);
        g = lowKey(lowKeyBoost.y, g) + highKey(highKeyBoost.y, g);
        b = lowKey(lowKeyBoost.z, b) + highKey(highKeyBoost.z, b);

        // Then contrast
        r = clamp((slope * r + bias) * range, 0.0f, range);
        g = clamp((slope * g + bias) * range, 0.0f, range);
        b = clamp((slope * b + bias) * range, 0.0f, range);

        // Place it in the table.
        toneCurveLut.push_back(Vec3f(r, g, b));
      }
    }
  }

 public:
  CameraIsp(const string jsonInput, const int outputBpp) :
      demosaicFilter(EDGE_AWARE_DM_FILTER),
      resize(1),
      disableToneCurve(false),
      outputBpp(outputBpp),
      width(0),
      height(0),
      maxDimension(0),
      maxD(0),
      sqrtMaxD(0) {

    const json::Object config = json::Deserialize(jsonInput);

    // Set the default values and override them from the json file
    bitsPerPixel = 8;
    maxPixelValue = (1 << bitsPerPixel) - 1;
    compandingLut.push_back(Point3f(0.0f, 0.0f, 0.0f));
    compandingLut.push_back(Point3f(1.0f, 1.0f, 1.0f));
    blackLevel = Point3f(0.0f, 0.0f, 0.0f);
    clampMin = Point3f(0.0f, 0.0f, 0.0f);
    clampMax = Point3f(1.0f, 1.0f, 1.0f);
    stuckPixelThreshold = 0;
    stuckPixelDarknessThreshold = 0;
    stuckPixelRadius = 0;
    vignetteRollOffH.push_back(Vec3f(1.0f, 1.0f, 1.0f));
    vignetteRollOffV.push_back(Vec3f(1.0f, 1.0f, 1.0f));
    whiteBalanceGain = Point3f(1.0f, 1.0f, 1.0f);
    ccm = Mat::eye(3, 3, CV_32F);
    saturation = 1.0f;
    contrast = 1.0f;
    sharpening = Point3f(0.0f, 0.0f, 0.0f);
    sharpeningSupport = sharpeningSupport = 10.0f / 2048.0f; // Approx filter support is 10 pixels
    noiseCore = 1000.0f;
    gamma = Point3f(1.0f, 1.0f, 1.0f);
    lowKeyBoost = Point3f(0.0f, 0.0f, 0.0f);
    highKeyBoost = Point3f(0.0f, 0.0f, 0.0f);
    bayerPattern = "GBRG";

    if (config.HasKey("CameraIsp")) {
      if (config["CameraIsp"].HasKey("bitsPerPixel")) {
        bitsPerPixel = getInteger(config, "CameraIsp", "bitsPerPixel");
      } else {
        VLOG(1) << "Using default bitPerPixel = " << bitsPerPixel << endl;
      }

      maxPixelValue = (1 << bitsPerPixel) - 1;

      if (config["CameraIsp"].HasKey("compandingLut")) {
        compandingLut = getCoordList(config, "CameraIsp", "compandingLut");
      } else {
        VLOG(1) << "Using default compandingLut = " << compandingLut << endl;
      }

      if (config["CameraIsp"].HasKey("blackLevel")) {
        blackLevel = getVector(config, "CameraIsp", "blackLevel");
      }  else {
        VLOG(1) << "Using default blackLevel = " << blackLevel << endl;
      }

      if (config["CameraIsp"].HasKey("clampMin")) {
        clampMin = getVector(config, "CameraIsp", "clampMin");
      }  else {
        VLOG(1) << "Using default clampMin = " << clampMin << endl;
      }

      if (config["CameraIsp"].HasKey("clampMax")) {
        clampMax = getVector(config, "CameraIsp", "clampMax");
      }  else {
        VLOG(1) << "Using default clampMax = " << clampMax << endl;
      }

      if (config["CameraIsp"].HasKey("stuckPixelThreshold")) {
        stuckPixelThreshold =
          getInteger(config, "CameraIsp", "stuckPixelThreshold");
      } else {
        VLOG(1) << "Using default stuckPixelThreshold = " << stuckPixelThreshold << endl;
      }

      if (config["CameraIsp"].HasKey("stuckPixelDarknessThreshold")) {
        stuckPixelDarknessThreshold =
          getDouble(config, "CameraIsp", "stuckPixelDarknessThreshold");
      } else {
        VLOG(1) << "Using default stuckPixelDarknessThreshold = " << stuckPixelDarknessThreshold << endl;
      }

      if (config["CameraIsp"].HasKey("stuckPixelRadius")) {
        stuckPixelRadius = 2 * getInteger(config, "CameraIsp", "stuckPixelRadius");
      } else {
        VLOG(1) << "Using default stuckPixelRadius = " << stuckPixelRadius << endl;
      }

      if (config["CameraIsp"].HasKey("vignetteRollOffH")) {
        vignetteRollOffH = getCoordList(config, "CameraIsp", "vignetteRollOffH");
      } else {
        VLOG(1) << "Using default vignetteRollOffH = " << vignetteRollOffH << endl;
      }

      if (config["CameraIsp"].HasKey("vignetteRollOffV")) {
        vignetteRollOffV = getCoordList(config, "CameraIsp", "vignetteRollOffV");
      } else {
        VLOG(1) << "Using default vignetteRollOffV = " << vignetteRollOffV << endl;
      }

      if (config["CameraIsp"].HasKey("whiteBalanceGain")) {
        whiteBalanceGain = getVector(config, "CameraIsp", "whiteBalanceGain");
      } else {
        VLOG(1) << "Using default whiteBalanceGain = " << whiteBalanceGain << endl;
      }

      if (config["CameraIsp"].HasKey("denoise")) {
        VLOG(1) << "[Deprecated] chroma denoising folded into demosaicing" << endl;
      }

      if (config["CameraIsp"].HasKey("denoiseRadius")) {
        VLOG(1) << "[Deprecated] chroma denoising folded into demosaicing" << endl;
      }

      if (config["CameraIsp"].HasKey("ccm")) {
        ccm = getMatrix(config, "CameraIsp", "ccm");
      } else {
        VLOG(1) << "Using default ccm = " << ccm << endl;
      }

      if (config["CameraIsp"].HasKey("saturation")) {
        saturation = getDouble(config, "CameraIsp", "saturation");
      } else {
        VLOG(1) << "Using default saturation = " << saturation << endl;
      }

      if (config["CameraIsp"].HasKey("gamma")) {
        gamma = getVector(config, "CameraIsp", "gamma");
      }  else {
        VLOG(1) << "Using default gamma = " << gamma << endl;
      }

      if (config["CameraIsp"].HasKey("lowKeyBoost")) {
        lowKeyBoost = getVector(config, "CameraIsp", "lowKeyBoost");
      } else {
        VLOG(1) << "Using default lowKeyBoost = " << lowKeyBoost << endl;
      }

      if (config["CameraIsp"].HasKey("highKeyBoost")) {
        highKeyBoost = getVector(config, "CameraIsp", "highKeyBoost");
      } else {
        VLOG(1) << "Using default highKeyBoost = " << highKeyBoost << endl;
      }

      if (config["CameraIsp"].HasKey("contrast")) {
        contrast = getDouble(config, "CameraIsp", "contrast");
      } else {
        VLOG(1) << "Using default constrast = " << contrast << endl;
      }

      if (config["CameraIsp"].HasKey("sharpening")) {
        sharpening = getVector(config, "CameraIsp", "sharpening");
      } else {
        VLOG(1) << "Using default sharpening = " << sharpening << endl;
      }

      if (config["CameraIsp"].HasKey("sharpeningSupport")) {
        sharpeningSupport = getDouble(config, "CameraIsp", "sharpeningSupport");
      } else {
        VLOG(1) << "Using default sharpening support = " << sharpeningSupport << endl;
      }

      if (config["CameraIsp"].HasKey("noiseCore")) {
        noiseCore = getDouble(config, "CameraIsp", "noiseCore");
      } else {
        VLOG(1) << "Using default noise core = " << noiseCore << endl;
      }

      if (config["CameraIsp"].HasKey("bayerPattern")) {
        bayerPattern = getString(config, "CameraIsp", "bayerPattern");
      } else {
        VLOG(1) << "Using default bayerPattern = " << bayerPattern << endl;
      }
    } else {
      VLOG(1) << "Missing or \"CameraIsp\" using defaults.\n";
    }

    setup();
  }

  void setup() {
    // Build the bayer pattern tables
    if (bayerPattern.find("RGGB") != std::string::npos) {
      filters = 0x94949494;
      redBayerPixel[0][0] = true;
      redBayerPixel[0][1] = false;
      redBayerPixel[1][0] = false;
      redBayerPixel[1][1] = false;

      greenBayerPixel[0][0] = false;
      greenBayerPixel[0][1] = true;
      greenBayerPixel[1][0] = true;
      greenBayerPixel[1][1] = false;
    } else if (bayerPattern.find("GRBG") != std::string::npos) {
      filters = 0x61616161;
      redBayerPixel[0][0] = false;
      redBayerPixel[0][1] = true;
      redBayerPixel[1][0] = false;
      redBayerPixel[1][1] = false;

      greenBayerPixel[0][0] = true;
      greenBayerPixel[0][1] = false;
      greenBayerPixel[1][0] = false;
      greenBayerPixel[1][1] = true;

    } else if (bayerPattern.find("GBRG") != std::string::npos) {
      filters = 0x49494949;
      redBayerPixel[0][0] = false;
      redBayerPixel[0][1] = false;
      redBayerPixel[1][0] = true;
      redBayerPixel[1][1] = false;

      greenBayerPixel[0][0] = true;
      greenBayerPixel[0][1] = false;
      greenBayerPixel[1][0] = false;
      greenBayerPixel[1][1] = true;
    } else if (bayerPattern.find("BGGR") != std::string::npos) {
      filters = 0x16161616;
      redBayerPixel[0][0] = false;
      redBayerPixel[0][1] = false;
      redBayerPixel[1][0] = false;
      redBayerPixel[1][1] = true;

      greenBayerPixel[0][0] = false;
      greenBayerPixel[0][1] = true;
      greenBayerPixel[1][0] = true;
      greenBayerPixel[1][1] = false;
    }

    vignetteCurveH.clearPoints();
    for (auto p : vignetteRollOffH) {
      vignetteCurveH.addPoint(p);
    }

    vignetteCurveV.clearPoints();
    for (auto p : vignetteRollOffV) {
      vignetteCurveV.addPoint(p);
    }

    // If saturation is unit this satMat will be the identity matrix.
    Mat satMat = Mat::zeros(3, 3, CV_32F);
    satMat.at<float>(0, 0) = 1.0f;
    satMat.at<float>(1, 1) = saturation;
    satMat.at<float>(2, 2) = saturation;

    // Move into yuv scale by the saturation and move back
    satMat = yuv2rgb * satMat * rgb2yuv;

    transpose(ccm, compositeCCM);
    compositeCCM *= satMat;

    // The stage following the CCM maps tone curve lut to 12bits so we
    // scale the pixel by the lut size here once instead of doing it
    // for every pixel.
    compositeCCM *= float(kToneCurveLutSize - 1);

    // Build the tone curve table
    buildToneCurveLut();
  }

  // Helper functions
  inline bool redPixel(const int i, const int j) const {
    return redBayerPixel[i % 2][j % 2];
  }

  inline bool greenPixel(const int i, const int j) const {
    return greenBayerPixel[i % 2][j % 2];
  }

  inline bool bluePixel(const int i, const int j) const {
    return !(greenBayerPixel[i % 2][j % 2] || redBayerPixel[i % 2][j % 2]);
  }

  inline int getChannelNumber(const int i, const int j) {
    return redPixel(i, j) ? 0 : greenPixel(i, j) ? 1 : 2;
  }

  inline Vec3f curveHAtPixel(const int x) {
    return vignetteCurveH(float(x) / float(maxDimension));
  }

  inline Vec3f curveVAtPixel(const int x) {
    return vignetteCurveV(float(x) / float(maxDimension));
  }

  void dumpConfigFile(const string configFileName) {
    ofstream ofs(configFileName.c_str(), ios::out);
    if (ofs) {
      ofs.precision(3);
      ofs << fixed;
      ofs << "{\n";
      ofs << "   \"CameraIsp\" : {\n";
      ofs << "        \"serial\" : 0,\n";
      ofs << "        \"name\" : \"PointGrey Grasshopper\",\n";
      ofs << "        \"bitsPerPixel\" : " << bitsPerPixel << ",\n";
      ofs << "        \"compandingLut\" :  [";
      for (int i = 0; i < compandingLut.size(); ++i) {
        if (i > 0) {
          ofs << "                            ";
        }
        ofs << "["
            << compandingLut[i].x << ", "
            << compandingLut[i].y << ", "
            << compandingLut[i].z << "]";
        if (i < compandingLut.size()-1) {
          ofs << ",\n";
        }
      }
      ofs << "],\n";
      ofs << "        \"blackLevel\" : ["
          << blackLevel.x << ", "
          << blackLevel.y << ", "
          << blackLevel.z << "],\n";
      ofs << "        \"clampMin\" : ["
          << clampMin.x << ", "
          << clampMin.y << ", "
          << clampMin.z << "],\n";
      ofs << "        \"clampMax\" : ["
          << clampMax.x << ", "
          << clampMax.y << ", "
          << clampMax.z << "],\n";
      ofs << "        \"vignetteRollOffH\" :  [";
      for (int i = 0; i < vignetteRollOffH.size(); ++i) {
        if (i > 0) {
          ofs << "                               ";
        }
        ofs << "["
            << vignetteRollOffH[i].x << ", "
            << vignetteRollOffH[i].y << ", "
            << vignetteRollOffH[i].z << "]";
        if (i < vignetteRollOffH.size() - 1) {
          ofs << ",\n";
        }
      }
      ofs << "],\n";
      ofs << "        \"vignetteRollOffV\" :  [";
      for (int i = 0; i < vignetteRollOffV.size(); ++i) {
        if (i > 0) {
          ofs << "                               ";
        }
        ofs << "["
            << vignetteRollOffV[i].x << ", "
            << vignetteRollOffV[i].y << ", "
            << vignetteRollOffV[i].z << "]";
        if (i < vignetteRollOffV.size() - 1) {
          ofs << ",\n";
        }
      }
      ofs << "],\n";
      ofs << "        \"whiteBalanceGain\" : ["
          << whiteBalanceGain.x << ","
          << whiteBalanceGain.y << ","
          << whiteBalanceGain.z << "],\n";
      ofs << "        \"stuckPixelThreshold\" : " << stuckPixelThreshold << ",\n";
      ofs << "        \"stuckPixelDarknessThreshold\" : " << stuckPixelDarknessThreshold<< ",\n";
      ofs << "        \"stuckPixelRadius\" : " << stuckPixelRadius << ",\n";
      ofs.precision(5);
      ofs << fixed;
      ofs << "        \"ccm\" : [["
          << ccm.at<float>(0,0) << ", "
          << ccm.at<float>(0,1) << ", "
          << ccm.at<float>(0,2) << "],\n";
      ofs << "                 ["
          << ccm.at<float>(1,0) << ", "
          << ccm.at<float>(1,1) << ", "
          << ccm.at<float>(1,2) << "],\n";
      ofs << "                 ["
          << ccm.at<float>(2,0) << ", "
          << ccm.at<float>(2,1) << ", "
          << ccm.at<float>(2,2) << "]],\n";
      ofs.precision(3);
      ofs << fixed;
      ofs << "        \"sharpening\" : ["
          << sharpening.x << ", "
          << sharpening.y << ", "
          << sharpening.z << "],\n";
      ofs << "        \"saturation\" : " << saturation << ",\n";
      ofs << "        \"contrast\" : " << contrast << ",\n";
      ofs << "        \"lowKeyBoost\" : ["
          << lowKeyBoost.x << ", "
          << lowKeyBoost.y << ", "
          << lowKeyBoost.z << "],\n";
      ofs << "        \"highKeyBoost\" : ["
          << highKeyBoost.x << ", "
          << highKeyBoost.y << ", "
          << highKeyBoost.z << "],\n";
      ofs << "        \"gamma\" : ["
          << gamma.x << ", "
          << gamma.y << ", "
          << gamma.z << "],\n";
      ofs << "        \"bayerPattern\" : \"" << bayerPattern << "\"\n";
      ofs << "    }\n";
      ofs << "}\n";
      ofs.close();
    } else {
      throw VrCamException("unable to open output ISP config file: " + configFileName);
    }
  }

  virtual void loadImage(const Mat& inputImage) {
    *const_cast<int*>(&width) = inputImage.cols / resize;
    *const_cast<int*>(&height) = inputImage.rows / resize;
    *const_cast<int*>(&maxDimension) = std::max(width, height);
    *const_cast<float*>(&maxD) = square(width) + square(width);
    *const_cast<float*>(&sqrtMaxD) = sqrt(maxD);

    rawImage = Mat::zeros(height, width, CV_32F);

    // Copy and convert to float
    uint8_t depth = inputImage.type() & CV_MAT_DEPTH_MASK;
    // Match the input bits per pixel overriding what is in the config file.
    if (depth == CV_8U) {
      bitsPerPixel = 8;
      resizeInput<uint8_t>(inputImage);
    } else if (depth == CV_16U) {
      bitsPerPixel = 16;
      maxPixelValue = (1 << bitsPerPixel) - 1;

      resizeInput<uint16_t>(inputImage);
    } else {
      throw VrCamException("input is larger that 16 bits per pixel");
    }
  }

  int getBitsPerPixel() const {
    return bitsPerPixel;
  }

  void setBitsPerPixel(const int bitsPerPixel)  {
    this->bitsPerPixel = bitsPerPixel;
  }

  int getMaxPixelValue() const {
    return maxPixelValue;
  }

  void setRawImage(const Mat& rawImage) {
    this->rawImage = rawImage;
  }

  Mat getRawImage() const {
    return rawImage;
  }

  void setDemosaicedImage(const Mat& demosaicedImage) {
    this->demosaicedImage = demosaicedImage;
  }

  Mat getDemosaicedImage() const {
    return demosaicedImage;
  }

  void addBlackLevelOffset(const int offset) {
    blackLevel.x += offset;
    blackLevel.y += offset;
    blackLevel.z += offset;
  }

  void setBlackLevel(const Point3f& blackLevel) {
    this->blackLevel = blackLevel;
  }

  Point3f getBlackLevel() const {
    return blackLevel;
  }

  void setClampMin(const Point3f& clampMin) {
    this->clampMin = clampMin;
  }

  Point3f getClampMin() const {
    return clampMin;
  }

  void setClampMax(const Point3f& clampMax) {
    this->clampMax = clampMax;
  }

  Point3f getClampMax() const {
    return clampMax;
  }

  void setVignetteRollOffH(const vector<Point3f>& vignetteRollOffH) {
    this->vignetteRollOffH = vignetteRollOffH;
  }

  vector<Point3f> getVignetteRollOffH() const {
    return vignetteRollOffH;
  }

  void setVignetteRollOffV(const vector<Point3f>& vignetteRollOffV) {
    this->vignetteRollOffV = vignetteRollOffV;
  }

  vector<Point3f> getVignetteRollOffV() const {
    return vignetteRollOffV;
  }

  void setCCM(const Mat& ccm) {
    this->ccm = ccm;
  }

  Mat getCCM() const {
    return ccm;
  }

  uint32_t getFilters() const {
    return filters;
  }

  void setWhiteBalance(const Point3f whiteBalanceGain ) {
    this->whiteBalanceGain = whiteBalanceGain;
  }

  Point3f getWhiteBalanceGain() const {
    return whiteBalanceGain;
  }

  void setGamma(const Point3f gamma) {
    this->gamma = gamma;
  }

  Point3f getGamma() const {
    return gamma;
  }

  void enableToneMap() {
    disableToneCurve = false;
    buildToneCurveLut();
  }

  void disableToneMap() {
    disableToneCurve = true;
    buildToneCurveLut();
  }

  void setDemosaicFilter(const int demosaicFilter) {
    if (0 <= demosaicFilter && demosaicFilter < int(LAST_DM_FILTER)) {
      this->demosaicFilter = static_cast<DemosaicFilter>(demosaicFilter);
    } else {
      throw VrCamException(
        "expecting Demosaic filter in [0," +
        std::to_string(int(LAST_DM_FILTER) - 1) + "]");
    }
  }

  void setResize(const int resize) {
    if (resize == 1 ||
        resize == 2 ||
        resize == 4 ||
        resize == 8) {
      this->resize = resize;
    } else {
      throw VrCamException(
        "expecting a resize value of 1, 2, 4, or 8. got " + std::to_string(resize));
    }
  }

  // If the sensor isn't linear we need to make it linear with a look up table
  void linearize() {
    Linear clut(0.0f, 1.0f, maxPixelValue*2, compandingLut);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        float& v = rawImage.at<float>(i, j);
        if (redPixel(i, j)) {
          v = clut(v);
        }
      }
    }
  }

  // Set the white point of the camera/scene
  void whiteBalance(const bool clampOutput = true) {
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        if (redPixel(i, j)) {
          rawImage.at<float>(i, j) *= whiteBalanceGain.x;
        } else if (greenPixel(i, j)) {
          rawImage.at<float>(i, j) *= whiteBalanceGain.y;
        } else {
          rawImage.at<float>(i, j) *= whiteBalanceGain.z;
        }

        if (clampOutput) {
          rawImage.at<float>(i, j) = clamp(rawImage.at<float>(i, j), 0.0f, 1.0f);
        }
      }
    }
  }


  void removeStuckPixels() {
    if (stuckPixelRadius > 0) {
      struct Pval {
        float val;
        int i;
        int j;

        Pval(float v) :
            val(v) {
        }

        bool operator <(const Pval& p) const {
          return val < p.val;
        }

        Pval& operator=(const Pval& p) {
          val = p.val;
          i = p.i;
          j = p.j;
          return *this;
        }
      };

      vector<Pval> region;

      for (int i = 0; i < height; ++i) {
        // Traverse boustrophedonically
        const bool evenScanLine = (i % 2) == 0;
        const int jStart = evenScanLine ? 0       : width - 1;
        const int jEnd   = evenScanLine ? width-1 : 0;
        const int jStep  = evenScanLine ? 1       : -1;

        for (int j = jStart; j != jEnd; j += jStep) {
          const bool thisPixelRed = redPixel(i, j);
          const bool thisPixelGreen = greenPixel(i, j);
          const bool thisPixelBlue = bluePixel(i, j);

          region.clear();

          float mean = 0.0f;
          for (int y = -stuckPixelRadius; y <= stuckPixelRadius; y++) {
            const int ip = reflect(i + y, height);
            for (int x = -stuckPixelRadius; x <= stuckPixelRadius; x++) {
              const int jp = reflect(j + x, width);
              Pval p(rawImage.at<float>(ip, jp));
              p.i = ip;
              p.j = jp;
              if (redPixel(ip, jp) && thisPixelRed) {
                mean += p.val;
                region.push_back(p);
              } else if (greenPixel(ip, jp) && thisPixelGreen) {
                mean += p.val;
                region.push_back(p);
              } else if (bluePixel(ip, jp) && thisPixelBlue){
                mean += p.val;
                region.push_back(p);
              }
            }
          }
          mean /= float(region.size());

          // Only deal with dark regions
          if (mean < stuckPixelDarknessThreshold) {
            sort(region.begin(), region.end());

            // See if the middle pixel is above the stack at pixel threshold and an outlier
            for (int k = region.size() - 1;
                 k <= region.size() - stuckPixelThreshold;
                 k--) {
              if (region[k].i == i && region[k].j == j) {
                rawImage.at<float>(i, j) = region[region.size()/2].val;
                goto l0;
              }
            }
          }
          l0: continue;
        }
      }
    }
  }


  void blackLevelAdjust() {
    const float br = blackLevel.x / float(maxPixelValue);
    const float bg = blackLevel.y / float(maxPixelValue);
    const float bb = blackLevel.z / float(maxPixelValue);
    const float sr = 1.0f / (1.0f - br);
    const float sg = 1.0f / (1.0f - bg);
    const float sb = 1.0f / (1.0f - bb);
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        if (rawImage.at<float>(i, j) < 1.0f) {
          if (redPixel(i, j)) {
            rawImage.at<float>(i, j) = (rawImage.at<float>(i, j) - br) * sr;
          } else if (greenPixel(i, j)) {
            rawImage.at<float>(i, j) = (rawImage.at<float>(i, j) - bg) * sg;
          } else {
            rawImage.at<float>(i, j) = (rawImage.at<float>(i, j) - bb) * sb;
          }
        }
      }
    }
  }

  void clampAndStretch() {
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        const float clampMinVal = redPixel(i, j)
          ? clampMin.x
          : (greenPixel(i, j) ? clampMin.y : clampMin.z);
        const float clampMaxVal = redPixel(i, j)
          ? clampMax.x
          : (greenPixel(i, j) ? clampMax.y : clampMax.z);
        const float v =
          clamp(rawImage.at<float>(i, j), clampMinVal, clampMaxVal);
        rawImage.at<float>(i, j) =
          (v - clampMinVal) / (clampMaxVal - clampMinVal);
      }
    }
  }

  void antiVignette() {
    for (int i = 0; i < height; ++i) {
      const Vec3f vV = curveVAtPixel(i);
      for (int j = 0; j < width; j++) {
        const Vec3f vH = curveHAtPixel(j);
        int ch = getChannelNumber(i, j);
        rawImage.at<float>(i, j) *= vH[ch] * vV[ch];
      }
    }
  }

  void demosaic() {
    Mat r(height, width, CV_32F);
    Mat g(height, width, CV_32F);
    Mat b(height, width, CV_32F);

    // Break out each plane into a separate image so we can demosaicFilter
    // them seperately and then recombine them.
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        if (redPixel(i, j)) {
          r.at<float>(i, j) = rawImage.at<float>(i, j);
        } else if (greenPixel(i, j)) {
          g.at<float>(i, j) = rawImage.at<float>(i, j);
        } else {
          b.at<float>(i,j) = rawImage.at<float>(i, j);
        }
      }
    }

    if (demosaicFilter == FREQUENCY_DM_FILTER) {
      // Move into the frequency domain
      thread t1([&]{dct(r, r);});
      thread t2([&]{dct(g, g);});
      thread t3([&]{dct(b, b);});

      t1.join();
      t2.join();
      t3.join();

      // Filter including sharpnning in the DCT domain
      demosaicFrequencyFilter(r, g, b);
#undef  DEBUG_DCT
#ifndef DEBUG_DCT
      // Move back into the spatial domain
      thread t4([&]{idct(r, r);});
      thread t5([&]{idct(g, g);});
      thread t6([&]{idct(b, b);});

      t4.join();
      t5.join();
      t6.join();
#endif
    } else if (demosaicFilter == BILINEAR_DM_FILTER) {
      demosaicBilinearFilter(r, g, b);
    } else {
      demosaicEdgeAware(r, g, b);
   }

    demosaicedImage = Mat::zeros(height, width, CV_32FC3);
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        demosaicedImage.at<Vec3f>(i, j)[0] = r.at<float>(i, j);
        demosaicedImage.at<Vec3f>(i, j)[1] = g.at<float>(i, j);
        demosaicedImage.at<Vec3f>(i, j)[2] = b.at<float>(i, j);
      }
    }
  }

  void colorCorrect() {
    const float kToneCurveLutRange = kToneCurveLutSize - 1;
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        Vec3f p =  demosaicedImage.at<Vec3f>(i, j);
#ifdef DEBUG_DCT
        Vec3f v(logf(square(p[0]) + 1.0f) * 255.0f, logf(square(p[1]) + 1.0f) * 255.0f, logf(square(p[2]) + 1.0f) * 255.0f);
#else
        Vec3f v(
          toneCurveLut[
              clamp(
                  compositeCCM.at<float>(0,0) * p[0] +
                  compositeCCM.at<float>(0,1) * p[1] +
                  compositeCCM.at<float>(0,2) * p[2], 0.0f, kToneCurveLutRange)][0],
          toneCurveLut[
              clamp(
                  compositeCCM.at<float>(1,0) * p[0] +
                  compositeCCM.at<float>(1,1) * p[1] +
                  compositeCCM.at<float>(1,2) * p[2], 0.0f, kToneCurveLutRange)][1],
          toneCurveLut[
              clamp(
                  compositeCCM.at<float>(2,0) * p[0] +
                  compositeCCM.at<float>(2,1) * p[1] +
                  compositeCCM.at<float>(2,2) * p[2], 0.0f, kToneCurveLutRange)][2]);
#endif
        demosaicedImage.at<Vec3f>(i, j) = v;
      }
    }
  }

  void sharpen() {
    if (sharpening.x != 0.0 && sharpening.y != 0.0 && sharpening.z != 0.0) {
      Mat lowPass(height, width, CV_32FC3);
      const ReflectBoundary<int> reflectB;
      const float maxVal = (1 << outputBpp) - 1.0f;
      iirLowPass<ReflectBoundary<int>, ReflectBoundary<int>, Vec3f>(demosaicedImage, sharpeningSupport, lowPass, reflectB, reflectB, maxVal);
      sharpenWithIirLowPass<Vec3f>(demosaicedImage,
          lowPass,
          1.0f + sharpening.x,
          1.0f + sharpening.y,
          1.0f + sharpening.z,
          noiseCore,
          maxVal);
    }
  }

 protected:
  // Replacable pipeline
  virtual void executePipeline(const bool swizzle) {
    // Apply the pipeline
    blackLevelAdjust();
    antiVignette();
    whiteBalance();
    clampAndStretch();
    removeStuckPixels();
    demosaic();
    colorCorrect();
    sharpen();
  }

 public:
  virtual void getImage(Mat& outputImage, const bool swizzle = true) {
    executePipeline(swizzle);

    const float range = float((1 << outputBpp) - 1);

    // Copy and convert to byte swizzling to BGR
    const int c0 = swizzle ? 2 : 0;
    const int c1 = swizzle ? 1 : 1;
    const int c2 = swizzle ? 0 : 2;
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        Vec3f& dp = demosaicedImage.at<Vec3f>(i, j);
        if (outputBpp == 8) {
          outputImage.at<Vec3b>(i, j)[c0] = dp[0];
          outputImage.at<Vec3b>(i, j)[c1] = dp[1];
          outputImage.at<Vec3b>(i, j)[c2] = dp[2];
        } else {
          outputImage.at<Vec3s>(i, j)[c0] = dp[0];
          outputImage.at<Vec3s>(i, j)[c1] = dp[1];
          outputImage.at<Vec3s>(i, j)[c2] = dp[2];
        }
      }
    }
  }
};

} // end namespace surround360
