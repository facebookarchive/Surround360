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

class CameraIsp {
 protected :
  string bayerPattern;
  vector<cv::Point3f> compandingLut;
  cv::Point3f blackLevel;
  vector<cv::Point3f>vignetteRollOff;

  int stuckPixelThreshold;
  float stuckPixelDarknessThreshold;
  int stuckPixelRadius;
  int bitsPerPixel;
  int maxPixelValue;
  cv::Point3f whiteBalanceGain;
  float denoise;
  int denoiseRadius;
  Mat ccm; // 3x3
  float saturation;
  cv::Point3f gamma;
  cv::Point3f lowKeyBoost;
  cv::Point3f highKeyBoost;
  float contrast;
  cv::Point3f sharpenning;
  Mat rawImage;
  bool redBayerPixel[2][2];
  bool greenBayerPixel[2][2];
  Mat  demosaicedImage;
  uint32_t filters;
  DemosaicFilter demosaicFilter;
  int resize;

  shared_ptr<BezierCurve<float, cv::Point3f > > vignetteCurve;

  const int width;
  const int height;
  const float halfWidth;
  const float halfHeight;
  const float maxD; // max diagonal distance
  const float sqrtMaxD; // max diagonal distance

  void denoisePlaneAdaptiveBilateral(Mat& inputImage, const int c) {
    Mat outputImage(inputImage.rows, inputImage.cols, CV_32FC1);

    outputImage = 0;
    const int radius = denoiseRadius;

    GaussianApproximation<float> gVariance(-0.001f, 0.001f, 0.0f, 1.0f);
    GaussianApproximation<float> gData(-denoise, denoise, 0.0f, 1.0f);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        int n = 0;
        float mean = 0.0f;
        float m2 = 0.0f;
        float sum = 0.0f;
        float wSum = 0.0f;
        const float currentPixel = inputImage.at<Vec3f>(i, j)[c];

        for (int ip = -radius; ip <= radius; ip++) {
          const int ipp = reflect(i + ip, height);
          for (int jp = -radius; jp <= radius; jp++) {
            const int jpp = reflect(j + jp, width);
            // Compute incremental variance
            const float val = inputImage.at<Vec3f>(ipp, jpp)[c];
            const float delta = val - mean;
            n++;
            mean += delta / n;
            m2 +=  delta * (val - mean);

            // Sum the bilateral filter
            const float w = gData(val - currentPixel);
            sum += val * w;
            wSum += w;
          }
        }
        const float variance = m2 / (n - 1);
        const float gv = gVariance(variance);
        outputImage.at<float>(i, j) =
          (wSum > 0.0f)
          ? lerp(currentPixel, sum / wSum, gv)
          : currentPixel;
      }
    }
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        inputImage.at<Vec3f>(i, j)[c] = outputImage.at<float>(i, j);
      }
    }
  }

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
    Butterworth dFilter(0.0f, 2.0f, width + height, 0.5f, 4);
    Sinc sFilter(-4.0, 4.0,  width + height);

    for (int i = 0; i < height; ++i) {
      const float y = float(i) / float(height - 1);
      const float y2 = y / 2.0f;
      for (int j = 0; j < width; ++j) {
        const float x = float(j) / float(width - 1);
        const float x2 = x / 2.0f;
        const float d = sqrt(square(x2) + square(y2) - square(x2 + y2)/2);
        const float scale = (sFilter(lerp(1.0f, 0.0f, d))*10 + 1.0f) / 1.5f;
        const float gGain = dFilter(x2 + y2) * scale;
        const float rgGain = dFilter(x) * dFilter(y) * 2.0f / 1.5f;

        // Filter the green separately
        g.at<float>(i, j) =
          g.at<float>(i, j) * gGain +
          r.at<float>(i, j) * rgGain +
          b.at<float>(i, j) * rgGain;
        // Filter for chroma that r - g and b - g vary slowly in most
        // natural scenes.
        float dr = r.at<float>(i, j) * 4.0f - g.at<float>(i, j);
        float db = b.at<float>(i, j) * 4.0f - g.at<float>(i, j);
        const float cGain = dFilter(x * 2.0f) * dFilter(y * 2.0f);

        dr *= cGain;
        db *= cGain;

        r.at<float>(i, j) = dr + g.at<float>(i, j);
        b.at<float>(i, j) = db + g.at<float>(i, j);
      }
    }
  }


  void demosaicEdgeAware(Mat& red, Mat& green, Mat& blue) const {
    // Horizontal and vertical green values
    Mat gV(height, width, CV_32FC1);
    Mat gH(height, width, CV_32FC1);

    // And their first and second order derivatives
    Mat dV(height, width, CV_32FC1);
    Mat dH(height, width, CV_32FC1);

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
            fabsf(green.at<float>(i2, j) - green.at<float>(i, j)) +
            fabsf(green.at<float>(i, j) - green.at<float>(i_2, j));

          dH.at<float>(i, j) =
            fabsf(green.at<float>(i,  j2) - green.at<float>(i, j)) +
            fabsf(green.at<float>(i, j) - green.at<float>(i,  j_2));
        } else {
          gV.at<float>(i, j) = (green.at<float>(i_1, j) + green.at<float>(i1, j)) / 2.0f;
          gH.at<float>(i, j) = (green.at<float>(i, j_1) + green.at<float>(i, j1)) / 2.0f;
          dV.at<float>(i, j) = fabsf(green.at<float>(i_1, j) - green.at<float>(i1, j));
          dH.at<float>(i, j) = fabsf(green.at<float>(i, j_1) - green.at<float>(i, j1));

          Mat& ch = redPixel(i, j) ? red : blue;
          gV.at<float>(i, j) += (2.0f * ch.at<float>(i, j) - ch.at<float>(i_2, j) - ch.at<float>(i2, j)) / 4.0f;
          gH.at<float>(i, j) += (2.0f * ch.at<float>(i, j) - ch.at<float>(i, j_2) - ch.at<float>(i, j2)) / 4.0f;
          dV.at<float>(i, j) += fabsf(-2.0f * ch.at<float>(i, j) + ch.at<float>(i_2, j) + ch.at<float>(i2, j));
          dH.at<float>(i, j) += fabsf(-2.0f * ch.at<float>(i, j) + ch.at<float>(i, j_2) + ch.at<float>(i, j2));
        }
      }
    }

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        // Homogenity test
        int n = 0;
        int vCount = 0;
        int hCount = 0;
        bool isHorizontal = false;

        for (int l = -2; l <= 2; ++l) {
          const int il = reflect(i + l, height);
          for (int k = -2; k <= 2; ++k) {
            const int jk = reflect(j + k, width);
            const float grad = gH.at<float>(il, jk) + gV.at<float>(il, jk);
            if (grad * dH.at<float>(il, jk) <= grad * dV.at<float>(il, jk)) {
              hCount++;
              if (l == 0 && k == 0) {
                isHorizontal = true;
              }
            } else {
              vCount++;
            }
          }
          if (isHorizontal) {
            if (vCount < hCount) {
              green.at<float>(i, j) = gH.at<float>(i, j);
            } else {
              green.at<float>(i, j) = gV.at<float>(i, j);
            }
          } else {
            if (vCount > hCount) {
              green.at<float>(i, j) = gV.at<float>(i, j);
            } else {
              green.at<float>(i, j) = gH.at<float>(i, j);
            }
          }
        }
      }
    }

    // compute r-b
    Mat redMinusGreen(height, width, CV_32FC1);
    Mat blueMinusGreen(height, width, CV_32FC1);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        if (redPixel(i, j)) {
          redMinusGreen.at<float>(i, j) = red.at<float>(i, j) - green.at<float>(i, j);
        } else if (!greenPixel(i, j)) {
          blueMinusGreen.at<float>(i, j) = blue.at<float>(i, j) - green.at<float>(i, j);
        }
      }
    }

    // Now use a constant hue based red/blue bilinear interopaltion
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
             blueMinusGreen.at<float>(i1, j1)) / 4 +
            green.at<float>(i, j);
        } else if (greenPixel(i, j)) {
          Mat& diffCh1 = redGreenRow ? blueMinusGreen : redMinusGreen;
          Mat& diffCh2 = redGreenRow ? redMinusGreen :  blueMinusGreen;

          Mat& ch1 = redGreenRow ? blue : red;
          Mat& ch2 = redGreenRow ? red :  blue;

          ch1.at<float>(i, j) =
            (diffCh1.at<float>(i_1, j) +
             diffCh1.at<float>(i1, j) +
             diffCh1.at<float>(i_1, j_2) +
             diffCh1.at<float>(i1, j_2) +
             diffCh1.at<float>(i_1, j2) +
             diffCh1.at<float>(i1, j2)) / 6 +
            green.at<float>(i, j);

          ch2.at<float>(i, j) =
            (diffCh2.at<float>(i, j_1) +
             diffCh2.at<float>(i, j1) +
             diffCh2.at<float>(i_2, j_1) +
             diffCh2.at<float>(i2, j_1) +
             diffCh2.at<float>(i_2, j1) +
             diffCh2.at<float>(i2, j1)) / 6 +
            green.at<float>(i, j);
        } else {
          red.at<float>(i, j) =
            (redMinusGreen.at<float>(i_1, j_1) +
             redMinusGreen.at<float>(i1, j_1) +
             redMinusGreen.at<float>(i_1, j1) +
             redMinusGreen.at<float>(i1, j1)) / 4.0 +
            green.at<float>(i, j);
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

 public:
  CameraIsp(string jsonInput) :
      demosaicFilter(EDGE_AWARE_DM_FILTER),
      resize(1),
      width(0),
      height(0),
      halfWidth(0),
      halfHeight(0),
      maxD(0),
      sqrtMaxD(0) {

    const json::Object config = json::Deserialize(jsonInput);

    // Set the default values and override them from the json file
    bitsPerPixel = 8;
    maxPixelValue = (1 << bitsPerPixel) - 1;
    compandingLut.push_back(Point3f(0.0f, 0.0f, 0.0f));
    compandingLut.push_back(Point3f(1.0f, 1.0f, 1.0f));
    blackLevel = Point3f(0.0f, 0.0f, 0.0f);
    stuckPixelThreshold = 0;
    stuckPixelDarknessThreshold = 0;
    stuckPixelRadius = 0;
    vignetteRollOff.push_back(Point3f(0.0f, 0.0f, 0.0f));
    vignetteRollOff.push_back(Point3f(1.0f, 1.0f, 1.0f));
    whiteBalanceGain = Point3f(1.0f, 1.0f, 1.0f);
    denoise = 0.0;
    denoiseRadius = 0;
    ccm = Mat::eye(3, 3, CV_32F);
    saturation = 1.0f;
    contrast = 1.0f;
    sharpenning = Point3f(0.0f, 0.0f, 0.0f);
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

      if (config["CameraIsp"].HasKey("vignetteRollOff")) {
        vignetteRollOff = getCoordList(config, "CameraIsp", "vignetteRollOff");
      } else {
        VLOG(1) << "Using default vignetteRollOff = " << vignetteRollOff << endl;
      }


      if (config["CameraIsp"].HasKey("whiteBalanceGain")) {
        whiteBalanceGain = getVector(config, "CameraIsp", "whiteBalanceGain");
      } else {
        VLOG(1) << "Using default whiteBalanceGain = " << whiteBalanceGain << endl;
      }


      if (config["CameraIsp"].HasKey("denoise")) {
        denoise = getDouble(config, "CameraIsp", "denoise");
      } else {
        VLOG(1) << "Using default denoise = " << denoise << endl;
      }

      if (config["CameraIsp"].HasKey("denoiseRadius")) {
        denoiseRadius = getInteger(config, "CameraIsp", "denoiseRadius");
      } else {
        VLOG(1) << "Using default denoiseRadius = " << denoiseRadius << endl;
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

      if (config["CameraIsp"].HasKey("sharpenning")) {
        sharpenning = getVector(config, "CameraIsp", "sharpenning");
      } else {
        VLOG(1) << "Using default sharpenning = " << sharpenning << endl;
      }

      if (config["CameraIsp"].HasKey("bayerPattern")) {
        bayerPattern = getString(config, "CameraIsp", "bayerPattern");
      } else {
        VLOG(1) << "Using default bayerPattern = " << bayerPattern << endl;
      }
    } else {
      VLOG(1) << "Missing or \"CameraIsp\" using defaults.\n";
    }

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

    vignetteCurve = shared_ptr<BezierCurve<float, cv::Point3f > > (
      new BezierCurve<float, cv::Point3f >());

    for (int i = 0; i < vignetteRollOff.size(); ++i) {
      cv::Point3f v(vignetteRollOff[i]);
      vignetteCurve->addPoint(v);
    }

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
      ofs << "        \"vignetteRollOff\" : [";
      for (int i = 0; i < vignetteRollOff.size(); ++i) {
        if (i > 0) {
          ofs << "                             ";
        }
        ofs << "["
            << vignetteRollOff[i].x << ", "
            << vignetteRollOff[i].y << ", "
            << vignetteRollOff[i].z << "]";
        if (i < vignetteRollOff.size()-1) {
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
      ofs << "        \"denoise\" : " << denoise << ",\n";
      ofs << "        \"denoiseRadius\" : " << denoiseRadius << ",\n";
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
      ofs << "        \"sharpenning\" : ["
          << sharpenning.x << ", "
          << sharpenning.y << ", "
          << sharpenning.z << "],\n";
      ofs << "        \"saturation\" : " << saturation << ",\n";
      ofs << "        \"contrast\" : " << contrast << ",\n";
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

  void loadImage(const Mat& inputImage) {
    *const_cast<int*>(&width) = inputImage.cols / resize;
    *const_cast<int*>(&height) = inputImage.rows / resize;

    *const_cast<float*>(&halfWidth) = width / 2.0f;
    *const_cast<float*>(&halfHeight) = height / 2.0f;
    *const_cast<float*>(&maxD) = square(width) + square(width);
    *const_cast<float*>(&sqrtMaxD) = sqrt(maxD);

    rawImage = Mat::zeros(height, width, CV_32FC1);

    // Copy and convert to float
    if (bitsPerPixel <= 8) {
      resizeInput<uint8_t>(inputImage);
    } else if (bitsPerPixel <= 16) {
      resizeInput<uint16_t>(inputImage);
    } else {
      throw VrCamException("input is larger that 16 bits per pixel");
    }
  }

  int getMaxPixelValue() const {
    return maxPixelValue;
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

  void setCCM(const Mat& ccm) {
    this->ccm = ccm;
  }

  Mat getCCM() const {
    return ccm;
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
    Linear cLut(0.0f, 1.0f, maxPixelValue*2, compandingLut);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        float& v = rawImage.at<float>(i, j);
        v = cLut(v);
      }
    }
  }

  // If the sensor isn't linear we need to make it linear with a look up table
  void whiteBalance() {
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        if (redPixel(i, j)) {
          rawImage.at<float>(i, j) =
            clamp(rawImage.at<float>(i, j) * whiteBalanceGain.x, 0.0f, 1.0f);
        } else if (greenPixel(i, j)) {
          rawImage.at<float>(i, j) =
            clamp(rawImage.at<float>(i, j) * whiteBalanceGain.y, 0.0f, 1.0f);
        } else {
          rawImage.at<float>(i, j) =
            clamp(rawImage.at<float>(i, j) * whiteBalanceGain.z, 0.0f, 1.0f);
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


  void demosiac() {
    Mat r(height, width, CV_32FC1);
    Mat g(height, width, CV_32FC1);
    Mat b(height, width, CV_32FC1);

    // Break out each plane into a separate image so we can demosiacFilter
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

  void denoiseFilter() {
    // Disable if denoising factor is zero
    if (denoise != 0.0f) {
      Mat Yuv(height, width, CV_32FC3);
      cvtColor(demosaicedImage, Yuv, CV_RGB2YCrCb);

      thread t2([&]{denoisePlaneAdaptiveBilateral(Yuv, 1);});
      thread t3([&]{denoisePlaneAdaptiveBilateral(Yuv, 2);});
      t2.join();
      t3.join();

      cvtColor(Yuv, demosaicedImage, CV_YCrCb2RGB);
    }
  }

  void colorCorrect() {
    // If saturation is unit this satImage will be the identity matrix.
    Mat satImage = Mat::zeros(3, 3, CV_32FC1);
    satImage.at<float>(0, 0) = 1.0f;
    satImage.at<float>(1, 1) = saturation;
    satImage.at<float>(2, 2) = saturation;

    // Move into yuv scale by the saturation
    satImage = yuv2rgb * satImage * rgb2yuv ;

    Mat colorImage;
    transpose(ccm, colorImage); // TODO: this could probably be optimized...
    colorImage *= satImage;
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        Vec3f v;
        v[0] = clamp(
          colorImage.at<float>(0,0) * demosaicedImage.at<Vec3f>(i, j)[0] +
          colorImage.at<float>(0,1) * demosaicedImage.at<Vec3f>(i, j)[1] +
          colorImage.at<float>(0,2) * demosaicedImage.at<Vec3f>(i, j)[2], 0.0f, 1.0f);
        v[1] = clamp(
          colorImage.at<float>(1,0) * demosaicedImage.at<Vec3f>(i, j)[0] +
          colorImage.at<float>(1,1) * demosaicedImage.at<Vec3f>(i, j)[1] +
          colorImage.at<float>(1,2) * demosaicedImage.at<Vec3f>(i, j)[2], 0.0f, 1.0f);
        v[2] = clamp(
          colorImage.at<float>(2,0) * demosaicedImage.at<Vec3f>(i, j)[0] +
          colorImage.at<float>(2,1) * demosaicedImage.at<Vec3f>(i, j)[1] +
          colorImage.at<float>(2,2) * demosaicedImage.at<Vec3f>(i, j)[2], 0.0f, 1.0f);

        demosaicedImage.at<Vec3f>(i, j) = v;
      }
    }
  }

  void antiVignette() {
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        const float d = sqrt(square(j - halfWidth) + square(i - halfHeight));
        const cv::Point3f v = (*vignetteCurve)(2 * d / sqrtMaxD);
        demosaicedImage.at<Vec3f>(i, j)[0] *= v.x;
        demosaicedImage.at<Vec3f>(i, j)[1] *= v.y;
        demosaicedImage.at<Vec3f>(i, j)[2] *= v.z;
      }
    }
  }

  void gammaCorrect() {
    Power rGamma(0.0f, 1.0f, maxPixelValue * 4, gamma.x);
    Power gGamma(0.0f, 1.0f, maxPixelValue * 4, gamma.y);
    Power bGamma(0.0f, 1.0f, maxPixelValue * 4, gamma.z);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        Vec3f& pixel = demosaicedImage.at<Vec3f>(i, j);
        pixel[0] = rGamma(pixel[0]);
        pixel[1] = gGamma(pixel[1]);
        pixel[2] = bGamma(pixel[2]);
      }
    }
  }

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
    return x > 0.5f ? bezier(a, b, c, d, (x - 0.5) * 2.0f) : 0;
  }

  inline float lowKey(float lowKeyBoost, float x) {
    const float a = 0.0f;
    const float b = clamp(0.1666f + lowKeyBoost, 0.0f, 1.0f);
    const float c = clamp(0.3333f, 0.0f, 1.0f);
    const float d = 0.5f;
    return x <= 0.5f ? bezier(a, b, c, d, x * 2.0f) : 0;
  }

  void toneCurve() {
    // Boost high and low parts of the pixel space
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        Vec3f& pixel = demosaicedImage.at<Vec3f>(i, j);
        pixel[0] = lowKey(lowKeyBoost.x, pixel[0]) + highKey(highKeyBoost.x, pixel[0]);
        pixel[1] = lowKey(lowKeyBoost.y, pixel[1]) + highKey(highKeyBoost.y, pixel[1]);
        pixel[2] = lowKey(lowKeyBoost.z, pixel[2]) + highKey(highKeyBoost.z, pixel[2]);
      }
    }
  }

  void contrastCurve() {
    // Contrast is just sigmoid curve about the middle of the color space
    if (contrast != 1.0f) {
      const float range = 1.0f / (5.0f * (contrast - 1.0f));
      sCurve contrastMap(0.5f - range, 0.5f + range,
          0.0f, 1.0f, maxPixelValue * 4);
      for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; j++) {
          Vec3f& pixel = demosaicedImage.at<Vec3f>(i, j);
          pixel[0] = contrastMap(pixel[0]);
          pixel[1] = contrastMap(pixel[1]);
          pixel[2] = contrastMap(pixel[2]);
        }
      }
    }
  }

void sharpen() {
    Mat lowPass(height, width, CV_32FC3);
    const WrapBoundary<float> wrapB;
    iirLowPass<WrapBoundary<float>, WrapBoundary<float>, Vec3f>(demosaicedImage, 0.25f, lowPass, wrapB, wrapB, 1.0f);
    sharpenWithIirLowPass<Vec3f>(demosaicedImage,
                          lowPass,
                          1.0f + sharpenning.x,
                          1.0f + sharpenning.y,
                          1.0f + sharpenning.z);
  }

  Mat getImage(const bool swizzle = true) {
    // Apply the pipeline
    blackLevelAdjust();
    linearize();
    whiteBalance();
    removeStuckPixels();
    demosiac();
    denoiseFilter();
    colorCorrect();
    antiVignette();
    gammaCorrect();
    contrastCurve();
    toneCurve();
    sharpen();

    Mat outputImage(height, width, CV_8UC3);

    // Copy and convert to byte swizzling to BGR
    const int c0 = swizzle ? 2 : 0;
    const int c1 = swizzle ? 1 : 1;
    const int c2 = swizzle ? 0 : 2;
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; j++) {
        outputImage.at<Vec3b>(i, j)[c0] = clamp(demosaicedImage.at<Vec3f>(i, j)[0] * 255.0f, 0.0f, 255.0f);
        outputImage.at<Vec3b>(i, j)[c1] = clamp(demosaicedImage.at<Vec3f>(i, j)[1] * 255.0f, 0.0f, 255.0f);
        outputImage.at<Vec3b>(i, j)[c2] = clamp(demosaicedImage.at<Vec3f>(i, j)[2] * 255.0f, 0.0f, 255.0f);
      }
    }
    return outputImage;
  }
};

} // end namespace surround360
