/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "Halide.h"

#include <gflags/gflags.h>

DEFINE_int32(output_bpp, 8,  "output image bits per pixel, either 8 or 16");

using namespace std;
using namespace Halide;

template<bool Fast>
class CameraIspGen
{
 protected:
  Target target;
  Var x, y, c, yi, yo;
  Func toneCorrected;
  Func ispOutput;

  // Average two positive values rounding up
  Expr avg(Expr a, Expr b) {
    return (a + b) * 0.5f;
  }

  Func interleave_x(Func a, Func b) {
    Func out("ix");
    out(x, y) = select((x % 2) == 0, a(x, y), b(x, y));
    return out;
  }

  Func interleave_y(Func a, Func b) {
    Func out("iy");
    out(x, y) = select((y % 2) == 0, a(x, y), b(x, y));
    return out;
  }


  Expr redPixel(Expr bayerPattern) {
    return select(bayerPattern == 0, (x % 2) == 0 && (y % 2) == 1,
    select(bayerPattern == 1, (x % 2) == 0 && (y % 2) == 0, 0));
  }

  Expr greenRedPixel(Expr bayerPattern) {
    return select(bayerPattern == 0, (x % 2) == 1 && (y % 2) == 1,
    select(bayerPattern == 1, (x % 2) == 1 && (y % 2) == 0, 0));
  }


  Expr bluePixel(Expr bayerPattern) {
    return select(bayerPattern == 0, (x % 2) == 1 && (y % 2) == 0,
    select(bayerPattern == 1, (x % 2) == 1 && (y % 2) == 1, 0));
   }

  Expr greenBluePixel(Expr bayerPattern) {
    return select(bayerPattern == 0, (x % 2) == 0 && (y % 2) == 0,
    select(bayerPattern == 1, (x % 2) == 0 && (y % 2) == 1, 0));
  }

  Expr greenPixel(Expr bayerPattern) {
    return greenBluePixel(bayerPattern) || greenRedPixel(bayerPattern);
  }


  Func deinterleave(Func raw, Expr bayerPattern) {
    // Deinterleave the color channels
    Func deinterleaved("deinterleaved");

    // Fixed pattern of:
    // G B
    // R G
    deinterleaved(x, y, c) =
      cast<float>(
          select(c == 1 && greenBluePixel(bayerPattern), raw(x, y), // green on blue row
          select(c == 2 && bluePixel(bayerPattern),      raw(x, y), // blue
          select(c == 0 && redPixel(bayerPattern),       raw(x, y), // red
          select(c == 1 && greenRedPixel(bayerPattern),  raw(x, y), // green on red row
          0)))));

    return deinterleaved;
  }

  void bilinearDemosaic(
      Func rawRed,
      Func rawBlue,
      Func rawGreenB,
      Func rawGreenR,
      Func& redOnGreenBlue,
      Func& greenOnGreenBlue,
      Func& blueOnGreenBlue,
      Func& redOnGreenRed,
      Func& greenOnGreenRed,
      Func& blueOnGreenRed,
      Func& redOnBlue,
      Func& greenOnBlue,
      Func& blueOnBlue,
      Func& redOnRed,
      Func& greenOnRed,
      Func& blueOnRed,
      Func& demosaiced,
      Expr bayerPattern) {

    // Neighbor indices
    Expr x1 = x + 1;
    Expr x_1 = x - 1;

    Expr y1 = y + 1;
    Expr y_1 = y - 1;

    // Horizontally or vertically average the red and blue channels at
    // green-red and green-blue pixels.
    redOnGreenBlue(x, y) = avg(rawRed(x, y_1), rawRed(x,y1));
    greenOnGreenBlue(x, y) = rawGreenB(x, y);
    blueOnGreenBlue(x, y) = avg(rawBlue(x_1, y), rawBlue(x1,y));

    blueOnGreenRed(x, y) = avg(rawBlue(x, y_1), rawBlue(x,y1));
    greenOnGreenRed(x, y) = rawGreenR(x, y);
    redOnGreenRed(x, y) = avg(rawRed(x_1, y), rawRed(x1,y));

    // Box average pixels surrounding the red or blue channels when
    // they are alternatively on blue and red pixels respectively.
    redOnBlue(x, y) = avg(avg(rawRed(x_1, y_1), rawRed(x1, y_1)), avg(rawRed(x_1, y1), rawRed(x1, y1)));
    greenOnBlue(x, y) = avg(avg(rawGreenB(x_1, y), rawGreenB(x1, y)), avg(rawGreenR(x, y_1), rawGreenR(x, y1)));
    blueOnBlue(x, y) = rawBlue(x, y);

    // Average the green colors that are above, below, left and right
    // at red and blue pixels.
    redOnRed(x, y) = rawRed(x, y);
    greenOnRed(x, y) = avg(avg(rawGreenR(x_1, y), rawGreenR(x1, y)), avg(rawGreenB(x, y_1), rawGreenB(x, y1)));
    blueOnRed(x, y) = avg(avg(rawBlue(x_1, y_1), rawBlue(x1, y_1)), avg(rawBlue(x_1, y1), rawBlue(x1, y1)));

    // Interleave the resulting channels
    Func r, g, b;
    r = interleave_y(interleave_x(redOnGreenBlue,  redOnBlue),    interleave_x(redOnRed, redOnGreenRed));
    g = interleave_y(interleave_x(greenOnGreenBlue,greenOnBlue),  interleave_x(greenOnRed, greenOnGreenRed));
    b = interleave_y(interleave_x(blueOnGreenBlue, blueOnBlue),   interleave_x(blueOnRed, blueOnGreenRed));

    // RGGB
    Func r_rggb, g_rggb, b_rggb;
    r_rggb = interleave_y(interleave_x(redOnRed, redOnGreenRed), interleave_x(redOnGreenBlue,  redOnBlue));
    g_rggb = interleave_y(interleave_x(greenOnRed, greenOnGreenRed), interleave_x(greenOnGreenBlue,greenOnBlue));
    b_rggb = interleave_y(interleave_x(blueOnRed, blueOnGreenRed), interleave_x(blueOnGreenBlue, blueOnBlue));

    demosaiced(x, y, c) =
      select(bayerPattern == 0,
      select(
          c == 0, r(x, y),
          c == 1, g(x, y),
          b(x, y)),
      select(
          c == 0, r_rggb(x, y),
          c == 1, g_rggb(x, y),
          b_rggb(x, y)));
  }

  void edgeAwareDemosaic(
      Func rawRed,
      Func rawBlue,
      Func rawGreenB,
      Func rawGreenR,
      Func& redOnGreenBlue,
      Func& greenOnGreenBlue,
      Func& blueOnGreenBlue,
      Func& redOnGreenRed,
      Func& greenOnGreenRed,
      Func& blueOnGreenRed,
      Func& redOnBlue,
      Func& greenOnBlue,
      Func& blueOnBlue,
      Func& redOnRed,
      Func& greenOnRed,
      Func& blueOnRed,
      Func& demosaiced,
      Expr bayerPattern) {
    // Neighbor indices
    Expr x1 = x + 1;
    Expr x2 = x + 2;
    Expr x_1 = x - 1;
    Expr x_2 = x - 2;

    Expr y1 = y + 1;
    Expr y2 = y + 2;
    Expr y_1 = y - 1;
    Expr y_2 = y - 2;

    // Green horizonal & vertical value interpolates at green input pixels
    Func gV("gV"), gH("gH");
    gV(x, y) =
      select(greenBluePixel(bayerPattern), rawGreenB(x, y), // green on blue row
      select(bluePixel(bayerPattern),      avg(rawGreenR(x, y1), rawGreenR(x, y_1)) + (2.0f * rawBlue(x, y) - rawBlue(x, y2) - rawBlue(x, y_2)) / 4.0f, // blue
      select(redPixel(bayerPattern),       avg(rawGreenB(x, y1), rawGreenB(x, y_1)) + (2.0f * rawRed(x, y)  - rawRed(x, y2)  - rawRed(x, y_2)) / 4.0f, // red
      select(greenRedPixel(bayerPattern),  rawGreenR(x, y), // green on red row
      0))));

    gH(x, y) =
      select(greenBluePixel(bayerPattern), rawGreenB(x, y), // green on blue row
      select(bluePixel(bayerPattern),      avg(rawGreenB(x1, y), rawGreenB(x_1, y)) + (2.0f * rawBlue(x, y) - rawBlue(x2, y) - rawBlue(x_2, y)) / 4.0f, // blue
      select(redPixel(bayerPattern),       avg(rawGreenR(x1, y), rawGreenR(x_1, y)) + (2.0f * rawRed(x, y)  - rawRed(x2, y)  - rawRed(x_2, y)) / 4.0f, // red
      select(greenRedPixel(bayerPattern),  rawGreenR(x, y), // green on red row
      0))));

    // And the derivatives
    Func dV("dV"), dH("dh");
    dV(x, y) =
      select(greenBluePixel(bayerPattern), avg(absd(rawGreenB(x, y2), rawGreenB(x, y)), absd(rawGreenB(x, y_2), rawGreenB(x, y))), // green on blue row
      select(bluePixel(bayerPattern),      avg(absd(rawGreenR(x, y1), rawGreenR(x, y_1)), absd(rawBlue(x, y2) + rawBlue(x, y_2), 2.0f*rawBlue(x, y))), // blue
      select(redPixel(bayerPattern),       avg(absd(rawGreenB(x, y1), rawGreenB(x, y_1)), absd(rawRed(x, y2)  + rawRed(x, y_2),  2.0f*rawRed(x, y))), // red
      select(greenRedPixel(bayerPattern),  avg(absd(rawGreenR(x, y2), rawGreenR(x, y)), absd(rawGreenR(x, y_2), rawGreenR(x, y))), // green on red row
      0))));

    dH(x, y) =
      select(greenBluePixel(bayerPattern), avg(absd(rawGreenB(x2, y), rawGreenB(x,   y)), absd(rawGreenB(x_2, y), rawGreenB(x, y))), // green on blue row
      select(bluePixel(bayerPattern),      avg(absd(rawGreenB(x1, y), rawGreenB(x_1, y)), absd(rawBlue(x2, y) + rawBlue(x_2, y), 2.0f*rawBlue(x, y))), // blue
      select(redPixel(bayerPattern),       avg(absd(rawGreenR(x1, y), rawGreenR(x_1, y)), absd(rawRed(x2, y)  + rawRed(x_2, y),  2.0f*rawRed(x, y))), // red
      select(greenRedPixel(bayerPattern),  avg(absd(rawGreenR(x2, y), rawGreenR(x,   y)), absd(rawGreenR(x_2, y), rawGreenR(x, y))), // green on red row
      0))));

    // Homogenity calulation over a diameter size region
    const int w = 4;
    const int diameter = 2 * w + 1;
    const int diameterSquared = diameter * diameter;
    RDom d(0, diameter, 0, diameter);
    Expr xp = x + d.x - w;
    Expr yp = y + d.y - w;

    // Count the number of pixels in the region that are horizontal.
    // This is given by determining when the vertical gradient is
    // greater than the horizontal gradient. Remember that a strong
    // vertical gradient means that the local area is horizontal since
    // the gradient is orthogonal to the direction of the local
    // feature.
    Func hCount("hCount");
    hCount(x, y) = sum(cast<int>(dH(xp, yp) <= dV(xp, yp)));

    // The local green estimate is a blend of the horizontal and vertical
    // green channel estimate based on how horizontal or vertical the
    // region is.
    Func g;
    g(x, y) = select(hCount(x, y) < diameterSquared / 2, gV(x, y), gH(x, y));

    // Red and blue minus the log of the green channel
    Func rmg, bmg;
    rmg(x, y) = rawRed(x, y) - g(x, y);
    bmg(x, y) = rawBlue(x, y) - g(x, y);

    // Use local box style filtering to estimate r-g and b-g
    // channels. Adding the g channel back in.
    Func r, b;
    r(x, y) =
      select(redPixel(bayerPattern),       (rmg(x, y) + rmg(x, y2) + rmg(x, y_2) + rmg(x2, y) + rmg(x_2, y)) / 5.0f,
      select(greenRedPixel(bayerPattern),  (rmg(x_1, y_2) + rmg(x_1, y) + rmg(x_1, y2) + rmg(x1, y_2) + rmg(x1, y) + rmg(x1, y2)) / 6.0f,
      select(greenBluePixel(bayerPattern), (rmg(x_2, y_1) + rmg(x, y_1) + rmg(x2, y_1) + rmg(x_2, y1) + rmg(x, y1) + rmg(x2, y1)) / 6.0f,
      select(bluePixel(bayerPattern),      (rmg(x1, y1) + rmg(x1, y_1) + rmg(x_1, y1) + rmg(x_1, y_1)) / 4.0f,
      0))))
      + g(x, y);

    b(x, y) =
      select(redPixel(bayerPattern),       (bmg(x1, y1) + bmg(x1, y_1) + bmg(x_1, y1) + bmg(x_1, y_1)) / 4.0f,
      select(greenRedPixel(bayerPattern),  (bmg(x_2, y_1) + bmg(x, y_1) + bmg(x2, y_1) + bmg(x_2, y1) + bmg(x, y1) + bmg(x2, y1)) / 6.0f,
      select(greenBluePixel(bayerPattern), (bmg(x_1, y_2) + bmg(x_1, y) + bmg(x_1, y2) + bmg(x1, y_2) + bmg(x1, y) + bmg(x1, y2)) / 6.0f,
      select(bluePixel(bayerPattern),      (bmg(x, y) + bmg(x, y2) + bmg(x, y_2) + bmg(x2, y) + bmg(x_2, y)) / 5.0f,
      0))))
      + g(x, y);

    demosaiced(x, y, c) =
      select(
          c == 0, r(x, y),
          c == 1, g(x, y),
          b(x, y));

    int kVec = target.natural_vector_size(Float(32));

    gV.compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    gH.compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    dV.compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    dH.compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    g.compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    r.compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    b.compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);
  }

  Func demosaic(
      Func deinterleaved,
      Func vignetteTableH,
      Func vignetteTableV,
      Expr blackLevelR,
      Expr blackLevelG,
      Expr blackLevelB,
      Expr whiteBalanceGainR,
      Expr whiteBalanceGainG,
      Expr whiteBalanceGainB,
      Expr clampMinR,
      Expr clampMinG,
      Expr clampMinB,
      Expr clampMaxR,
      Expr clampMaxG,
      Expr clampMaxB,
      Expr bayerPattern) {

    // These are the values we already know from the input
    // x_y = the value of channel x at a site in the input of channel y
    // gb refers to green sites in the blue rows
    // gr refers to green sites in the red rows

    // We want to express black level, white balance, and clamping as A(x - B)
    // Assuming x in [0, 1]
    // y = [(WB(x - BL)/(1 - BL)) - CLmin] / (CLmax - CLmin)
    //   = WB[x - (BL + CLmin(1 - BL) / WB)] / [(1 - BL)(CLmax - CLmin)]
    //   = A(x - B),
    //     A = WB / [(1 - BL)(CLmax - CLmin)], B = BL + CLmin(1 - BL) / WB

    Expr minRawR = blackLevelR;
    Expr minRawG = blackLevelG;
    Expr minRawB = blackLevelB;
    Expr maxRaw = float((1 << 16) - 1);

    Expr rangeRawR = maxRaw - minRawR;
    Expr rangeRawG = maxRaw - minRawG;
    Expr rangeRawB = maxRaw - minRawB;

    Expr biasR = minRawR + clampMinR * rangeRawR / whiteBalanceGainR;
    Expr biasG = minRawG + clampMinG * rangeRawG / whiteBalanceGainG;
    Expr biasB = minRawB + clampMinB * rangeRawB / whiteBalanceGainB;

    Expr invRangeR = whiteBalanceGainR / (rangeRawR * (clampMaxR - clampMinR));
    Expr invRangeG = whiteBalanceGainG / (rangeRawG * (clampMaxG - clampMinG));
    Expr invRangeB = whiteBalanceGainB / (rangeRawB * (clampMaxB - clampMinB));

    Expr redWB = (deinterleaved(x, y, 0) - biasR) * invRangeR;
    Expr greenWB = (deinterleaved(x, y, 1) - biasG) * invRangeG;
    Expr blueWB = (deinterleaved(x, y, 2) - biasB) * invRangeB;

    if (!Fast) {
      redWB *= vignetteTableH(0, x) * vignetteTableV(0, y);
      greenWB *= vignetteTableH(1, x) * vignetteTableV(1, y);
      blueWB *= vignetteTableH(2, x) * vignetteTableV(2, y);
    }


    // Raw bayer pixels
    Func rawRed("rawRed");
    Func rawBlue("rawBlue");
    Func rawGreenB("rawGreenB");
    Func rawGreenR("rawGreenR");

    rawRed(x, y)    = clamp(redWB, 0.0f, 1.0f);
    rawBlue(x, y)   = clamp(blueWB, 0.0f, 1.0f);
    rawGreenB(x, y) = clamp(greenWB, 0.0f, 1.0f);
    rawGreenR(x, y) = rawGreenB(x, y);

    // These are the ones we need to interpolate
    // Rename build up planar bayer channels
    Func redOnGreenBlue("redOnGreenBlue"); // Red on a greeb/blue pixel
    Func greenOnGreenBlue("greenOnGreenBlue"); // Green on a green/blue pixel
    Func blueOnGreenBlue("blueOnGreenBlue"); // Blue on a green/blue pixel

    Func redOnGreenRed("redOnGreenRed");  // Red on a green/red pixel
    Func greenOnGreenRed("greenOnGreenRed"); // Green on a green/red pixel
    Func blueOnGreenRed("blueOnGreenRed"); // Blue on a green/red pixel

    Func redOnBlue("redOnBlue");  // Red on a blue pixel
    Func greenOnBlue("greenOnBlue"); // Green on a blue pixel
    Func blueOnBlue("blueOnBlue");  // Blue on a blue pixel

    Func redOnRed("redOnRed"); // red on a red pixel
    Func greenOnRed("greenOnRed"); // Green on a red pixel
    Func blueOnRed("blueOnRed"); // Blue on a red pixel

    Func demosaiced("demosaiced");
    if (Fast) {
      bilinearDemosaic(rawRed, rawBlue, rawGreenB, rawGreenR,
          redOnGreenBlue, greenOnGreenBlue, blueOnGreenBlue,
          redOnGreenRed, greenOnGreenRed, blueOnGreenRed,
          redOnBlue, greenOnBlue, blueOnBlue,
          redOnRed, greenOnRed, blueOnRed,
          demosaiced,
          bayerPattern);
    } else {
      edgeAwareDemosaic(rawRed, rawBlue, rawGreenB, rawGreenR,
          redOnGreenBlue, greenOnGreenBlue, blueOnGreenBlue,
          redOnGreenRed, greenOnGreenRed, blueOnGreenRed,
          redOnBlue, greenOnBlue, blueOnBlue,
          redOnRed, greenOnRed, blueOnRed,
          demosaiced,
          bayerPattern);
    }

    int kVec = target.natural_vector_size(Float(32));

    demosaiced
      .compute_at(toneCorrected, x)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .reorder(y, x, c)
      .unroll(c);

    return demosaiced;
  }

  static const int kToneCurveLutSize = 4096;

  Func applyCCM(Func input, ImageParam ccm) {
    Expr ir = input(x, y, 0);
    Expr ig = input(x, y, 1);
    Expr ib = input(x, y, 2);

    Expr r = ccm(0, 0) * ir + ccm(1, 0) * ig + ccm(2, 0) * ib;
    Expr g = ccm(0, 1) * ir + ccm(1, 1) * ig + ccm(2, 1) * ib;
    Expr b = ccm(0, 2) * ir + ccm(1, 2) * ig + ccm(2, 2) * ib;

    Func corrected("corrected");
    corrected(x, y, c) =
      cast<uint16_t>(
          clamp(
              select(
                  c == 0, r,
                  c == 1, g,
                  b), 0, kToneCurveLutSize - 1));
    return corrected;
  }

  Func lp(
      Func input,
      Expr height,
      Expr alpha) {
    Expr alphaComp = 1.0f - alpha;

    Func blur("blur");
    // Classic exponetially weighted two-tap IIR filter
    blur(x, y, c) = undef<float>(); // Pure function init
    blur(x, 0, c) = cast<float>(input(x, 0, c)); // Seed the first value

    RDom ry(1, height - 1);
    Expr yp = height - 1 - ry;

    // Causal pass
    blur(x, ry, c) = blur(x, ry - 1, c) * alpha + cast<float>(input(x, ry, c)) * alphaComp;
    // Anti-causal pass
    blur(x, yp, c) = blur(x, yp + 1, c) * alpha + blur(x, yp, c) * alphaComp;

    Func transpose("transpose");
    transpose(x, y, c) = blur(y, x, c);

    const int kVec = target.natural_vector_size(Float(32));

    Var xo, xi, yo, yi, si;
    transpose
      .compute_root()
      .tile(x, y, xo, yo, x, y, 8, 8)
      .vectorize(x, kVec);

    blur.
      compute_at(transpose, yo);

    blur
      .update(1)
      .reorder(c, x, ry)
      .unroll(c)
      .vectorize(x);

    blur
      .update(2)
      .reorder(c, x, ry)
      .unroll(c)
      .vectorize(x);

    return transpose;
  }

  Func applyUnsharpMask(
      Func input,
      Expr width,
      Expr height,
      Expr sR,
      Expr sG,
      Expr sB,
      Expr alpha,
      Param<float> noiseCore,
      Param<bool> BGR,
      int outputBpp) {

    Func lowPass("lowPass");
    Func lowPass0("lowPass0");
    lowPass0 = lp(input, height, alpha);
    lowPass = lp(lowPass0, width, alpha);

    Func highPass("hp");
    highPass(x, y, c) =  cast<float>(input(x, y, c)) - lowPass(x, y, c);

    Func noiseGain("noiseGain");
    noiseGain(x, y, c) = 1.0f - exp(-(highPass(x, y, c) * highPass(x, y, c) * noiseCore));

    Expr cp = select(BGR == 0, c, 2 - c);
    const int range = (1 << outputBpp) - 1;

    Expr outputVal =
      clamp(
          lowPass(x, y, cp) + highPass(x, y, cp) * noiseGain(x, y, c) *
          select(cp == 0, sR, cp == 1, sG, sB),
          0, range);

    Func sharpened("sharpi");
    if (outputBpp == 8) {
      sharpened(x, y, c) = cast<uint8_t>(outputVal);
    } else {
      sharpened(x, y, c) = cast<uint16_t>(outputVal);
    }

    const int kStripSize = 32;
    const int kVec = target.natural_vector_size(Float(32));

    Var xo, xi;
    sharpened
      .compute_root()
      .split(y, yo, yi, kStripSize)
      .vectorize(yi, kVec);

    return sharpened;
  }

 public:
  Func generate(
      Target target,
      Func raw,
      Param<int> width,
      Param<int> height,
      Func vignetteTableH,
      Func vignetteTableV,
      Param<float> blackLevelR,
      Param<float> blackLevelG,
      Param<float> blackLevelB,
      Param<float> whiteBalanceGainR,
      Param<float> whiteBalanceGainG,
      Param<float> whiteBalanceGainB,
      Param<float> clampMinR,
      Param<float> clampMinG,
      Param<float> clampMinB,
      Param<float> clampMaxR,
      Param<float> clampMaxG,
      Param<float> clampMaxB,
      Param<float> sharpeningR,
      Param<float> sharpeningG,
      Param<float> sharpeningB,
      Param<float> sharpeningSupport,
      Param<float> noiseCore,
      ImageParam ccm,
      ImageParam toneTable,
      Param<bool> BGR,
      Param<int> bayerPattern,
      int outputBpp) {

    this->target = target;
    Expr sR = 1.0f + cast<float>(sharpeningR);
    Expr sG = 1.0f + cast<float>(sharpeningG);
    Expr sB = 1.0f + cast<float>(sharpeningB);

    // This is the ISP pipeline
    Func vignettingGain("vignettingGain");
    Func deinterleaved("deinterleaved");
    Func demosaiced("demosaiced");
    Func colorCorrected("ccm");
    Func sharpened("sharpened");

    deinterleaved   = deinterleave(raw, bayerPattern);
    demosaiced      = demosaic(
      deinterleaved,
      vignetteTableH, vignetteTableV,
      blackLevelR, blackLevelG, blackLevelB,
      whiteBalanceGainR, whiteBalanceGainG, whiteBalanceGainB,
      clampMinR, clampMinG, clampMinB, clampMaxR, clampMaxG, clampMaxB, bayerPattern);
    colorCorrected  = applyCCM(demosaiced, ccm);
    toneCorrected(x, y, c) = toneTable(c, colorCorrected(x, y, c));

    if (Fast) {
      Expr cp = select(BGR == 0, c, 2 - c);
      ispOutput(x, y, c) = toneCorrected(x , y, cp);
    } else {
      Expr alpha = pow(sharpeningSupport, 1.0f/4.0f);
      ispOutput = applyUnsharpMask(toneCorrected, width, height, sR, sG, sB, alpha, noiseCore, BGR, outputBpp);
    }
    // Schedule it using local vars
    Var yii("yii"), xi("xi");

    const int kStripSize = 32;
    const int kVec = target.natural_vector_size(Int(16));

    deinterleaved
      .compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, 2 * kVec, TailStrategy::RoundUp)
      .reorder(c, x, y)
      .unroll(c);

    toneCorrected
      .compute_root()
      .split(y, yo, yi, kStripSize)
      .split(yi, yi, yii, 2)
      .split(x, x, xi, 2*kVec, TailStrategy::RoundUp)
      .reorder(xi, c, yii, x, yi, yo)
      .vectorize(xi, 2 * kVec);

    ispOutput
      .output_buffer()
      .set_stride(0, 3)
      .set_stride(2, 1)
      .set_bounds(2, 0, 3);

    return ispOutput;
  }

  CameraIspGen() {};
};


int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ImageParam input(UInt(16), 2);
  Param<int> width;
  Param<int> height;
  ImageParam vignetteTableH(Float(32), 2, "vignetteH");
  ImageParam vignetteTableV(Float(32), 2, "vignetteV");
  Param<float> blackLevelR("blackLevelR");
  Param<float> blackLevelG("blackLevelG");
  Param<float> blackLevelB("blackLevelB");
  Param<float> whiteBalanceGainR("whiteBalanceGainR");
  Param<float> whiteBalanceGainG("whiteBalanceGainG");
  Param<float> whiteBalanceGainB("whiteBalanceGainB");
  Param<float> clampMinR("clampMinR");
  Param<float> clampMinG("clampMinG");
  Param<float> clampMinB("clampMinB");
  Param<float> clampMaxR("clampMaxR");
  Param<float> clampMaxG("clampMaxG");
  Param<float> clampMaxB("clampMaxB");
  Param<float> sharpeningR("sharpeningR");
  Param<float> sharpeningG("sharpeningG");
  Param<float> sharpeningB("sharpeningB");
  Param<float> sharpeningSupport("sharpeningSupport");
  Param<float> noiseCore("noiseCore");
  ImageParam ccm(Float(32), 2, "ccm");
  ImageParam toneTable(UInt(FLAGS_output_bpp), 2, "toneTable");
  Param<bool> BGR;
  Param<int> bayerPattern("bayerPattern");

  // Pick a target architecture
  Target target = get_target_from_environment();

  // Build variants of the pipeline
  CameraIspGen<false> cameraIspGen;
  CameraIspGen<true> cameraIspGenFast;

  Func rawM("rawm");
  rawM = BoundaryConditions::mirror_interior(input);

  Func vignetteTableHM("vhm");
  vignetteTableHM = BoundaryConditions::mirror_image(vignetteTableH);

  Func vignetteTableVM("vvm");
  vignetteTableVM = BoundaryConditions::mirror_image(vignetteTableV);

  Func cameraIsp = cameraIspGen.generate(
      target, rawM, width, height, vignetteTableHM, vignetteTableVM,
      blackLevelR, blackLevelG, blackLevelB,
      whiteBalanceGainR, whiteBalanceGainG, whiteBalanceGainB,
      clampMinR, clampMinG, clampMinB, clampMaxR, clampMaxG, clampMaxB,
      sharpeningR, sharpeningG, sharpeningB, sharpeningSupport, noiseCore,
      ccm, toneTable, BGR, bayerPattern, FLAGS_output_bpp);

  Func cameraIspFast =
    cameraIspGenFast.generate(
        target, rawM, width, height, vignetteTableHM, vignetteTableVM,
        blackLevelR, blackLevelG, blackLevelB,
        whiteBalanceGainR, whiteBalanceGainG, whiteBalanceGainB,
        clampMinR, clampMinG, clampMinB, clampMaxR, clampMaxG, clampMaxB,
        sharpeningR, sharpeningG, sharpeningB, sharpeningSupport, noiseCore,
        ccm, toneTable, BGR, bayerPattern, FLAGS_output_bpp);

  std::vector<Argument> args = {
    input, width, height, vignetteTableH, vignetteTableV,
    blackLevelR, blackLevelG, blackLevelB,
    whiteBalanceGainR, whiteBalanceGainG, whiteBalanceGainB,
    clampMinR, clampMinG, clampMinB, clampMaxR, clampMaxG, clampMaxB,
    sharpeningR, sharpeningG, sharpeningB, sharpeningSupport, noiseCore, ccm, toneTable, BGR, bayerPattern};

  // Compile the pipelines
  // Use to cameraIsp.print_loop_nest() here to debug loop unrolling
  std::cout << "Halide: " << "Generating " << FLAGS_output_bpp << " bit isp" << std::endl;
  string cigName = "CameraIspGen" + to_string(FLAGS_output_bpp);
  cameraIsp.compile_to_static_library(cigName, args, cigName, target);
  cameraIsp.compile_to_assembly(cigName + ".s", args, target);

  std::cout << "Halide: " << "Generating " << FLAGS_output_bpp << " bit fastest isp" << std::endl;
  cigName = "CameraIspGenFast" + to_string(FLAGS_output_bpp);
  cameraIspFast.compile_to_static_library(cigName, args,  cigName, target);
  cameraIspFast.compile_to_assembly(cigName + ".s", args, target);

  return EXIT_SUCCESS;
}
