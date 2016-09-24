#include "ImageParam.h"
#include "Halide.h"
#include "Pipeline.h"
#include "Var.h"

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

  Func deinterleave(Func raw) {
    // Deinterleave the color channels
    Func deinterleaved("deinterleaved");

    // Fixed pattern of:
    // G B
    // R G
    deinterleaved(x, y, c) =
      cast<float>(
          select(c == 1 && (x % 2) == 0 && (y % 2) == 0, raw(x, y), // green on blue row
          select(c == 2 && (x % 2) == 1 && (y % 2) == 0, raw(x, y), // blue
          select(c == 0 && (x % 2) == 0 && (y % 2) == 1, raw(x, y), // red
          select(c == 1 && (x % 2) == 1 && (y % 2) == 1, raw(x, y), // green on red row
          0)))));

    return deinterleaved;
  }

  void bilinearDemosaic(
      Func r_r,
      Func b_b,
      Func g_gb,
      Func g_gr,
      Func& r_gb,
      Func& b_gb,
      Func& b_gr,
      Func& r_gr,
      Func& b_r,
      Func& r_b,
      Func& g_r,
      Func& g_b ) {
    // Neighbor indices
    Expr x1 = x + 1;
    Expr x_1 = x - 1;

    Expr y1 = y + 1;
    Expr y_1 = y - 1;

    // Horizontally or vertically average the red and blue channels at
    // green-red and green blue pixels.
    r_gb(x, y) = avg(r_r(x, y_1), r_r(x,y1));
    b_gb(x, y) = avg(b_b(x_1, y), b_b(x1,y));

    b_gr(x, y) = avg(b_b(x, y_1), b_b(x,y1));
    r_gr(x, y) = avg(r_r(x_1, y), r_r(x1,y));

    // Box average pixels surrounding the red or blue channels when
    // they are alternatively on blue and red pixels respectively.
    r_b(x, y) = avg(avg(r_r(x_1, y_1), r_r(x1, y_1)), avg(r_r(x_1, y1), r_r(x1, y1)));
    b_r(x, y) = avg(avg(b_b(x_1, y_1), b_b(x1, y_1)), avg(b_b(x_1, y1), b_b(x1, y1)));

    // Average the green colors that are above, below, left and right
    // at red and blue pixels.
    g_r(x, y) = avg(avg(g_gr(x_1, y), g_gr(x1, y)), avg(g_gb(x, y_1), g_gb(x, y1)));
    g_b(x, y) = avg(avg(g_gb(x_1, y), g_gb(x1, y)), avg(g_gr(x, y_1), g_gr(x, y1)));
  }

  void edgeAwareDemosaic(
      Func r_r,
      Func b_b,
      Func g_gb,
      Func g_gr,
      Func& r_gb,
      Func& b_gb,
      Func& b_gr,
      Func& r_gr,
      Func& b_r,
      Func& r_b,
      Func& g_r,
      Func& g_b ) {
    // Neighbor indices
    Expr x1 = x + 1;
    Expr x2 = x + 2;
    Expr x_1 = x - 1;
    Expr x_2 = x - 2;

    Expr y1 = y + 1;
    Expr y2 = y + 2;
    Expr y_1 = y - 1;
    Expr y_2 = y - 2;

    Expr gv_r  = avg(g_gb(x, y_1), g_gb(x, y1));
    Expr gvd_r = absd(g_gb(x, y_1), g_gb(x, y1));
    Expr gh_r  = avg(g_gr(x1, y), g_gr(x_1, y));
    Expr ghd_r = absd(g_gr(x1, y), g_gr(x_1, y));

    g_r(x, y)  = select(ghd_r < gvd_r, gh_r, gv_r);

    Expr gv_b  = avg(g_gr(x, y1), g_gr(x, y_1));
    Expr gvd_b = absd(g_gr(x, y1), g_gr(x, y_1));
    Expr gh_b  = avg(g_gb(x_1, y), g_gb(x1, y));
    Expr ghd_b = absd(g_gb(x_1, y), g_gb(x1, y));

    g_b(x, y)  = select(ghd_b < gvd_b, gh_b, gv_b);

    // Next interpolate red at gr by first interpolating, then
    // correcting using the error green would have had if we had
    // interpolated it in the same way (i.e. add the second derivative
    // of the green channel at the same place).
    Expr correction;
    correction = g_gr(x, y) - avg(g_r(x1, y), g_r(x_1, y));
    r_gr(x, y) = correction + avg(r_r(x_1, y), r_r(x1, y));

    // Do the same for other reds and blues at green sites
    correction = g_gr(x, y) - avg(g_b(x, y1), g_b(x, y_1));
    b_gr(x, y) = correction + avg(b_b(x, y1), b_b(x, y_1));

    correction = g_gb(x, y) - avg(g_r(x, y_1), g_r(x, y1));
    r_gb(x, y) = correction + avg(r_r(x, y_1), r_r(x, y1));

    correction = g_gb(x, y) - avg(g_b(x1, y), g_b(x1, y));
    b_gb(x, y) = correction + avg(b_b(x1, y), b_b(x1, y));

    // Interpolate diagonally to get red at blue and blue at
    // red.
    correction = g_b(x, y)  - avg(g_r(x1, y_1), g_r(x_1, y1));
    Expr rp_b  = correction + avg(r_r(x1, y_1), r_r(x_1, y1));
    Expr rpd_b = absd(r_r(x1, y_1), r_r(x_1, y1));

    correction = g_b(x, y)  - avg(g_r(x_1, y_1), g_r(x1, y1));
    Expr rn_b  = correction + avg(r_r(x_1, y_1), r_r(x1, y1));
    Expr rnd_b = absd(r_r(x_1, y_1), r_r(x1, y1));

    r_b(x, y)  = select(rpd_b < rnd_b, rp_b, rn_b);

    // Same thing for blue at red
    correction = g_r(x, y)  - avg(g_b(x_1, y1), g_b(x1, y_1));
    Expr bp_r  = correction + avg(b_b(x_1, y1), b_b(x1, y_1));
    Expr bpd_r = absd(b_b(x_1, y1), b_b(x1, y_1));

    correction = g_r(x, y)  - avg(g_b(x1, y1), g_b(x_1, y_1));
    Expr bn_r  = correction + avg(b_b(x1, y1), b_b(x_1, y_1));
    Expr bnd_r = absd(b_b(x1, y1), b_b(x_1, y_1));

    b_r(x, y)  =  select(bpd_r < bnd_r, bp_r, bn_r);
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
      Expr whiteBalanceGainB) {
    // These are the values we already know from the input
    // x_y = the value of channel x at a site in the input of channel y
    // gb refers to green sites in the blue rows
    // gr refers to green sites in the red rows

    Expr minRawR = blackLevelR;
    Expr minRawG = blackLevelG;
    Expr minRawB = blackLevelB;
    Expr maxRaw = float((1 << 16) - 1);

    Expr invRangeR = whiteBalanceGainR / (maxRaw - minRawR);
    Expr invRangeG = whiteBalanceGainG / (maxRaw - minRawG);
    Expr invRangeB = whiteBalanceGainB / (maxRaw - minRawB);

    // Bayer pixels
    Func r_r("r_r");
    Func b_b("b_b");
    Func g_gb("g_gb");
    Func g_gr("g_gr");

    if (Fast) {
      r_r(x, y)  = (deinterleaved(x, y, 0) - minRawR) * invRangeR;
      b_b(x, y)  = (deinterleaved(x, y, 2) - minRawB) * invRangeB;
      g_gb(x, y) = (deinterleaved(x, y, 1) - minRawG) * invRangeG;
      g_gr(x, y) = (deinterleaved(x, y, 1) - minRawG) * invRangeG;
    } else {
      r_r(x, y)  = (deinterleaved(x, y, 0) - minRawR) * invRangeR * vignetteTableH(0, x) * vignetteTableV(0, y);
      b_b(x, y)  = (deinterleaved(x, y, 2) - minRawB) * invRangeB * vignetteTableH(2, x) * vignetteTableV(2, y);
      g_gb(x, y) = (deinterleaved(x, y, 1) - minRawG) * invRangeG * vignetteTableH(1, x) * vignetteTableV(1, y);
      g_gr(x, y) = (deinterleaved(x, y, 1) - minRawG) * invRangeG * vignetteTableH(1, x) * vignetteTableV(1, y);
    }

    // These are the ones we need to interpolate
    // Rename build up planar bayer channels
    Func r_gb("r_gb"); // Red on a greeb/blue pixel
    Func b_gb("b_gb"); // Blue on a green/blue pixel

    Func b_gr("b_gr"); // Blue on a green/red pixel
    Func r_gr("r_gr");  // Red on a green/red pixel

    Func b_r("b_r"); // Blue on a red pixel
    Func r_b("r_b");  // Red on a blue pixel

    Func g_r("g_r"); // Green on a red pixel
    Func g_b("g_b"); // Green on a blue pixel

    if (Fast) {
      bilinearDemosaic(r_r, b_b, g_gb, g_gr, r_gb, b_gb, b_gr, r_gr, b_r, r_b, g_r, g_b);
    } else {
      edgeAwareDemosaic(r_r, b_b, g_gb, g_gr, r_gb, b_gb, b_gr, r_gr, b_r, r_b, g_r, g_b);
    }

    // Interleave the resulting channels
    Func r = interleave_y(interleave_x(r_gb, r_b), interleave_x(r_r, r_gr));
    Func g = interleave_y(interleave_x(g_gb, g_b), interleave_x(g_r, g_gr));
    Func b = interleave_y(interleave_x(b_gb, b_b), interleave_x(b_r, b_gr));

    Func demosaiced("demosaiced");
    demosaiced(x, y, c) =
      select(
          c == 0, r(x, y),
          c == 1, g(x, y),
          b(x, y));

    int kVec = target.natural_vector_size(Float(32));

    g_r
      .compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    g_b
      .compute_at(toneCorrected, yi)
      .store_at(toneCorrected, yo)
      .vectorize(x, kVec, TailStrategy::RoundUp)
      .fold_storage(y, 64);

    demosaiced
      .compute_at(toneCorrected, x)
      .vectorize(x, kVec)
      .reorder(c, x, y)
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
      Expr height) {
    const float alpha = powf(0.25f, 1.0f/4.0f);
    const float alphaComp = 1.0f - alpha;

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
      .vectorize(x, kVec)
      .parallel(yo)
      .parallel(c);

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
      Param<bool> BGR,
      int outputBpp) {

    Func lowPass("lowPass");
    Func lowPass0("lowPass0");
    lowPass0 = lp(input, height);
    lowPass = lp(lowPass0, width);

    Func highPass("hp");
    highPass(x, y, c) =  cast<float>(input(x, y, c)) - lowPass(x, y, c);

    Expr cp = select(BGR == 0, c, 2 - c);
    const int range = (1 << outputBpp) - 1;

    Expr outputVal =
      clamp(
          lowPass(x, y, cp) + highPass(x, y, cp) *
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
      .vectorize(yi, kVec)
      .parallel(yo)
      .parallel(c);

    return sharpened;
  }

 public:
  Func generate(
      Target target,
      Func raw,
      Param<int> width,
      Param<int> height,
      Param<float> denoise,
      Param<int> denoiseRadius,
      Func vignetteTableH,
      Func vignetteTableV,
      Param<float> blackLevelR,
      Param<float> blackLevelG,
      Param<float> blackLevelB,
      Param<float> whiteBalanceGainR,
      Param<float> whiteBalanceGainG,
      Param<float> whiteBalanceGainB,
      Param<float> sharpenningR,
      Param<float> sharpenningG,
      Param<float> sharpenningB,
      ImageParam ccm,
      ImageParam toneTable,
      Param<bool> BGR,
      int outputBpp) {

    this->target = target;
    Expr sR = 1.0f + cast<float>(sharpenningR);
    Expr sG = 1.0f + cast<float>(sharpenningG);
    Expr sB = 1.0f + cast<float>(sharpenningB);

    // This is the ISP pipeline
    Func vignettingGain("vignettingGain");
    Func deinterleaved("deinterleaved");
    Func demosaiced("demosaiced");
    Func colorCorrected("ccm");
    Func sharpened("sharpened");

    deinterleaved = deinterleave(raw);
    demosaiced     = demosaic(deinterleaved, vignetteTableH, vignetteTableV, blackLevelR, blackLevelG, blackLevelB, whiteBalanceGainR, whiteBalanceGainG, whiteBalanceGainB);
    colorCorrected = applyCCM(demosaiced, ccm);
    toneCorrected(x, y, c) = toneTable(c, colorCorrected(x, y, c));

    if (Fast) {
      Expr cp = select(BGR == 0, c, 2 - c);
      ispOutput(x, y, c) = toneCorrected(x , y, cp);
    } else {
      ispOutput = applyUnsharpMask(toneCorrected, width, height, sR, sG, sB, BGR, outputBpp);
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
      .vectorize(xi, 2 * kVec)
      .parallel(yo);

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
  Param<float> denoise;
  Param<int> denoiseRadius;
  ImageParam vignetteTableH(Float(32), 2, "vignetteH");
  ImageParam vignetteTableV(Float(32), 2, "vignetteV");
  Param<float> blackLevelR("blackLevelR");
  Param<float> blackLevelG("blackLevelG");
  Param<float> blackLevelB("blackLevelB");
  Param<float> whiteBalanceGainR("whiteBalanceGainR");
  Param<float> whiteBalanceGainG("whiteBalanceGainG");
  Param<float> whiteBalanceGainB("whiteBalanceGainB");
  Param<float> sharpenningR("sharpenningR");
  Param<float> sharpenningG("sharpenningG");
  Param<float> sharpenningB("sharpenninngB");
  ImageParam ccm(Float(32), 2, "ccm");
  ImageParam toneTable(UInt(FLAGS_output_bpp), 2, "toneTable");
  Param<bool> BGR;

  // Pick a target architecture
  Target target = get_target_from_environment();

  // Build variants of the pipeline
  CameraIspGen<false> cameraIspGen;
  CameraIspGen<true> cameraIspGenFast;

  Func rawM("rawm");
  rawM = BoundaryConditions::mirror_image(input);

  Func vignetteTableHM("vhm");
  vignetteTableHM = BoundaryConditions::mirror_image(vignetteTableH);

  Func vignetteTableVM("vvm");
  vignetteTableVM = BoundaryConditions::mirror_image(vignetteTableV);

  Func cameraIsp = cameraIspGen.generate(
      target, rawM, width, height, denoise, denoiseRadius, vignetteTableHM, vignetteTableVM, blackLevelR, blackLevelG, blackLevelB, whiteBalanceGainR,
      whiteBalanceGainG, whiteBalanceGainB, sharpenningR, sharpenningG, sharpenningB, ccm, toneTable, BGR, FLAGS_output_bpp);

  Func cameraIspFast =
    cameraIspGenFast.generate(
        target, rawM, width, height, denoise, denoiseRadius, vignetteTableHM, vignetteTableVM, blackLevelR, blackLevelG, blackLevelB, whiteBalanceGainR,
        whiteBalanceGainG, whiteBalanceGainB, sharpenningR, sharpenningG, sharpenningB, ccm, toneTable, BGR, FLAGS_output_bpp);

  std::vector<Argument> args = {
    input, width, height, denoise, denoiseRadius, vignetteTableH, vignetteTableV, blackLevelR, blackLevelG, blackLevelB, whiteBalanceGainR, whiteBalanceGainG,
    whiteBalanceGainB, sharpenningR, sharpenningG, sharpenningB,  ccm, toneTable,  BGR};

  // Compile the pipelines
  // Use to cameraIsp.print_loop_nest() here to debug loop unrolling
  std::cout << "Halide: " << "Generating " << FLAGS_output_bpp << " bit isp" << std::endl;
  cameraIsp.compile_to_static_library("CameraIspGen" + to_string(FLAGS_output_bpp), args, target);
  cameraIsp.compile_to_assembly("CameraIspGen" + to_string(FLAGS_output_bpp) + ".s", args, target);

  std::cout << "Halide: " << "Generating " << FLAGS_output_bpp << " bit fastest isp" << std::endl;
  cameraIspFast.compile_to_static_library("CameraIspGenFast" + to_string(FLAGS_output_bpp), args, target);
  cameraIspFast.compile_to_assembly("CameraIspGenFast" + to_string(FLAGS_output_bpp) + ".s", args, target);
  return EXIT_SUCCESS;
}
