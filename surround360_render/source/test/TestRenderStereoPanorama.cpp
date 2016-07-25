/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "CameraMetadata.h"
#include "CvUtil.h"
#include "Filter.h"
#include "ImageWarper.h"
#include "IntrinsicCalibration.h"
#include "MathUtil.h"
#include "MonotonicTable.h"
#include "NovelView.h"
#include "OpticalFlowFactory.h"
#include "OpticalFlowVisualization.h"
#include "PoleRemoval.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace cv;
using namespace std;
using namespace surround360;
using namespace surround360::calibration;
using namespace surround360::math_util;
using namespace surround360::optical_flow;
using namespace surround360::util;
using namespace surround360::warper;

DEFINE_string(src_intrinsic_param_file,   "",             "path to read intrinsic matrices");
DEFINE_string(rig_json_file,              "",             "path to json file drescribing camera array");
DEFINE_string(ring_rectify_file,          "NONE",         "path to rectification transforms file for ring of cameras");
DEFINE_string(imgs_dir,                   "",             "path to folder of images with names matching cameras in the rig file");
DEFINE_string(output_data_dir,            "",             "path to write spherical projections for debugging");
DEFINE_string(prev_frame_data_dir,        "NONE",         "path to data for previous frame; used for temporal regularization");
DEFINE_string(output_cubemap_path,        "",             "path to write output oculus 360 cubemap");
DEFINE_string(output_equirect_path,       "",             "path to write output oculus 360 cubemap");
DEFINE_double(interpupilary_dist,         6.4,            "separation of eyes for stereo, spherical_in whatever units the rig json uses.");
DEFINE_double(camera_ring_radius,         19.0,           "used for computing shift for zero parallax as a function of IPD.");
DEFINE_int32(side_alpha_feather_size,     100,            "alpha feather for projection of side cameras to spherical coordinates");
DEFINE_int32(std_alpha_feather_size,      31,             "alpha feather for all other purposes. must be odd");
DEFINE_bool(save_debug_images,            false,          "if true, lots of debug images are generated");
DEFINE_double(sharpenning,                0.0f,           "0.0 to 1.0 amount of sharpenning");
DEFINE_bool(enable_top,                   false,          "is there a top camera?");
DEFINE_bool(enable_bottom,                false,          "are there two bottom cameras?");
DEFINE_bool(enable_pole_removal,          false,          "if true, pole removal masks are used; if false, primary bottom camera is used");
DEFINE_string(bottom_pole_masks_dir,      "",             "path to bottom camera pole masks dir");
DEFINE_string(side_flow_alg,              "pixflow_low",  "which optical flow algorithm to use for sides");
DEFINE_string(polar_flow_alg,             "pixflow_low",  "which optical flow algorithm to use for top/bottom warp with sides");
DEFINE_string(poleremoval_flow_alg,       "pixflow_low",  "which optical flow algorithm to use for pole removal with secondary bottom camera");
DEFINE_double(zero_parallax_dist,         10000.0,        "distance where parallax is zero");
DEFINE_int32(eqr_width,                   256,            "height of spherical projection image (0 to 2pi)");
DEFINE_int32(eqr_height,                  128,            "height of spherical projection image (0 to pi)");
DEFINE_int32(final_eqr_width,             3480,           "resize before stacking stereo equirect width");
DEFINE_int32(final_eqr_height,            960,            "resize before stacking stereo equirect height");
DEFINE_int32(cubemap_face_resolution,     1536,           "resolution of output cubemaps");
DEFINE_string(cubemap_format,             "video",        "either video or photo");
DEFINE_bool(fast_preview_mode,            false,          "if true, lots of tweaks for +speed -quality");

// project the image of a single camera into spherical coordinates
void projectCamImageToSphericalThread(
    Mat* intrinsic,
    Mat* distCoeffs,
    CameraMetadata* cam,
    Mat* perspectiveTransform,
    Mat* camImage,
    Mat* outProjectedImage) {

  Mat projectedImage;
  if (cam->isFisheye) {
    VLOG(1) << "Projecting fisheye camera";
    projectedImage = sideFisheyeToSpherical(
      *camImage,
      *cam,
      FLAGS_eqr_width * (cam->fisheyeFovDegrees / 360.0),
      FLAGS_eqr_height * (cam->fisheyeFovDegrees / 180.0));
  } else {
    VLOG(1) << "Projecting non-fisheye camera";
    const bool skipUndistort =
      FLAGS_fast_preview_mode || FLAGS_src_intrinsic_param_file == "NONE";
    projectedImage = undistortToSpherical(
      cam->fovHorizontal,
      cam->fovHorizontal / cam->aspectRatioWH,
      FLAGS_eqr_width * (cam->fovHorizontal / 360.0),
      FLAGS_eqr_height * ((cam->fovHorizontal / cam->aspectRatioWH) / 180.0),
      *intrinsic,
      *distCoeffs,
      *perspectiveTransform,
      *camImage,
      FLAGS_side_alpha_feather_size,
      skipUndistort);

    // If the un-padded height is odd and FLAGS_eqr_height is even, we can't do equal
    // padding to get the final image to be FLAGS_eqr_height. the formulas below give
    // equal padding if possible, or equal +/-1 if not.
    const int paddingAbove = (FLAGS_eqr_height - projectedImage.rows) / 2;
    const int paddingBelow = FLAGS_eqr_height - projectedImage.rows - paddingAbove;
    copyMakeBorder(
      projectedImage,
      projectedImage,
      paddingAbove,
      paddingBelow,
      0,
      0,
      BORDER_CONSTANT,
      Scalar(0.0, 0.0, 0.0));
  }
  *outProjectedImage = projectedImage;
}

// project all of the (side) cameras' images into spherical coordinates
void projectSphericalCamImages(
      const vector<CameraMetadata>& camModelArray,
      vector<Mat>& sideCamTransforms,
      const string& imagesDir,
      vector<Mat>& projectionImages) {

  VLOG(1) << "Projecting side camera images to spherical coordinates";

  const double startLoadCameraImagesTime = getCurrTimeSec();
  vector< pair<CameraMetadata, Mat> > camImagePairs;
  loadCameraImagePairs(camModelArray, imagesDir, camImagePairs);
  const double endLoadCameraImagesTime = getCurrTimeSec();
  VLOG(1) << "Time to load images from file: "
    << endLoadCameraImagesTime - startLoadCameraImagesTime
    << " sec";

  Mat intrinsic, distCoeffs;
  if (FLAGS_src_intrinsic_param_file == "NONE") {
    VLOG(1) << "src_intrinsic_param_file = NONE. no intrinsics loaded";
  } else {
    FileStorage fileStorage(FLAGS_src_intrinsic_param_file, FileStorage::READ);
    if (fileStorage.isOpened()) {
      fileStorage["intrinsic"] >> intrinsic;
      fileStorage["distCoeffs"] >> distCoeffs;
    } else {
      throw VrCamException("file read failed: " + FLAGS_src_intrinsic_param_file);
    }
  }

  // to ensure thread safety, we need to make absolutely sure there is no monkey business
  // with STL pair + cv::Mat going out of scope, so before spawning threads we unpack the
  // pairs.
  vector<CameraMetadata> camModels;
  vector<Mat> camImages;
  for (int camIdx = 0; camIdx < camImagePairs.size(); ++camIdx) {
    const auto& camImagePair = camImagePairs[camIdx];
    camModels.push_back(camImagePair.first);
    camImages.push_back(camImagePair.second);
  }

  projectionImages = vector<Mat>(camImagePairs.size(), Mat());
  vector<std::thread> threads;
  for (int camIdx = 0; camIdx < camImagePairs.size(); ++camIdx) {
    threads.push_back(std::thread(
      projectCamImageToSphericalThread,
      &intrinsic,
      &distCoeffs,
      &camModels[camIdx],
      &sideCamTransforms[camIdx],
      &camImages[camIdx],
      &projectionImages[camIdx]
    ));
  }
  for (std::thread& t : threads) { t.join(); }

  if (FLAGS_save_debug_images) {
    for (int camIdx = 0; camIdx < camImagePairs.size(); ++camIdx) {
      const string cropImageFilename = FLAGS_output_data_dir +
        "/projections/crop_" + camImagePairs[camIdx].first.cameraId + ".png";
      imwriteExceptionOnFail(cropImageFilename, projectionImages[camIdx]);
    }
  }
}

// this is where the main work of optical flow for adjacent side cameras is done
void prepareNovelViewGeneratorThread(
    const int overlapImageWidth,
    const int leftIdx, // only used to determine debug image filename
    Mat* imageL,
    Mat* imageR,
    NovelViewGenerator* novelViewGen) {

  Mat overlapImageL = (*imageL)(Rect(
    imageL->cols - overlapImageWidth, 0, overlapImageWidth, imageL->rows));
  Mat overlapImageR = (*imageR)(Rect(0, 0, overlapImageWidth, imageR->rows));

  // save the images that are going into flow. we will need them in the next frame
  imwriteExceptionOnFail(
    FLAGS_output_data_dir + "/flow_images/overlap_" + std::to_string(leftIdx) + "_L.png",
    overlapImageL);
  imwriteExceptionOnFail(
    FLAGS_output_data_dir + "/flow_images/overlap_" + std::to_string(leftIdx) + "_R.png",
    overlapImageR);

  // read the previous frame's flow results, if available
  Mat prevFrameFlowLtoR, prevFrameFlowRtoL, prevOverlapImageL, prevOverlapImageR;
  if (FLAGS_prev_frame_data_dir != "NONE" && !FLAGS_fast_preview_mode) {
    VLOG(1) << "Reading previous frame flow and images from: "
      << FLAGS_prev_frame_data_dir;
    prevFrameFlowLtoR = readFlowFromFile(
      FLAGS_prev_frame_data_dir + "/flow/flowLtoR_" + std::to_string(leftIdx) + ".bin");
    prevFrameFlowRtoL = readFlowFromFile(
      FLAGS_prev_frame_data_dir + "/flow/flowRtoL_" + std::to_string(leftIdx) + ".bin");
    prevOverlapImageL = imreadExceptionOnFail(
      FLAGS_prev_frame_data_dir + "/flow_images/overlap_" + std::to_string(leftIdx) + "_L.png",
      -1);
    prevOverlapImageR = imreadExceptionOnFail(
      FLAGS_prev_frame_data_dir + "/flow_images/overlap_" + std::to_string(leftIdx) + "_R.png",
      -1);
    VLOG(1) << "Loaded previous frame's flow OK";
  }

  // this is the call to actually compute optical flow
  novelViewGen->prepare(
    overlapImageL,
    overlapImageR,
    prevFrameFlowLtoR,
    prevFrameFlowRtoL,
    prevOverlapImageL,
    prevOverlapImageR);

  // get the results of flow and save them. we will need these for temporal regularization
  const Mat flowLtoR = novelViewGen->getFlowLtoR();
  const Mat flowRtoL = novelViewGen->getFlowRtoL();
  saveFlowToFile(
    flowLtoR,
    FLAGS_output_data_dir + "/flow/flowLtoR_" + std::to_string(leftIdx) + ".bin");
  saveFlowToFile(
    flowRtoL,
    FLAGS_output_data_dir + "/flow/flowRtoL_" + std::to_string(leftIdx) + ".bin");
}

// a "chunk" is the portion from a pair of overlapping cameras. returns left/right images
void renderStereoPanoramaChunksThread(
    const int leftIdx, // left camera
    const int numCams,
    const int camImageWidth,
    const int camImageHeight,
    const int numNovelViews,
    const float fovHorizontalRadians,
    const float vergeAtInfinitySlabDisplacement,
    NovelViewGenerator* novelViewGen,
    Mat* chunkL,
    Mat* chunkR) {

  int currChunkX = 0; // current column in chunk to write
  LazyNovelViewBuffer lazyNovelViewBuffer(FLAGS_eqr_width / numCams, camImageHeight);
  for (int nvIdx = 0; nvIdx < numNovelViews; ++nvIdx) {
    const float shift = float(nvIdx) / float(numNovelViews);
    const float slabShift =
      float(camImageWidth) * 0.5f - float(numNovelViews - nvIdx);

    for (int v = 0; v < camImageHeight; ++v) {
      lazyNovelViewBuffer.warpL[currChunkX][v] =
        Point3f(slabShift + vergeAtInfinitySlabDisplacement, v, shift);
      lazyNovelViewBuffer.warpR[currChunkX][v] =
        Point3f(slabShift - vergeAtInfinitySlabDisplacement, v, shift);
    }
    ++currChunkX;
  }

  pair<Mat, Mat> lazyNovelChunksLR =
    novelViewGen->combineLazyNovelViews(lazyNovelViewBuffer);
  *chunkL = lazyNovelChunksLR.first;
  *chunkR = lazyNovelChunksLR.second;
}

// generates a left/right eye equirect panorama using slices of novel views
void generateRingOfNovelViewsAndRenderStereoSpherical(
    const float camFovHorizontalDegrees,
    vector<Mat>& projectionImages,
    Mat& panoImageL,
    Mat& panoImageR,
    double& opticalFlowRuntime,
    double& novelViewRuntime) {

  const int numCams = projectionImages.size();

  // this is the amount of horizontal overlap the cameras would have if they
  // were all perfectly aligned (in fact due to misalignment they overlap by a
  // different amount for each pair, but we ignore that to make it simple)
  const float fovHorizontalRadians = toRadians(camFovHorizontalDegrees);
  const float overlapAngleDegrees =
    (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
  const int camImageWidth = projectionImages[0].cols;
  const int camImageHeight = projectionImages[0].rows;
  const int overlapImageWidth =
    float(camImageWidth) * (overlapAngleDegrees / camFovHorizontalDegrees);
  const int numNovelViews = camImageWidth - overlapImageWidth; // per image pair

  // setup paralllel optical flow
  double startOpticalFlowTime = getCurrTimeSec();
  vector<NovelViewGenerator*> novelViewGenerators(projectionImages.size());
  vector<std::thread> threads;
  for (int leftIdx = 0; leftIdx < projectionImages.size(); ++leftIdx) {
    const int rightIdx = (leftIdx + 1) % projectionImages.size();
    novelViewGenerators[leftIdx] =
      new NovelViewGeneratorAsymmetricFlow(FLAGS_side_flow_alg);
    threads.push_back(std::thread(
      prepareNovelViewGeneratorThread,
      overlapImageWidth,
      leftIdx,
      &projectionImages[leftIdx],
      &projectionImages[rightIdx],
      novelViewGenerators[leftIdx]
    ));
  }
  for (std::thread& t : threads) { t.join(); }

  opticalFlowRuntime = getCurrTimeSec() - startOpticalFlowTime;

  // lightfield/parallax formulas
  const float v =
    atanf(FLAGS_zero_parallax_dist / (FLAGS_interpupilary_dist / 2.0f));
  const float psi =
    asinf(sinf(v) * (FLAGS_interpupilary_dist / 2.0f) / FLAGS_camera_ring_radius);
  const float vergeAtInfinitySlabDisplacement =
    psi * (float(camImageWidth) / fovHorizontalRadians);
  const float theta = -M_PI / 2.0f + v + psi;
  const float zeroParallaxNovelViewShiftPixels =
    float(FLAGS_eqr_width) * (theta / (2.0f * M_PI));

  double startNovelViewTime = getCurrTimeSec();
  // a "chunk" will be just the part of the panorama formed from one pair of
  // adjacent cameras. we will stack them horizontally to build the full
  // panorama. we do this so it can be parallelized.
  vector<Mat> panoChunksL(projectionImages.size(), Mat());
  vector<Mat> panoChunksR(projectionImages.size(), Mat());
  vector<std::thread> panoThreads;
  for (int leftIdx = 0; leftIdx < projectionImages.size(); ++leftIdx) {
    panoThreads.push_back(std::thread(
      renderStereoPanoramaChunksThread,
      leftIdx,
      numCams,
      camImageWidth,
      camImageHeight,
      numNovelViews,
      fovHorizontalRadians,
      vergeAtInfinitySlabDisplacement,
      novelViewGenerators[leftIdx],
      &panoChunksL[leftIdx],
      &panoChunksR[leftIdx]
    ));
  }
  for (std::thread& t : panoThreads) { t.join(); }

  novelViewRuntime = getCurrTimeSec() - startNovelViewTime;

  for (int leftIdx = 0; leftIdx < projectionImages.size(); ++leftIdx) {
    delete novelViewGenerators[leftIdx];
  }

  panoImageL = stackHorizontal(panoChunksL);
  panoImageR = stackHorizontal(panoChunksR);

  panoImageL = offsetHorizontalWrap(panoImageL, zeroParallaxNovelViewShiftPixels);
  panoImageR = offsetHorizontalWrap(panoImageR, -zeroParallaxNovelViewShiftPixels);
}

// handles flow between the fisheye top or bottom with the left/right eye side panoramas
void poleToSideFlowThread(
    string eyeName,
    CameraMetadata anySideCamModel,
    CameraMetadata fisheyeCamModel,
    Mat* sideSphericalForEye,
    Mat* fisheyeSpherical,
    Mat* warpedSphericalForEye) {

  // crop the side panorama to the height of the pole image
  Mat croppedSideSpherical = (*sideSphericalForEye)(Rect(0, 0, fisheyeSpherical->cols, fisheyeSpherical->rows));
  croppedSideSpherical = featherAlphaChannel(croppedSideSpherical, FLAGS_std_alpha_feather_size);

  // extend the panoramas and wrap horizontally so we can avoid a seam
  const float kExtendFrac = 1.2f;
  const int extendedWidth = float(fisheyeSpherical->cols) * kExtendFrac;
  Mat extendedSideSpherical(Size(extendedWidth, fisheyeSpherical->rows), CV_8UC4);
  Mat extendedFisheyeSpherical(extendedSideSpherical.size(),  CV_8UC4);
  for (int y = 0; y < extendedSideSpherical.rows; ++y) {
    for (int x = 0; x < extendedSideSpherical.cols; ++x) {
      extendedSideSpherical.at<Vec4b>(y, x) =
        croppedSideSpherical.at<Vec4b>(y, x % fisheyeSpherical->cols);
      extendedFisheyeSpherical.at<Vec4b>(y, x) =
        fisheyeSpherical->at<Vec4b>(y, x % fisheyeSpherical->cols);
    }
  }

  imwriteExceptionOnFail(FLAGS_output_data_dir + "/flow_images/extendedSideSpherical_" + eyeName + ".png", extendedSideSpherical);
  imwriteExceptionOnFail(FLAGS_output_data_dir + "/flow_images/extendedFisheyeSpherical_" + eyeName + ".png", extendedFisheyeSpherical);

  Mat prevFisheyeFlow, prevExtendedSideSpherical, prevExtendedFisheyeSpherical;
  if (FLAGS_prev_frame_data_dir != "NONE") {
    VLOG(1) << "Reading previous frame fisheye flow results from: "
      << FLAGS_prev_frame_data_dir;
    prevFisheyeFlow = readFlowFromFile(
      FLAGS_prev_frame_data_dir + "/flow/flow_" + eyeName + ".bin");
    prevExtendedSideSpherical = imreadExceptionOnFail(
      FLAGS_prev_frame_data_dir + "/flow_images/extendedSideSpherical_" + eyeName + ".png", -1);
    prevExtendedFisheyeSpherical = imreadExceptionOnFail(
      FLAGS_prev_frame_data_dir + "/flow_images/extendedFisheyeSpherical_" + eyeName + ".png", -1);
  }

  Mat flow;
  OpticalFlowInterface* flowAlg = makeOpticalFlowByName(FLAGS_polar_flow_alg);
  flowAlg->computeOpticalFlow(
    extendedSideSpherical,
    extendedFisheyeSpherical,
    prevFisheyeFlow,
    prevExtendedSideSpherical,
    prevExtendedFisheyeSpherical,
    flow);
  delete flowAlg;

  VLOG(1) << "Serializing fisheye flow result";
  saveFlowToFile(
    flow,
    FLAGS_output_data_dir + "/flow/flow_" + eyeName + ".bin");

  // make a ramp for alpha/flow magnitude
  const float kRampFrac = 1.0f; // fraction of available overlap used for ramp
  const float phiFromPole = fisheyeCamModel.fisheyeFovDegreesCrop / 2.0f;
  const float phiFromSide =
    90.0f - (anySideCamModel.fovHorizontal / anySideCamModel.aspectRatioWH) / 2.0f;
  const float phiMid = (phiFromPole + phiFromSide) / 2.0f;
  const float phiDiff = fabsf(phiFromPole - phiFromSide);
  const float phiRampStart = phiMid - kRampFrac * phiDiff / 2.0f;
  const float phiRampEnd = phiMid + kRampFrac * phiDiff / 2.0f;

  Mat warp(extendedFisheyeSpherical.size(), CV_32FC2);
  for (int y = 0; y < warp.rows; ++y) {
    for (int x = 0; x < warp.cols; ++x) {
      const float phi =
        fisheyeCamModel.fisheyeFovDegrees * 0.5f * float(y) / float(warp.rows - 1);
      const float alpha = 1.0f - rampf(phi, phiRampStart, phiMid);
      warp.at<Point2f>(y, x) = Point2f(x, y) + (1.0f - alpha) * flow.at<Point2f>(y, x);
    }
  }

  Mat warpedExtendedFisheyeSpherical;
  remap(
    extendedFisheyeSpherical,
    warpedExtendedFisheyeSpherical,
    warp,
    Mat(),
    CV_INTER_CUBIC,
    BORDER_CONSTANT);

  // take the extra strip on the right side and alpha-blend it out on the left side of the result
  *warpedSphericalForEye = warpedExtendedFisheyeSpherical(Rect(0, 0, fisheyeSpherical->cols, fisheyeSpherical->rows));
  int maxBlendX = float(fisheyeSpherical->cols) * (kExtendFrac - 1.0f);
  for (int y = 0; y < warpedSphericalForEye->rows; ++y) {
    for (int x = 0; x < maxBlendX; ++x) {
      const float srcB = warpedSphericalForEye->at<Vec4b>(y, x)[0];
      const float srcG = warpedSphericalForEye->at<Vec4b>(y, x)[1];
      const float srcR = warpedSphericalForEye->at<Vec4b>(y, x)[2];
      const float srcA = warpedSphericalForEye->at<Vec4b>(y, x)[3];
      const float wrapB = warpedExtendedFisheyeSpherical.at<Vec4b>(y, x + fisheyeSpherical->cols)[0];
      const float wrapG = warpedExtendedFisheyeSpherical.at<Vec4b>(y, x + fisheyeSpherical->cols)[1];
      const float wrapR = warpedExtendedFisheyeSpherical.at<Vec4b>(y, x + fisheyeSpherical->cols)[2];
      float alpha = 1.0f - rampf(x, float(maxBlendX) * 0.333f, float(maxBlendX) * 0.667f);
      warpedSphericalForEye->at<Vec4b>(y, x) = Vec4b(
        wrapB * alpha + srcB * (1.0f - alpha),
        wrapG * alpha + srcG * (1.0f - alpha),
        wrapR * alpha + srcR * (1.0f - alpha),
        srcA);
    }
  }

  // make a ramp in the alpha channel for blending with the sides
  for (int y = 0; y < warp.rows; ++y) {
    for (int x = 0; x < warp.cols; ++x) {
      const float phi =
        fisheyeCamModel.fisheyeFovDegrees * 0.5f * float(y) / float(warp.rows - 1);
      const float alpha = 1.0f - rampf(phi, phiMid, phiRampEnd);
      (*warpedSphericalForEye).at<Vec4b>(y, x)[3] = 255.0f * alpha;
    }
  }

  copyMakeBorder(
    *warpedSphericalForEye,
    *warpedSphericalForEye,
    0,
    sideSphericalForEye->rows - warpedSphericalForEye->rows,
    0,
    0,
    BORDER_CONSTANT,
    Scalar(0,0,0,0));

  if (FLAGS_save_debug_images) {
    imwriteExceptionOnFail(
      FLAGS_output_data_dir + "/croppedSideSpherical_" + eyeName + ".png",
      croppedSideSpherical);
    imwriteExceptionOnFail(
      FLAGS_output_data_dir + "/warpedSpherical_" + eyeName + ".png",
      *warpedSphericalForEye);
    imwriteExceptionOnFail(
      FLAGS_output_data_dir + "/extendedSideSpherical_" + eyeName + ".png",
      extendedSideSpherical);
  }
}

// does pole removal from the two bottom cameras, and projects the result to equirect
void prepareBottomImagesThread(
    vector<CameraMetadata> camModelArrayWithTop,
    CameraMetadata* bottomCamModel,
    Mat* bottomSpherical) {

  Mat bottomImage;
  if (FLAGS_enable_pole_removal) {
    LOG(INFO) << "Using pole removal masks";
    requireArg(FLAGS_bottom_pole_masks_dir, "bottom_pole_masks_dir");
    combineBottomImagesWithPoleRemoval(
      FLAGS_imgs_dir,
      FLAGS_bottom_pole_masks_dir,
      FLAGS_prev_frame_data_dir,
      FLAGS_output_data_dir,
      FLAGS_save_debug_images,
      true, // save data that will be used in the next frame
      FLAGS_poleremoval_flow_alg,
      FLAGS_std_alpha_feather_size,
      camModelArrayWithTop,
      *bottomCamModel,
      bottomImage);
  } else {
    LOG(INFO) << "Using primary bottom camera";
    *bottomCamModel = getBottomCamModel(camModelArrayWithTop);
    const string bottomImageFilename = bottomCamModel->cameraId + ".png";
    const string bottomImagePath = FLAGS_imgs_dir + "/" + bottomImageFilename;
    bottomImage = imreadExceptionOnFail(bottomImagePath, CV_LOAD_IMAGE_COLOR);
  }

  *bottomSpherical = bicubicRemapFisheyeToSpherical(
    *bottomCamModel,
    bottomImage,
    Size(
      FLAGS_eqr_width,
      FLAGS_eqr_height * (bottomCamModel->fisheyeFovDegrees / 2.0f) / 180.0f));

  // if we skipped pole removal, there is no alpha channel and we need to add one.
  if (bottomSpherical->type() != CV_8UC4) {
    cvtColor(*bottomSpherical, *bottomSpherical, CV_BGR2BGRA);
  }

  // the alpha channel in bottomSpherical is the result of pole removal/flow. this can in
  // some cases cause an alpha-channel discontinuity at the boundary of the image, which
  // will have an effect on flow between bottom and sides. to mitigate that, we do another
  // pass of feathering on bottomSpherical before converting to polar coordinates.
  const int yFeatherStart = bottomSpherical->rows - 1 - FLAGS_std_alpha_feather_size;
  for (int y = yFeatherStart; y < bottomSpherical->rows; ++y) {
    for (int x = 0; x < bottomSpherical->cols; ++x) {
      const float alpha =
        1.0f - float(y - yFeatherStart) / float(FLAGS_std_alpha_feather_size);
      bottomSpherical->at<Vec4b>(y, x)[3] =
        min(bottomSpherical->at<Vec4b>(y, x)[3], (unsigned char)(255.0f * alpha));
    }
  }

  if (FLAGS_save_debug_images) {
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/_bottomSpherical.png", *bottomSpherical);
  }
}

// similar to prepareBottomImagesThread but there is no pole removal
void prepareTopImagesThread(
    CameraMetadata topCamModel,
    Mat* topSpherical) {

  const string topImageFilename = topCamModel.cameraId + ".png";
  const string topImagePath = FLAGS_imgs_dir + "/" + topImageFilename;
  Mat topImage = imreadExceptionOnFail(topImagePath, CV_LOAD_IMAGE_COLOR);
  *topSpherical = bicubicRemapFisheyeToSpherical(
    topCamModel,
    topImage,
    Size(
      FLAGS_eqr_width,
      FLAGS_eqr_height * (topCamModel.fisheyeFovDegrees / 2.0f) / 180.0f));

  // alpha feather the top spherical image for flow purposes
  cvtColor(*topSpherical, *topSpherical, CV_BGR2BGRA);
  const int yFeatherStart = topSpherical->rows - 1 - FLAGS_std_alpha_feather_size;
  for (int y = yFeatherStart ; y < topSpherical->rows ; ++y) {
    for (int x = 0; x < topSpherical->cols; ++x) {
      const float alpha =
        1.0f - float(y - yFeatherStart) / float(FLAGS_std_alpha_feather_size);
      topSpherical->at<Vec4b>(y, x)[3] = 255.0f * alpha;
    }
  }

  if (FLAGS_save_debug_images) {
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/_topSpherical.png", *topSpherical);
  }
}

// sharpen the left or right eye panorama using a periodic boundary
void sharpenThread(Mat* sphericalImage) {
  const WrapBoundary<float> wrapB;
  const ReflectBoundary<float> reflectB;
  Mat lowPassSphericalImage(sphericalImage->rows, sphericalImage->cols, CV_8UC3);
  iirLowPass<WrapBoundary<float>, ReflectBoundary<float>, Vec3b>(
    *sphericalImage, 0.25f, lowPassSphericalImage, wrapB, reflectB);
  sharpenWithIirLowPass<Vec3b>(
    *sphericalImage, lowPassSphericalImage, 1.0f + FLAGS_sharpenning);
}

// run the whole stereo panorama rendering pipeline
void renderStereoPanorama() {
  requireArg(FLAGS_src_intrinsic_param_file, "src_intrinsic_param_file");
  requireArg(FLAGS_rig_json_file, "rig_json_file");
  requireArg(FLAGS_imgs_dir, "imgs_dir");
  requireArg(FLAGS_output_data_dir, "output_data_dir");
  requireArg(FLAGS_output_equirect_path, "output_equirect_path");

  const double startTime = getCurrTimeSec();

  // load camera meta data and source images
  VLOG(1) << "Reading camera model json";
  vector<CameraMetadata> camModelArrayWithTop =
    readCameraProjectionModelArrayFromJSON(FLAGS_rig_json_file);

  VLOG(1) << "Verifying image filenames";
  verifyImageDirFilenamesMatchCameraArray(camModelArrayWithTop, FLAGS_imgs_dir);

  VLOG(1) << "Removing top and bottom cameras";
  vector<CameraMetadata> camModelArray =
    removeTopAndBottomFromCamArray(camModelArrayWithTop);

  if (FLAGS_eqr_width % camModelArray.size() != 0) {
    VLOG(1) << "Number of side cameras:" << camModelArray.size();
    VLOG(1) << "Suggested widths:";
    for (int i = FLAGS_eqr_width * 0.9; i < FLAGS_eqr_width * 1.1; ++i) {
      if (i % camModelArray.size() == 0) {
        VLOG(1) << i;
      }
    }
    throw VrCamException("eqr_width must be evenly divisible by the number of cameras");
  }

  // prepare the bottom camera(s) by doing pole removal and projections in a thread.
  // will join that thread as late as possible.
  CameraMetadata bottomCamModel;
  Mat bottomImage, bottomSpherical;
  std::thread prepareBottomThread;
  if (FLAGS_enable_bottom) {
    VLOG(1) << "Bottom cameras enabled. Preparing bottom projections in a thread";
    prepareBottomThread = std::thread(
      prepareBottomImagesThread,
      camModelArrayWithTop,
      &bottomCamModel,
      &bottomSpherical);
  }

  // top cameras are handled similar to bottom cameras- do anything we can in a thread
  // that is joined as late as possible.
  CameraMetadata topCamModel;
  Mat topSpherical;
  std::thread prepareTopThread;
  if (FLAGS_enable_top) {
    topCamModel = getTopCamModel(camModelArrayWithTop);
    prepareTopThread = std::thread(
      prepareTopImagesThread,
      topCamModel,
      &topSpherical);
  }

  // read bundle adjustment for side cameras
  vector<Mat> sideCamTransforms;
  if (FLAGS_ring_rectify_file == "NONE") {
    LOG(WARNING) << "No ring rectification file specified";
    for (int i = 0; i < camModelArray.size(); ++i) {
      sideCamTransforms.push_back(Mat());
    }
  } else {
    VLOG(1) << "Reading ring rectification file: " << FLAGS_ring_rectify_file;
    FileStorage fileStorage(FLAGS_ring_rectify_file, FileStorage::READ);
    if (!fileStorage.isOpened()) {
      throw VrCamException("file read failed: " + FLAGS_ring_rectify_file);
    }

    for (int i = 0; i < camModelArray.size(); ++i) {
      Mat transformForCamI;
      fileStorage[camModelArray[i].cameraId] >> transformForCamI;
      sideCamTransforms.push_back(transformForCamI);
    }
  }

  // projection to spherical coordinates
  vector<Mat> projectionImages;

  if (FLAGS_save_debug_images) {
    system(string("rm -f " + FLAGS_output_data_dir + "/projections/*").c_str());
  }

  const double startProjectSphericalTime = getCurrTimeSec();
  LOG(INFO) << "Projecting camera images to spherical";
  projectSphericalCamImages(
    camModelArray,
    sideCamTransforms,
    FLAGS_imgs_dir,
    projectionImages);
  const double endProjectSphericalTime = getCurrTimeSec();

  // generate novel views and stereo spherical panoramas
  double opticalFlowRuntime, novelViewRuntime;
  Mat sphericalImageL, sphericalImageR;
  LOG(INFO) << "Rendering stereo panorama";
  generateRingOfNovelViewsAndRenderStereoSpherical(
    camModelArray[0].fovHorizontal,
    projectionImages,
    sphericalImageL,
    sphericalImageR,
    opticalFlowRuntime,
    novelViewRuntime);

  if (FLAGS_save_debug_images) {
    VLOG(1) << "Offset-warping images for debugging";
    Mat wrapSphericalImageL, wrapSphericalImageR;
    wrapSphericalImageL = offsetHorizontalWrap(sphericalImageL, sphericalImageL.cols/3);
    wrapSphericalImageR = offsetHorizontalWrap(sphericalImageR, sphericalImageR.cols/3);
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/sphericalImgL.png", sphericalImageL);
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/sphericalImgR.png", sphericalImageR);
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/sphericalImg_offsetwrapL.png", wrapSphericalImageL);
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/sphericalImg_offsetwrapR.png", wrapSphericalImageR);
  }

  // if both top and bottom cameras are enabled, there are 4 threads that can be done in
  // parallel (for top/bottom, we flow to the left eye and right eye side panoramas).
  std::thread topFlowThreadL, topFlowThreadR, bottomFlowThreadL, bottomFlowThreadR;
  const double topBottomToSideStartTime = getCurrTimeSec();

  // if we have a top camera, do optical flow with its image and the side camera
  Mat topSphericalWarpedL, topSphericalWarpedR;
  if (FLAGS_enable_top) {
    prepareTopThread.join(); // this is the latest we can wait

    topFlowThreadL = std::thread(
      poleToSideFlowThread,
      "top_left",
      camModelArray[0],
      topCamModel,
      &sphericalImageL,
      &topSpherical,
      &topSphericalWarpedL);

    topFlowThreadR = std::thread(
      poleToSideFlowThread,
      "top_right",
      camModelArray[0],
      topCamModel,
      &sphericalImageR,
      &topSpherical,
      &topSphericalWarpedR);
  }

  Mat flipSphericalImageL, flipSphericalImageR;
  Mat bottomSphericalWarpedL, bottomSphericalWarpedR;
  if (FLAGS_enable_bottom) {
    prepareBottomThread.join(); // this is the latest we can wait

    // flip the side images upside down for bottom flow
    flip(sphericalImageL, flipSphericalImageL, -1);
    flip(sphericalImageR, flipSphericalImageR, -1);

    bottomFlowThreadL = std::thread(
      poleToSideFlowThread,
      "bottom_left",
      camModelArray[0],
      bottomCamModel,
      &flipSphericalImageL,
      &bottomSpherical,
      &bottomSphericalWarpedL);

    bottomFlowThreadR = std::thread(
      poleToSideFlowThread,
      "bottom_right",
      camModelArray[0],
      bottomCamModel,
      &flipSphericalImageR,
      &bottomSpherical,
      &bottomSphericalWarpedR);
  }

  // now that all 4 possible threads have been spawned, we are ready to wait for the
  // threads to finish, then composite the results
  if (FLAGS_enable_top) {
    topFlowThreadL.join();
    topFlowThreadR.join();

    sphericalImageL = flattenLayersDeghostPreferBase(sphericalImageL, topSphericalWarpedL);
    sphericalImageR = flattenLayersDeghostPreferBase(sphericalImageR, topSphericalWarpedR);
  }

  if (FLAGS_enable_bottom) {
    bottomFlowThreadL.join();
    bottomFlowThreadR.join();

    flip(sphericalImageL, sphericalImageL, -1);
    flip(sphericalImageR, sphericalImageR, -1);
    sphericalImageL = flattenLayersDeghostPreferBase(sphericalImageL, bottomSphericalWarpedL);
    sphericalImageR = flattenLayersDeghostPreferBase(sphericalImageR, bottomSphericalWarpedR);
    flip(sphericalImageL, sphericalImageL, -1);
    flip(sphericalImageR, sphericalImageR, -1);
  }
  const double topBottomToSideEndTime = getCurrTimeSec();

  // depending on how things are handled, we might still have an alpha channel.
  // if so, flatten the image to 3 channel
  if (sphericalImageL.type() != CV_8UC3) {
    VLOG(1) << "Flattening from 4 channels to 3 channels";
    cvtColor(sphericalImageL, sphericalImageL, CV_BGRA2BGR);
    cvtColor(sphericalImageR, sphericalImageR, CV_BGRA2BGR);
  }

  if (FLAGS_save_debug_images) {
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/eqr_sideL.png", sphericalImageL);
    imwriteExceptionOnFail(FLAGS_output_data_dir + "/eqr_sideR.png", sphericalImageR);
  }

  const double startSharpenTime = getCurrTimeSec();
  if (FLAGS_sharpenning > 0.0f) {
    VLOG(1) << "Sharpening";
    std::thread sharpenThreadL(sharpenThread, &sphericalImageL);
    std::thread sharpenThreadR(sharpenThread, &sphericalImageR);
    sharpenThreadL.join();
    sharpenThreadR.join();
    if (FLAGS_save_debug_images) {
      imwriteExceptionOnFail(FLAGS_output_data_dir + "/_eqr_sideL_sharpened.png", sphericalImageL);
      imwriteExceptionOnFail(FLAGS_output_data_dir + "/_eqr_sideR_sharpened.png", sphericalImageR);
    }
  }
  const double endSharpenTime = getCurrTimeSec();

  // project the horizontal panoramas to cubemaps and composite the top
  const double startCubemapTime = getCurrTimeSec();
  if (FLAGS_cubemap_face_resolution > 0 && !FLAGS_output_cubemap_path.empty()) {
    LOG(INFO) << "Generating stereo cubemap";
    Mat cubemapImageL = stackOutputCubemapFaces(
        FLAGS_cubemap_format,
        convertSphericalToCubemapBicubicRemap(
          sphericalImageL, M_PI, FLAGS_cubemap_face_resolution));
    Mat cubemapImageR = stackOutputCubemapFaces(
        FLAGS_cubemap_format, convertSphericalToCubemapBicubicRemap(
          sphericalImageR, M_PI, FLAGS_cubemap_face_resolution));
    Mat stereoCubemap = stackVertical(vector<Mat>({cubemapImageL, cubemapImageR}));
    imwriteExceptionOnFail(FLAGS_output_cubemap_path, stereoCubemap);
  }
  const double endCubemapTime = getCurrTimeSec();

  if (FLAGS_final_eqr_width != 0 &&
      FLAGS_final_eqr_height != 0 &&
      FLAGS_final_eqr_width != FLAGS_eqr_width &&
      FLAGS_final_eqr_height != FLAGS_eqr_height / 2) {
    VLOG(1) << "Resizing before final equirect stack (for proper video size)";
    resize(
      sphericalImageL,
      sphericalImageL,
      Size(FLAGS_final_eqr_width, FLAGS_final_eqr_height / 2),
      0,
      0,
      INTER_CUBIC);
    resize(
      sphericalImageR,
      sphericalImageR,
      Size(FLAGS_final_eqr_width, FLAGS_final_eqr_height / 2),
      0,
      0,
      INTER_CUBIC);
  }

  LOG(INFO) << "Creating stereo equirectangular image";
  Mat stereoEquirect = stackVertical(vector<Mat>({sphericalImageL, sphericalImageR}));
  imwriteExceptionOnFail(FLAGS_output_equirect_path, stereoEquirect);

  const double endTime = getCurrTimeSec();
  VLOG(1) << "--- Runtime breakdown (sec) ---";
  VLOG(1) << "Total:\t\t\t" << endTime - startTime;
  VLOG(1) << "Spherical projection:\t" << endProjectSphericalTime - startProjectSphericalTime;
  VLOG(1) << "Side optical flow:\t\t" << opticalFlowRuntime;
  VLOG(1) << "Novel view panorama:\t" << novelViewRuntime;
  VLOG(1) << "Flow top+bottom with sides:\t" << topBottomToSideEndTime - topBottomToSideStartTime;
  VLOG(1) << "Sharpen:\t\t" << endSharpenTime - startSharpenTime;
  VLOG(1) << "Equirect -> Cubemap:\t" << endCubemapTime - startCubemapTime;
}

int main(int argc, char** argv) {
  initSurround360(argc, argv);
  renderStereoPanorama();
  return EXIT_SUCCESS;
}
