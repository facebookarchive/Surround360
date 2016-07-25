/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "CameraMetadata.h"

#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

#include "thirdparty/supereasyjson/json.h"

#include "CvUtil.h"
#include "StringUtil.h"
#include "SystemUtil.h"
#include "VrCamException.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace surround360 {
namespace calibration {

using namespace cv;
using namespace std;
using namespace util;

vector<CameraMetadata> readCameraProjectionModelArrayFromJSON(
    const string& jsonFilePath) {

  // read the file into a string
  ifstream fs(jsonFilePath);
  if (!fs) {
    throw VrCamException("file read failed: " + jsonFilePath);
  }

  string jsonStr((istreambuf_iterator<char>(fs)), istreambuf_iterator<char>());
  vector<CameraMetadata> cameraModels;
  try {
    json::Value jsobBlob = json::Deserialize(jsonStr);
    json::Array jsonCamArray = jsobBlob.ToArray();
    for (int i = 0 ; i < jsonCamArray.size(); ++i) {
      CameraMetadata model;
      json::Object jsonCam = jsonCamArray[i];
      model.cameraId = jsonCam["camera_id"].ToString();

      if (jsonCam.HasKey("is_bottom2")) {
        model.isBottom2 = true;
        if (jsonCam.HasKey("flip180") && jsonCam["flip180"].ToBool()) {
          model.flip180 = true;
        }
        if (jsonCam.HasKey("fisheye_rotation_deg")) {
          throw VrCamException(
            "fisheye_rotation_deg is not supported for is_bottom2 cameras");
        }
      } else { // not a secondary bottom camera
        if (jsonCam.HasKey("flip180")) {
          throw VrCamException("flip180 option is only supported for is_bottom2 cameras");
        }
      }
      if (jsonCam.HasKey("is_fisheye") && jsonCam["is_fisheye"].ToBool()) {
        model.isFisheye = true;
        model.fisheyeFovDegrees = jsonCam["fisheye_fov_deg"].ToFloat();
        model.fisheyeFovDegreesCrop = jsonCam["fisheye_fov_deg_crop"].ToFloat();
        model.imageCenterX = jsonCam["image_center_x"].ToFloat();
        model.imageCenterY = jsonCam["image_center_y"].ToFloat();
        model.usablePixelsRadius = jsonCam["usable_pixels_radius"].ToFloat();
        if (!model.isBottom2) {
          model.fisheyeRotationDegrees = jsonCam["fisheye_rotation_deg"].ToFloat();
        }
        model.fovHorizontal = model.fisheyeFovDegrees;
      } else {
        model.isFisheye = false;
        model.fovHorizontal = jsonCam["fov_horizontal"].ToFloat();
        model.aspectRatioWH = jsonCam["aspect_ratio_wh"].ToFloat();
      }
      if (jsonCam.HasKey("is_top") && jsonCam["is_top"].ToBool()) {
        model.isTop = true;
      }
      if (jsonCam.HasKey("is_bottom") && jsonCam["is_bottom"].ToBool()) {
        model.isBottom = true;
      }
      cameraModels.push_back(model);
    }
  } catch(VrCamException& e) {
    throw e;
  } catch(...) {
    throw VrCamException("failed to parse json camera array description");
  }
  return cameraModels;
}

void verifyImageDirFilenamesMatchCameraArray(
    const vector<CameraMetadata>& cameraArray,
    const string& imageDir) {

  map<string, bool> cameraNameToFileOK;
  for (const CameraMetadata& cam : cameraArray) {
    cameraNameToFileOK[cam.cameraId] =  false;
  }

  vector<string> imageFilenames = getFilesInDir(imageDir, false);
  for (const string& imageFilename : imageFilenames) {
    vector<string> parts = stringSplit(imageFilename, '.');
    if (parts.size() != 2) {
      throw VrCamException("expected exactly one . in filename:" + imageFilename);
    }
    const string prefix = parts[0];

    if (cameraNameToFileOK.find(prefix) == cameraNameToFileOK.end()) {
      throw VrCamException("image doesn't match any camera in json: " + imageFilename);
    }

    cameraNameToFileOK[prefix] = true;
  }

  for (const CameraMetadata& cam : cameraArray) {
    if (!cameraNameToFileOK[cam.cameraId]) {
      throw VrCamException("missing image for camera: " + cam.cameraId);
    }
  }
}

void loadCameraImagePairs(
    const vector<CameraMetadata>& cameraArray,
    const string& imageDir,
    vector< pair<CameraMetadata, Mat> >& camImagePairs) {

  // build a map to lookup camera models by camera_id
  map<string, CameraMetadata> camIdToModel;
  map<string, int> camIdToIndex;
  for (int i = 0; i < cameraArray.size(); ++i) {
    const CameraMetadata& cam = cameraArray[i];
    camIdToModel[cam.cameraId] = cam;
    camIdToIndex[cam.cameraId] = i;
  }

  // figure out the file extension used by the images.. this is complicated but
  // its all so we can iterate over the images in the right order.
  // assumption: no weird mixtures of file extension
  vector<string> imageFilenames = getFilesInDir(imageDir, false);
  assert(imageFilenames.size() > 0);
  vector<string> firstFilenameParts = stringSplit(imageFilenames[0], '.');
  assert(firstFilenameParts.size() == 2);
  const string imageFileExtension = firstFilenameParts[1];

  VLOG(1) << "loadCameraImagePairs spawning threads";
  vector<std::thread> threads;
  vector<Mat> images(cameraArray.size(), Mat());
  for (int i = 0; i < cameraArray.size(); ++i) {
    const CameraMetadata& cam = cameraArray[i];
    const string imageFilename = cam.cameraId + "." + imageFileExtension;
    const string imagePath = imageDir + "/" + imageFilename;
    VLOG(1) << "imagePath = " << imagePath;
    threads.push_back(std::thread(
      imreadInStdThread,
      imagePath,
      CV_LOAD_IMAGE_COLOR,
      &(images[i])
    ));
  }

  for (int i = 0; i < cameraArray.size(); ++i) {
    threads[i].join();
    camImagePairs.push_back(make_pair(cameraArray[i], images[i]));
  }
}

vector<CameraMetadata> removeTopAndBottomFromCamArray(
    const vector<CameraMetadata>& camModelArray) {

  vector<CameraMetadata> camModelArrayNoTop;
  for (const CameraMetadata& cam : camModelArray) {
    if (!cam.isTop && !cam.isBottom && !cam.isBottom2) {
      camModelArrayNoTop.push_back(cam);
    }
  }
  return camModelArrayNoTop;
}

CameraMetadata getTopCamModel(const vector<CameraMetadata>& camModelArray) {
  for (const CameraMetadata& cam : camModelArray) {
    if (cam.isTop) {
      return cam;
    }
  }
  throw VrCamException("expected camera with 'is_top: true' in json.");
}

CameraMetadata getBottomCamModel(const vector<CameraMetadata>& camModelArray) {
  for (const CameraMetadata& cam : camModelArray) {
    if (cam.isBottom) {
      return cam;
    }
  }
  throw VrCamException("expected camera with 'is_bottom: true' in json.");
}

CameraMetadata getBottomCamModel2(const vector<CameraMetadata>& camModelArray) {
  for (const CameraMetadata& cam : camModelArray) {
    if (cam.isBottom2) {
      return cam;
    }
  }
  throw VrCamException("expected camera with 'is_bottom2: true' in json.");
}

CameraMetadata getCameraById(
    const vector<CameraMetadata>& camModelArray,
    const string& id) {

  for (const CameraMetadata& cam : camModelArray) {
    if (cam.cameraId == id) {
      return cam;
    }
  }
  throw VrCamException("camera_id " + id + " not found");
}

} // namespace calibration
} // namespace surround360
