/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "JsonUtil.h"

#include "MathUtil.h"
#include "VrCamException.h"

namespace surround360 {

using namespace std;
using namespace cv;

void exceptionIfMissingKeys(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  if (!obj.HasKey(objKey)) {
    throw VrCamException("JSON error. missing key:" + objKey);
  }

  const json::Object obj2 = obj[objKey];
  if (!obj2.HasKey(key)) {
    throw VrCamException("JSON error. missing key:" + key);
  }
}

const json::Array getArray(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  exceptionIfMissingKeys(obj, objKey, key);
  return obj[objKey][key].ToArray();
}

const string getString(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  exceptionIfMissingKeys(obj, objKey, key);
  return obj[objKey][key].ToString();
}

const double getDouble(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  exceptionIfMissingKeys(obj, objKey, key);
  return obj[objKey][key].ToDouble();
}

const int getInteger(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  exceptionIfMissingKeys(obj, objKey, key);
  return obj[objKey][key].ToInt();
}

cv::Point3f getVector(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  exceptionIfMissingKeys(obj, objKey, key);
  json::Array vec = obj[objKey][key].ToArray();
  cv::Point3f v;
  if (vec.size() == 3) {
    v.x = vec[0].ToDouble();
    v.y = vec[1].ToDouble();
    v.z = vec[2].ToDouble();
  } else {
    throw VrCamException("JSON error. Expecting a vector to be a list with 3 elements.");
  }
  return v;
}

cv::Mat getMatrix(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  exceptionIfMissingKeys(obj, objKey, key);
  cv::Mat m(3, 3, CV_32F);
  json::Array matrix = obj[objKey][key].ToArray();
  if (matrix.size() == 3) {
    json::Array m0 = matrix[0].ToArray();
    if (m0.size() == 3) {
      m.at<float>(0, 0) = m0[0].ToDouble();
      m.at<float>(0, 1) = m0[1].ToDouble();
      m.at<float>(0, 2) = m0[2].ToDouble();
    } else {
      throw VrCamException(
        "JSON error. Expecting the first row to be a list with 3 elements.");
    }

    json::Array m1 = matrix[1].ToArray();
    if (m1.size() == 3) {
      m.at<float>(1, 0) = m1[0].ToDouble();
      m.at<float>(1, 1) = m1[1].ToDouble();
      m.at<float>(1, 2) = m1[2].ToDouble();
    } else {
      throw VrCamException(
        "JSON error. Expecting the second row to be a list with 3 elements.");
    }

    json::Array m2 = matrix[2].ToArray();
    if (m2.size() == 3) {
      m.at<float>(2, 0) = m2[0].ToDouble();
      m.at<float>(2, 1) = m2[1].ToDouble();
      m.at<float>(2, 2) = m2[2].ToDouble();
    } else {
      throw VrCamException(
        "JSON error. Expecting the third row to be a list with 3 elements.");
    }
  } else {
    throw VrCamException(
      "JSON error. Expecting a matrix being a 3 x 3 element list: "
      "[[row1], [row2], [row3]].");
  }
  return m;
}

vector<cv::Point3f> getCoordList(
    const json::Object& obj,
    const string& objKey,
    const string& key) {

  exceptionIfMissingKeys(obj, objKey, key);
  vector<cv::Point3f> v;
  json::Array vector = obj[objKey][key].ToArray();
  for (int i = 0;  i < vector.size(); ++i) {
    json::Array vl = vector[i].ToArray();
    cv::Point3f p;
    if (vl.size() == 3) {
      p.x = vl[0].ToDouble();
      p.y = vl[1].ToDouble();
      p.z = vl[2].ToDouble();
      v.push_back(p);
    } else {
      throw VrCamException("JSON error. Expecting the list with 3 element rows.");
    }
  }
  return v;
}

} // end namespace surround360
