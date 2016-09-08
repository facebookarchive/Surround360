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

#include "CvUtil.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace surround360 {
namespace linear_regression {

using namespace std;
using namespace cv;

inline float dot(
    const int& dim,
    const vector<float>& a,
    const vector<float>& b) {

  float sum = 0.0f;
  for (int i = 0; i < dim; ++i) {
    sum += a[i] * b[i];
  }
  return sum;
}

// solve a linear regression problem from R^d to R^k.
static vector<vector<float>> solveLinearRegressionRdToRk(
    const int d,
    const int k,
    const vector<vector<float>>& x,
    const vector<vector<float>>& y,
    const int numItersGradientDescent,
    const float stepSize,
    const bool printObjective) {

  assert(x.size() == y.size());
  const int n = x.size();

  // w[j] for j = 0...k-1 is the weight vector for output j. it is a vector of
  // dimension d (the input dimension).
  vector<vector<float>> w(k, vector<float>(d, 0));
  for (int itr = 0; itr < numItersGradientDescent; ++itr) {
    if (printObjective) {
      float objective = 0.0f;
      for (int j = 0; j < k; ++j) {
        for (int i = 0; i < n; ++i) {
          const float fx = dot(d, w[j], x[i]);
          objective += (y[i][j] - fx) * (y[i][j] - fx);
        }
      }
      objective /= float(n);
      cout << itr << "\t" << objective << endl; // cout (not LOG) is intentional
    }

    // compute the gradient of the loss function
    vector<vector<float>> g(k, vector<float>(d, 0));
    for (int j = 0; j < k; ++j) {
      for (int i = 0; i < n; ++i) {
        const float fx = dot(d, w[j], x[i]);
        const float s = y[i][j] - fx;
        for (int l = 0; l < d; ++l) {
          g[j][l] -= s * x[i][l] / float(n);
        }
      }
    }

    // take a gradient descent step
    for (int j = 0; j < k; ++j) {
      for (int l = 0; l < d; ++l) {
        w[j][l] -= stepSize * g[j][l];
      }
    }
  }

  return w;
}

// apply a model w=[w1...wk] mapping from R^d to R^k
inline void applyLinearModelRdToRk(
    const int& d,
    const int& k,
    const vector<vector<float>>& w,
    const vector<float>& x,
    vector<float>& y) {

  for (int j = 0; j < k; ++j) {
    y[j] = dot(d, w[j], x);
  }
}

} // namespace linear_regression
} // namespace surround360
