/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#pragma once

#include <memory>
#include <ostream>
#include <vector>

#include "MathUtil.h"
#include "VrCamException.h"

namespace surround360 {
namespace math_util {

using namespace std;

template <typename T>
class MonotonicTable {
protected:
  const int size;
  const shared_ptr<vector<T> > table;
  const T minX, maxX;
  const T rangeScale;

  // Function that is used to compute the table
  virtual T f(const T x) const = 0;

public:
  MonotonicTable(const T minX_, const T maxX_, const int size_) :
    size(size_),
    table(new vector<T>),
    minX(minX_),
    maxX(maxX_),
    rangeScale(T(size-1) / (maxX - minX)) {}

  void initTable() {
    const T dx = T(1) / rangeScale;
    for (int i = 0; i < size; ++i) {
      table->push_back(f(dx*i + minX));
    }
  }

  inline int getSize() const { return size; }

  T inline operator()(const T x) const {
    const int i = clamp(int((x - minX) * rangeScale), 0, size - 1);
    return (*table)[i];
  }
};

class Linear : public MonotonicTable<float> {
private:
  vector<cv::Point3f> points_;
  int n;

protected:
  float f(const float x) const {
    if (x < points_[0].x) {
      const float alpha = (x - points_[0].x) / (points_[1].x - points_[0].x);
      return lerp(points_[0].y, points_[1].y, alpha);
    } else if (x > points_[n - 1].x) {
      const float alpha =
        (x - points_[n - 1].x) / (points_[n - 1].x - points_[n - 2].x);
      return lerp(points_[n - 2].y, points_[n - 1].y, alpha);
    } else {
      for (int i = 0; i < n - 1; ++i) {
        if (points_[i].x <= x && x <= points_[i + 1].x) {
          const float alpha =
            (x - points_[i].x) /
            (points_[i + 1].x - points_[i].x);
          return lerp(points_[i].y, points_[i + 1].y, alpha);
        }
      }
      assert(false);
      return 0.0f;
    }
  }

public:
  Linear(
      const float minX,
      const float maxX,
      const int size,
      vector<cv::Point3f >& points) :
  MonotonicTable<float>(minX, maxX, size) {

    for (auto p = points.begin(); p != points.end(); p++) {
      points_.push_back(*p);
    }
    n = points_.size();
    if (n < 2) {
      throw VrCamException("You need at least two points to have a linear curve");
    }
    initTable();
  }
};

class Power : public MonotonicTable<float> {
protected:
  const float p_;
  float f(const float x) const { return powf(x, p_); }

public:
  Power(
      const float minX,
      const float maxX,
      const int size,
      const float p) :
  MonotonicTable<float>(minX, maxX, size),
  p_(p) {
    initTable();
  }
};

class sCurve : public MonotonicTable<float> {
private:
  shared_ptr<const GaussianApproximation<float>> gain;
  const float dx;
  float sumRecip;
  float scale;
  float bias;

protected:
  float f(const float x) const {
    float sum = 0.0f;
    for (int i = 0; i < size; ++i) {
      const float xp = dx*i + minX;
      if (xp <= x) {
        sum += (*gain)(xp);
      } else {
        break;
      }
    }
    return sum * sumRecip * scale + bias;
  }

public:
  sCurve(
      const float minX,
      const float maxX,
      const float minY,
      const float maxY,
      const int size) :
  MonotonicTable<float>(minX, maxX, size),
  gain(new GaussianApproximation<float>(minX, maxX, 0.0f, 1.0f)),
  dx((maxX - minX) / float(size - 1)) {
    sumRecip  = 1.0f;
    scale     = 1.0f;
    bias      = 0.0f;
    sumRecip  = 1.0f / f(maxX);
    scale     = maxY - minY;
    bias      = minY;
    initTable();
  }
};

class Butterworth : public MonotonicTable<float> {
private:
  const int order_;
  const float cutoffFreq_;

protected:
  float f(const float x) const {
    return 1.0f / (1.0f + powf(x/cutoffFreq_, 2.0f*order_));
  }

public:
  Butterworth(
    const float minX,
    const float maxX,
    const int size,
    const float cutoffFreq,
    const int order) :
  MonotonicTable<float>(minX, maxX, size),
  order_(order),
  cutoffFreq_(cutoffFreq > 0.0f ? cutoffFreq : 1.0e-6) {
    initTable();
  }
};

class Sinc : public MonotonicTable<float> {
protected:
  float f(const float x) const {
    const float xp = M_PI * x;
    return xp == 0 ? 1.0f : sin(xp)/xp;
  }

public:
  Sinc(const float minX, const float maxX, const int size) :
    MonotonicTable<float>(minX, maxX, size) {
    initTable();
  }
};

} // end namespace math_util
} // end namespace surround360
