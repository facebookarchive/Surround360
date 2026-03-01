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

#include <Eigen/Geometry>

#include <glog/logging.h>

#ifndef WIN32

#include <folly/dynamic.h>
#include <folly/FileUtil.h>
#include <folly/json.h>

using folly::dynamic;

#else // WIN32

#ifndef M_PI
#define M_PI EIGEN_PI
#endif

// windows doesn't need the full rig i/o functionality
#define SUPPRESS_RIG_IO

// dynamic is an interposer that mimics folly::dynamic

#include <boost/property_tree/json_parser.hpp>

struct dynamic : public boost::property_tree::ptree {
  dynamic(const boost::property_tree::ptree& tree) : boost::property_tree::ptree(tree) {}

  dynamic operator[](const char* key) const {
    return get_child(key);
  }

  dynamic operator[](const std::size_t index) const {
    auto it = begin();
    for (int i = 0; i < index; ++i) {
      CHECK(it != end());
      ++it;
    }
    return it->second;
  }

  std::string getString() const {
    return get_value<std::string>();
  }

  double asDouble() const {
    return get_value<double>();
  }

  friend std::ostream& operator<<(std::ostream& s, const dynamic& dyn) {
    for (const auto& pair : dyn) {
      s << pair.first << "->" << pair.second.get_value<std::string>() << ", ";
    }
    return s;
  }
};

#endif // WIN32

namespace surround360 {

struct Camera {
  using Real = double;
  using Vector2 = Eigen::Matrix<Real, 2, 1>;
  using Vector3 = Eigen::Matrix<Real, 3, 1>;
  using Matrix3 = Eigen::Matrix<Real, 3, 3>;
  using Ray = Eigen::ParametrizedLine<Real, 3>;
  using Rig = std::vector<Camera>;
  static const Camera::Real kNearInfinity;

  // member variables
  enum struct Type { FTHETA, RECTILINEAR } type;

  Vector3 position;
  Matrix3 rotation;

  Vector2 resolution;

  Vector2 principal;
  Vector2 distortion;
  Vector2 focal;
  Real fovThreshold; // cos(fov) * abs(cos(fov))

  std::string id;
  std::string group;

  // construction and de/serialization
  Camera(const Type type, const Vector2& resolution, const Vector2& focal);
  Camera(const dynamic& json);
  dynamic serialize() const;
  static Rig loadRig(const std::string& filename);
  static void saveRig(const std::string& filename, const Rig& rig);
  static Camera createRescaledCamera(const Camera& cam, const float scale);

  // access rotation as forward/up/right vectors
  Vector3 forward() const { return -backward(); }
  Vector3 up() const { return rotation.row(1); }
  Vector3 right() const { return rotation.row(0); }
  void setRotation(
    const Vector3& forward,
    const Vector3& up,
    const Vector3& right);
  void setRotation(const Vector3& forward, const Vector3& up);

  // access rotation as angle * axis
  Vector3 getRotation() const;
  void setRotation(const Vector3& angleAxis);

  // access focal as a scalar (x right, y down, square pixels)
  void setScalarFocal(const Real& scalar);
  Real getScalarFocal() const;

  // access fov (measured in radians from optical axis)
  void setFov(const Real& radians);
  Real getFov() const;
  void setDefaultFov();
  bool isDefaultFov() const;

  // compute pixel coordinates
  Vector2 pixel(const Vector3& rig) const {
    // transform from rig to camera space
    Vector3 camera = rotation * (rig - position);
    // transform from camera to distorted sensor coordinates
    Vector2 sensor = cameraToSensor(camera);
    // transform from sensor coordinates to pixel coordinates
    return focal.cwiseProduct(sensor) + principal;
  }

  // compute rig coordinates, returns a ray, inverse of pixel()
  Ray rig(const Vector2& pixel) const {
    // transform from pixel to distorted sensor coordinates
    Vector2 sensor = (pixel - principal).cwiseQuotient(focal);
    // transform from distorted sensor coordinates to unit camera vector
    Vector3 unit = sensorToCamera(sensor);
    // transform from camera space to rig space
    return Ray(position, rotation.transpose() * unit);
  }

  // compute rig coordinates for point near infinity, inverse of pixel()
  Vector3 rigNearInfinity(const Vector2& pixel) const {
    return rig(pixel).pointAt(kNearInfinity);
  }

  bool isBehind(const Vector3& rig) const {
    return backward().dot(rig - position) >= 0;
  }

  bool isOutsideFov(const Vector3& rig) const {
    if (fovThreshold == -1) {
      return false;
    }
    if (fovThreshold == 0) {
      return isBehind(rig);
    }
    Vector3 v = rig - position;
    Real dot = -backward().dot(v);
    return dot * std::abs(dot) <= fovThreshold * v.squaredNorm();
  }

  bool sees(const Vector3& rig) const {
    if (isOutsideFov(rig)) {
      return false;
    }
    Vector2 p = pixel(rig);
    return
      0 <= p.x() && p.x() < resolution.x() &&
      0 <= p.y() && p.y() < resolution.y();
  }

  // estimate the fraction of the frame that is covered by the other camera
  Real overlap(const Camera& other) const {
    // just brute force probeCount x probeCount points
    const int kProbeCount = 10;
    int inside = 0;
    for (int y = 0; y < kProbeCount; ++y) {
      for (int x = 0; x < kProbeCount; ++x) {
        Vector2 p(x, y);
        p /= kProbeCount - 1;
        if (other.sees(rigNearInfinity(p.cwiseProduct(resolution)))) {
          ++inside;
        }
      }
    }
    return inside / Real(kProbeCount * kProbeCount);
  }

  // sample the camera's fov cone to find the closest point to the image center
  static float approximateUsablePixelsRadius(const Camera& camera) {
    const Camera::Real fov = camera.getFov();
    const Camera::Real kStep = 2 * M_PI / 10.0;
    Camera::Real result = camera.resolution.norm();
    for (Camera::Real a = 0; a < 2 * M_PI; a += kStep) {
      Camera::Vector3 ortho = cos(a) * camera.right() + sin(a) * camera.up();
      Camera::Vector3 direction = cos(fov) * camera.forward() + sin(fov) * ortho;
      Camera::Vector2 pixel = camera.pixel(camera.position + direction);
      result = std::min(result, (pixel - camera.resolution / 2.0).norm());
    }
    return result;
  }

  static void unitTest();

 private:
  Vector3 backward() const { return rotation.row(2); }

  // distortion is modeled in pixel space as:
  //   distort(r) = r + d0 * r^3 + d1 * r^5
  Real distort(Real r) const {
    return distortFactor(r * r) * r;
  }

  Real distortFactor(Real rSquared) const {
    return 1 + rSquared * (distortion[0] + rSquared * distortion[1]);
  }

  Real undistort(Real d) const {
    if (distortion.isZero()) {
      return d; // short circuit common case
    }
    // solve d = distort(r) for r using newton's method
    Real r0 = d;
    const Real smidgen = 1.0 / kNearInfinity;
    const int kMaxSteps = 10;
    for (int step = 0; step < kMaxSteps; ++step) {
      Real d0 = distort(r0);
      if (std::abs(d0 - d) < smidgen)
        break; // close enough
      // probably ok to assume derivative == 1, but let's do the right thing
      Real r1 = r0 + smidgen;
      Real d1 = distort(r1);
      Real derivative = (d1 - d0) / smidgen;
      r0 -= (d0 - d) / derivative;
    }
    return r0;
  }

  Vector2 cameraToSensor(const Vector3& camera) const {
    if (type == Type::FTHETA) {
      Real norm = camera.head<2>().norm();
      Real r = atan2(norm, -camera.z());
      return distort(r) / norm * camera.head<2>();
    } else {
      CHECK(type == Type::RECTILINEAR) << "unexpected: " << int(type);
      // project onto z = -1 plane
      Vector2 planar = camera.head<2>() / -camera.z();
      return distortFactor(planar.squaredNorm()) * planar;
    }
  }

  // compute unit vector in camera coordinates
  Vector3 sensorToCamera(const Vector2& sensor) const {
    Real squaredNorm = sensor.squaredNorm();
    if (squaredNorm == 0) {
      // avoid divide-by-zero later
      return Vector3(0, 0, -1);
    }
    Real norm = sqrt(squaredNorm);
    Real r = undistort(norm);
    Real angle;
    if (type == Type::FTHETA) {
      angle = r;
    } else {
      CHECK(type == Type::RECTILINEAR) << "unexpected: " << int(type);
      angle = atan(r);
    }
    Vector3 unit;
    unit.head<2>() = sin(angle) / norm * sensor;
    unit.z() = -cos(angle);

    return unit;
  }

  Vector3 pixelToCamera(const Vector2& pixel) const {
    // transform from pixel to distorted sensor coordinates
    Vector2 sensor = (pixel - principal).cwiseQuotient(focal);
    // transform from distorted sensor coordinates to unit camera vector
    return sensorToCamera(sensor);
  }

  template <typename V>
  static dynamic serializeVector(const V& v) {
    return dynamic(v.data(), v.data() + v.size());
  }

  template <int kSize>
  static Eigen::Matrix<Real, kSize, 1> deserializeVector(
      const dynamic& json) {
    CHECK_EQ(kSize, json.size()) << "bad vector" << json;
    Eigen::Matrix<Real, kSize, 1> result;
    for (int i = 0; i < kSize; ++i) {
      result[i] = json[i].asDouble();
    }
    return result;
  }

  static std::string serializeType(const Type& type) {
    if (type == Type::FTHETA) {
      return "FTHETA";
    } else {
      CHECK(type == Type::RECTILINEAR) << "unexpected: " << int(type);
      return "RECTILINEAR";
    }
  }

  static Type deserializeType(const dynamic& json) {
    for (int i = 0; ; ++i) {
      if (serializeType(Type(i)) == json.getString()) {
        return Type(i);
      }
    }
  }
};

} // namespace surround360
