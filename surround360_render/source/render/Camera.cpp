/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-style license found in the
* LICENSE_render file in the root directory of this subproject. An additional grant
* of patent rights can be found in the PATENTS file in the same directory.
*/

#include "Camera.h"

namespace surround360 {

const Camera::Real Camera::kNearInfinity = 1e6;

void Camera::setRotation(
    const Vector3& forward,
    const Vector3& up,
    const Vector3& right) {
  CHECK_LT(right.cross(up).dot(forward), 0) << "rotation must be right-handed";
  rotation.row(2) = -forward; // +z is back
  rotation.row(1) = up; // +y is up
  rotation.row(0) = right; // +x is right
  // re-unitarize
  const Camera::Real tol = 0.001;
  CHECK(rotation.isUnitary(tol)) << rotation << " is not close to unitary";
  Eigen::AngleAxis<Camera::Real> aa(rotation);
  rotation = aa.toRotationMatrix();
}

void Camera::setRotation(const Vector3& forward, const Vector3& up) {
  setRotation(forward, up, forward.cross(up));
}

Camera::Camera(const Type type, const Vector2& res, const Vector2& focal):
    type(type), resolution(res), focal(focal) {
  position.setZero();
  rotation.setIdentity();
  principal = resolution / 2;
  distortion.setZero();
  setDefaultFov();
}

Camera::Camera(const dynamic& json) {
  CHECK_GE(json["version"].asDouble(), 1.0);

  id = json["id"].getString();

  type = deserializeType(json["type"]);

  position = deserializeVector<3>(json["origin"]);

  setRotation(
    deserializeVector<3>(json["forward"]),
    deserializeVector<3>(json["up"]),
    deserializeVector<3>(json["right"]));

  resolution = deserializeVector<2>(json["resolution"]);

  if (json.count("principal")) {
    principal = deserializeVector<2>(json["principal"]);
  } else {
    principal = resolution / 2;
  }

  if (json.count("distortion")) {
    distortion = deserializeVector<2>(json["distortion"]);
  } else {
    distortion.setZero();
  }

  if (json.count("fov")) {
    setFov(json["fov"].asDouble());
  } else {
    setDefaultFov();
  }

  focal = deserializeVector<2>(json["focal"]);

  if (json.count("group")) {
    group = json["group"].getString();
  }
}

#ifndef SUPPRESS_RIG_IO

dynamic Camera::serialize() const {
  dynamic result = dynamic::object
    ("version", 1)
    ("type", serializeType(type))
    ("origin", serializeVector(position))
    ("forward", serializeVector(forward()))
    ("up", serializeVector(up()))
    ("right", serializeVector(right()))
    ("resolution", serializeVector(resolution))
    ("principal", serializeVector(principal))
    ("focal", serializeVector(focal))
    ("id", id);
  if (!distortion.isZero()) {
    result["distortion"] = serializeVector(distortion);
  }
  if (!isDefaultFov()) {
    result["fov"] = getFov();
  }
  if (!group.empty()) {
    result["group"] = group;
  }

  return result;
}

#endif // SUPPRESS_RIG_IO

void Camera::setRotation(const Vector3& angleAxis) {
  // convert angle * axis to rotation matrix
  Real angle = angleAxis.norm();
  Vector3 axis = angleAxis / angle;
  if (angle == 0) {
    axis = Vector3::UnitX();
  }
  rotation = Eigen::AngleAxis<Real>(angle, axis).toRotationMatrix();
}

Camera::Vector3 Camera::getRotation() const {
  // convert rotation matrix to angle * axis
  Eigen::AngleAxis<Real> angleAxis(rotation);
  if (angleAxis.angle() > M_PI) {
    angleAxis.angle() = 2 * M_PI - angleAxis.angle();
    angleAxis.axis() = -angleAxis.axis();
  }

  return angleAxis.angle() * angleAxis.axis();
}

void Camera::setScalarFocal(const Real& scalar) {
  focal = { scalar, -scalar };
}

Camera::Real Camera::getScalarFocal() const {
  CHECK_EQ(focal.x(), -focal.y()) << "pixels are not square";
  return focal.x();
}

void Camera::setFov(const Real& fov) {
  CHECK(fov <= M_PI / 2 || type == Type::FTHETA);
  Real cosFov = std::cos(fov);
  fovThreshold = cosFov * std::abs(cosFov);
}

Camera::Real Camera::getFov() const {
  return fovThreshold < 0
    ? std::acos(-std::sqrt(-fovThreshold))
    : std::acos(std::sqrt(fovThreshold));
}

void Camera::setDefaultFov() {
  if (type == Type::FTHETA) {
    fovThreshold = -1;
  } else {
    CHECK(type == Type::RECTILINEAR) << "unexpected: " << int(type);
    fovThreshold = 0;
  }
}

bool Camera::isDefaultFov() const {
  return type == Type::FTHETA ? fovThreshold == -1 : fovThreshold == 0;
}

Camera::Real cross2(const Camera::Vector2& a, const Camera::Vector2& b) {
  return -a.y() * b.x() + a.x() * b.y();
}

Camera::Vector3 midpoint(
    const Camera::Ray& a,
    const Camera::Ray& b,
    const bool forceInFront) {
  // this is the mid-point method

  // find ta and tb that minimizes the distance between
  // a(ta) = pa + ta * va and b(tb) = pb + tb * vb:

  // then return the mid-point between a(ta) and b(tb)

  // d(a - b)^2/dta = 0 &&
  // d(a - b)^2/dtb = 0 <=>
  // dot( va, 2 * (a(ta) - b(tb))) = 0 &&
  // dot(-vb, 2 * (a(ta) - b(tb))) = 0 <=>
  // dot(va, a(ta) - b(tb)) = 0 &&
  // dot(vb, a(ta) - b(tb)) = 0 <=>
  // dot(va, pa) + ta * dot(va, va) - dot(va, pb) - tb * dot(va, vb) = 0 &&
  // dot(vb, pa) + ta * dot(vb, va) - dot(vb, pb) - tb * dot(vb, vb) = 0 <=>

  // reformulate as vectors
  //    fa * ta - fb * tb + fc = (0, 0), where
  //    m = rows(va, vb)
  //    fa = m * va
  //    fb = m * vb
  //    fc = m * (pa - pb)
  // -det(fa, fb) * ta + det(fb, fc) = 0 &&
  // -det(fa, fb) * tb + det(fa, fc) = 0 <=>
  // ta = det(fb, fc) / det(fa, fb) &&
  // tb = det(fa, fc) / det(fa, fb)
  Eigen::Matrix<Camera::Real, 2, 3> m;
  m.row(0) = a.direction();
  m.row(1) = b.direction();
  Camera::Vector2 fa = m * a.direction();
  Camera::Vector2 fb = m * b.direction();
  Camera::Vector2 fc = m * (a.origin() - b.origin());
  Camera::Real det = cross2(fa, fb);
  Camera::Real ta = cross2(fb, fc) / det;
  Camera::Real tb = cross2(fa, fc) / det;

  // check for parallel lines
  if (!std::isfinite(ta) || !std::isfinite(tb)) {
    ta = tb = Camera::kNearInfinity;
  }

  // check whether intersection is behind camera
  if (forceInFront && (ta < 0 || tb < 0)) {
    ta = tb = Camera::kNearInfinity;
  }

  Camera::Vector3 pa = a.pointAt(ta);
  Camera::Vector3 pb = b.pointAt(tb);
  return (pa + pb) / 2;
}

#ifdef WIN32

Camera::Rig Camera::loadRig(const std::string& filename) {
  boost::property_tree::ptree tree;
  boost::property_tree::json_parser::read_json(std::ifstream(filename), tree);

  Camera::Rig rig;
  for (const auto& camera : tree.get_child("cameras")) {
    rig.emplace_back(camera.second);
  }
  return rig;
}

#else // WIN32

std::vector<Camera> Camera::loadRig(const std::string& filename) {
  std::string json;
  folly::readFile(filename.c_str(), json);
  CHECK(!json.empty()) << "could not read JSON file: " << filename;
  dynamic dynamic = folly::parseJson(json);

  std::vector<Camera> cameras;
  for (const auto& camera : dynamic["cameras"]) {
    cameras.emplace_back(camera);
  }
  return cameras;
}

#endif // WIN32

#ifndef SUPPRESS_RIG_IO

void Camera::saveRig(
    const std::string& filename,
    const std::vector<Camera>& cameras) {
  dynamic dynamic = dynamic::object(
    "cameras", dynamic::array());
  for (const auto& camera : cameras) {
    dynamic["cameras"].push_back(camera.serialize());
  }
  folly::writeFile(folly::toPrettyJson(dynamic), filename.c_str());
}

// takes a camera and a scale factor. returns a camera model equivalent to
// changing the resolution of the original camera
Camera Camera::createRescaledCamera(
    const Camera& cam,
    const float scale) {

  Camera scaledCam(cam);
  scaledCam.resolution = Camera::Vector2(
    int(cam.resolution.x() * scale),
    int(cam.resolution.y() * scale));

  const float scaleX = scaledCam.resolution.x() / cam.resolution.x();
  const float scaleY = scaledCam.resolution.y() / cam.resolution.y();
  scaledCam.principal.x() *= scaleX;
  scaledCam.principal.y() *= scaleY;
  scaledCam.focal.x() *= scaleX;
  scaledCam.focal.y() *= scaleY;
  return scaledCam;
}

void Camera::unitTest() {
  dynamic serialized = dynamic::object
      ("version", 1)
      ("type", "FTHETA")
      ("origin", dynamic::array(
        -10.51814,
        13.00734,
        -4.22656))
      ("forward", dynamic::array(
        -0.6096207796429852,
        0.7538922995778138,
        -0.24496715221587234))
      ("up", dynamic::array(
        0.7686134846014325,
        0.6376793279268061,
        0.050974366338976666))
      ("right", dynamic::array(
        0.19502945167097138,
        -0.15702371237098722,
        -0.9681462011153862))
      ("resolution", dynamic::array(2448, 2048))
      ("focal", dynamic::array(1240, -1240))
      ("id", "cam9");

  Camera camera(serialized);
  CHECK_EQ(camera.id, "cam9");
  CHECK_EQ(camera.position, Camera::Vector3(-10.51814, 13.00734, -4.22656));
  // use isApprox() because camera orthogonalizes the rotation
  Camera::Vector3 right(
    0.19502945167097138,
    -0.15702371237098722,
    -0.9681462011153862);
  CHECK(camera.right().isApprox(right, 1e-3)) << camera.right();

  auto center = camera.pixel(camera.position + camera.forward());
  CHECK_NEAR(2448 / 2, center.x(), 1e-10);
  CHECK_NEAR(2048 / 2, center.y(), 1e-10);

  // check fov
  CHECK(camera.isDefaultFov());
  CHECK(camera.sees(camera.rigNearInfinity({ 1, 1 })));
  camera.setFov(0.9 * M_PI);
  CHECK_NEAR(camera.getFov(), 0.9 * M_PI, 1e-10);
  camera.setFov(0.1 * M_PI);
  CHECK_NEAR(camera.getFov(), 0.1 * M_PI, 1e-10);
  CHECK(!camera.sees(camera.rigNearInfinity({ 1, 1 })));
  CHECK(camera.sees(camera.rigNearInfinity({ 1200, 1000 })));
  camera.setDefaultFov();
  CHECK(camera.sees(camera.rigNearInfinity({ 1, 1 })));

  {
    // check that rig undoes pixel
    auto d = 3.1;
    auto expected = camera.position + d * Camera::Vector3(-2, 3, -1).normalized();
    auto actual = camera.rig(camera.pixel(expected)).pointAt(d);
    CHECK(expected.isApprox(actual)) << actual << " " << expected;

    // check that this survives getting/setting parameters
    Camera modified = camera;
    modified.setRotation(camera.getRotation());
    auto modifiedActual = modified.rig(modified.pixel(expected)).pointAt(d);
    CHECK(expected.isApprox(modifiedActual))
      << expected << "\n\n" << modifiedActual;
    CHECK(modified.getRotation().isApprox(camera.getRotation()))
      << modified.getRotation() << "\n\n" << camera.getRotation();
  }

  {
    // check that undistort undoes no-op distort
    Real expected = 3;
    Real distorted = camera.distort(expected);
    Real undistorted = camera.undistort(distorted);
    CHECK_NEAR(expected, undistorted, 1.0 / kNearInfinity);
  }

  {
    // check that undistort undoes distort
    camera.distortion[0] = 0.20;
    camera.distortion[1] = 0.02;
    Real expected = 3;
    Real distorted = camera.distort(expected);
    Real undistorted = camera.undistort(distorted);
    CHECK_NEAR(expected, undistorted, 1.0 / kNearInfinity);
  }

  // lines intersect at (1, 2, 3)
  Camera::Ray a(Camera::Vector3(11, 12, -17), Camera::Vector3(-1, -1, 2));
  Camera::Ray b(Camera::Vector3(-8, -4, 0), Camera::Vector3(3, 2, 1));

  auto ab = midpoint(a, b, false);
  CHECK(ab.isApprox(Camera::Vector3(1, 2, 3))) << ab;

  // lines do not intersect, but are at their closest near (1, 1, 1)
  Camera::Ray c(Camera::Vector3(2, 2, 2), Camera::Vector3(-1, -1, 0));
  Camera::Ray d(Camera::Vector3(0, 2, 0), Camera::Vector3(1, -1, 0));

  auto cd = midpoint(c, d, false);
  CHECK(cd.isApprox(Camera::Vector3(1, 1, 1))) << cd;

  // lines are parallel
  Camera::Ray e(Camera::Vector3(2, 2, 2), Camera::Vector3(1, 2, 3));
  Camera::Ray f(Camera::Vector3(1, 2, 3), Camera::Vector3(-1, -2, -3));

  auto ef = midpoint(e, f, false);
  CHECK(ef.isApprox(Camera::Vector3(1.5, 2, 2.5))) << ef;

  {
    Camera::Ray a(Camera::Vector3(11, 12, -17), Camera::Vector3(-1, -1, 2));
    Camera::Ray b(Camera::Vector3(-7, 5, -7), Camera::Vector3(0, 0, 0));
    b.direction() = (a.pointAt(10) - b.origin()) / 10;

    Camera::Vector3 i = midpoint(a, b, false);
    CHECK(i.isApprox(a.pointAt(10))) << i;

    Camera::Vector3 ortho = a.direction().cross(b.direction());
    a.origin() += ortho;
    b.origin() -= ortho;
    CHECK(midpoint(a, b, false).isApprox(i)) << midpoint(a, b, false);
  }
}

#endif // SUPPRESS_RIG_IO

} // namespace surround360
