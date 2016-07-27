CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

PROJECT(Surround360Render CXX)

FIND_PACKAGE(OpenCV)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/util)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/optical_flow)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/camera_isp)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/calibration)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/render)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
  SET(${CMAKE_BUILD_TYPE} Release ... FORCE)
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -Ofast")
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Dinteger=int")

IF(NOT APPLE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
ENDIF()

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Add custom targets to show all source and header files in Xcode ###

FILE(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/source/*")
ADD_CUSTOM_TARGET( _source SOURCES ${SRC} )

### LibJSON ###

ADD_LIBRARY(
  LibJSON
  source/thirdparty/supereasyjson/json.cpp
)

### LibVrCamera ###

FILE(GLOB calibration_SRC "source/calibration/*.cpp")
FILE(GLOB optical_flow_SRC "source/optical_flow/*.cpp")
FILE(GLOB render_SRC "source/render/*.cpp")
FILE(GLOB util_SRC "source/util/*.cpp")

ADD_LIBRARY(
  LibVrCamera
  source/util/SystemUtil.cpp
  ${calibration_SRC}
  ${optical_flow_SRC}
  ${render_SRC}
  ${util_SRC}
)
TARGET_COMPILE_FEATURES(LibVrCamera PRIVATE cxx_range_for)


### Raw2Rgb ###

ADD_EXECUTABLE(
  Raw2Rgb
  source/camera_isp/Raw2Rgb.cpp
)

TARGET_LINK_LIBRARIES(
  Raw2Rgb
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
)

### TestRenderStereoPanorama ###

ADD_EXECUTABLE(
  TestRenderStereoPanorama
  source/test/TestRenderStereoPanorama.cpp
)
TARGET_COMPILE_FEATURES(TestRenderStereoPanorama PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestRenderStereoPanorama
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestJson ###

ADD_EXECUTABLE(
  TestJson
  source/test/TestJson.cpp
)
TARGET_COMPILE_FEATURES(TestJson PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestJson
  LibVrCamera
  LibJSON
  glog
  gflags
)

### TestIntrinsicCalibration ###

ADD_EXECUTABLE(
  TestIntrinsicCalibration
  source/test/TestIntrinsicCalibration.cpp
)
TARGET_COMPILE_FEATURES(TestIntrinsicCalibration PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestIntrinsicCalibration
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
)

### TestOpticalFlow ###

ADD_EXECUTABLE(
  TestOpticalFlow
  source/test/TestOpticalFlow.cpp
)
TARGET_COMPILE_FEATURES(TestOpticalFlow PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestOpticalFlow
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestImageWarper ###

ADD_EXECUTABLE(
  TestImageWarper
  source/test/TestImageWarper.cpp
)
TARGET_COMPILE_FEATURES(TestImageWarper PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestImageWarper
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestLogger ###

ADD_EXECUTABLE(
  TestLogger
  source/test/TestLogger.cpp
)
TARGET_COMPILE_FEATURES(TestLogger PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestLogger
  LibVrCamera
  glog
  gflags
  ${OpenCV_LIBS}
)

### UnpackImageBundle ###

ADD_EXECUTABLE(
  UnpackImageBundle
  source/camera_isp/UnpackImageBundle.cpp
)
TARGET_COMPILE_FEATURES(UnpackImageBundle PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  UnpackImageBundle
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestRingRectification ###

ADD_EXECUTABLE(
  TestRingRectification
  source/test/TestRingRectification.cpp
)
TARGET_COMPILE_FEATURES(TestRingRectification PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestRingRectification
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestPoleRemoval ###

ADD_EXECUTABLE(
  TestPoleRemoval
  source/test/TestPoleRemoval.cpp
)
TARGET_COMPILE_FEATURES(TestPoleRemoval PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestPoleRemoval
  LibVrCamera
  LibJSON
  gflags
  glog
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestOpticalCenterVisualization ###

ADD_EXECUTABLE(
  TestOpticalCenterVisualization
  source/test/TestOpticalCenterVisualization.cpp
)
TARGET_COMPILE_FEATURES(TestOpticalCenterVisualization PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestOpticalCenterVisualization
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestRenderWithoutSides ###

ADD_EXECUTABLE(
  TestRenderWithoutSides
  source/test/TestRenderWithoutSides.cpp
)
TARGET_COMPILE_FEATURES(TestRenderWithoutSides PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestRenderWithoutSides
  LibVrCamera
  LibJSON
  gflags
  glog
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestExtractAlphaChannel ###

ADD_EXECUTABLE(
  TestExtractAlphaChannel
  source/test/TestExtractAlphaChannel.cpp
)
TARGET_COMPILE_FEATURES(TestExtractAlphaChannel PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestExtractAlphaChannel
  LibVrCamera
  LibJSON
  gflags
  glog
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)
