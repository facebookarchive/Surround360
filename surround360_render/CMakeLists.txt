cmake_minimum_required(VERSION 3.1)
project(Surround360Render CXX)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/util)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/optical_flow)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/camera_isp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/calibration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/render)

IF( NOT DEFINED CMAKE_BUILD_TYPE )
   SET( ${CMAKE_BUILD_TYPE} Release ... FORCE )
ENDIF()

IF ( NOT ${CMAKE_BUILD_TYPE} MATCHES  Debug )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -Ofast" )
ENDIF ()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Dinteger=int")

IF(NOT APPLE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
ENDIF()

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Add custom targets to show all source and header files in Xcode ###

file(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/source/*")
add_custom_target( _source SOURCES ${SRC} )

### LibJSON ###

add_library(
  LibJSON
  source/thirdparty/supereasyjson/json.cpp
)

### LibVrCamera ###

file(GLOB calibration_SRC "source/calibration/*.cpp")
file(GLOB optical_flow_SRC "source/optical_flow/*.cpp")
file(GLOB render_SRC "source/render/*.cpp")
file(GLOB util_SRC "source/util/*.cpp")

add_library(
  LibVrCamera
  source/util/SystemUtil.cpp
  ${calibration_SRC}
  ${optical_flow_SRC}
  ${render_SRC}
  ${util_SRC}
)
target_compile_features(LibVrCamera PRIVATE cxx_range_for)


### Raw2Rgb ###

add_executable(
  Raw2Rgb
  source/camera_isp/Raw2Rgb.cpp
)

target_link_libraries(
  Raw2Rgb
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
)

### TestRenderStereoPanorama ###

add_executable(
  TestRenderStereoPanorama
  source/test/TestRenderStereoPanorama.cpp
)
target_compile_features(TestRenderStereoPanorama PRIVATE cxx_range_for)
target_link_libraries(
  TestRenderStereoPanorama
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestJson ###

add_executable(
  TestJson
  source/test/TestJson.cpp
)
target_compile_features(TestJson PRIVATE cxx_range_for)
target_link_libraries(
  TestJson
  LibVrCamera
  LibJSON
  glog
  gflags
)

### TestIntrinsicCalibration ###

add_executable(
  TestIntrinsicCalibration
  source/test/TestIntrinsicCalibration.cpp
)
target_compile_features(TestIntrinsicCalibration PRIVATE cxx_range_for)
target_link_libraries(
  TestIntrinsicCalibration
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
)

### TestOpticalFlow ###

add_executable(
  TestOpticalFlow
  source/test/TestOpticalFlow.cpp
)
target_compile_features(TestOpticalFlow PRIVATE cxx_range_for)
target_link_libraries(
  TestOpticalFlow
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestImageWarper ###

add_executable(
  TestImageWarper
  source/test/TestImageWarper.cpp
)
target_compile_features(TestImageWarper PRIVATE cxx_range_for)
target_link_libraries(
  TestImageWarper
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestLogger ###

add_executable(
  TestLogger
  source/test/TestLogger.cpp
)
target_compile_features(TestLogger PRIVATE cxx_range_for)
target_link_libraries(
  TestLogger
  LibVrCamera
  glog
  gflags
  ${OpenCV_LIBS}
)

### UnpackImageBundle ###

add_executable(
  UnpackImageBundle
  source/camera_isp/UnpackImageBundle.cpp
)
target_compile_features(UnpackImageBundle PRIVATE cxx_range_for)
target_link_libraries(
  UnpackImageBundle
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestRingRectification ###

add_executable(
  TestRingRectification
  source/test/TestRingRectification.cpp
)
target_compile_features(TestRingRectification PRIVATE cxx_range_for)
target_link_libraries(
  TestRingRectification
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestPoleRemoval ###

add_executable(
  TestPoleRemoval
  source/test/TestPoleRemoval.cpp
)
target_compile_features(TestPoleRemoval PRIVATE cxx_range_for)
target_link_libraries(
  TestPoleRemoval
  LibVrCamera
  LibJSON
  gflags
  glog
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestOpticalCenterVisualization ###

add_executable(
  TestOpticalCenterVisualization
  source/test/TestOpticalCenterVisualization.cpp
)
target_compile_features(TestOpticalCenterVisualization PRIVATE cxx_range_for)
target_link_libraries(
  TestOpticalCenterVisualization
  LibVrCamera
  LibJSON
  glog
  gflags
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestRenderWithoutSides ###

add_executable(
  TestRenderWithoutSides
  source/test/TestRenderWithoutSides.cpp
)
target_compile_features(TestRenderWithoutSides PRIVATE cxx_range_for)
target_link_libraries(
  TestRenderWithoutSides
  LibVrCamera
  LibJSON
  gflags
  glog
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)

### TestExtractAlphaChannel ###

add_executable(
  TestExtractAlphaChannel
  source/test/TestExtractAlphaChannel.cpp
)
target_compile_features(TestExtractAlphaChannel PRIVATE cxx_range_for)
target_link_libraries(
  TestExtractAlphaChannel
  LibVrCamera
  LibJSON
  gflags
  glog
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)
