CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

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
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

IF (DEFINED HALIDE_DIR)
  INCLUDE_DIRECTORIES(${HALIDE_DIR}/include)
  INCLUDE_DIRECTORIES(${HALIDE_DIR}/src)
ENDIF()

IF(NOT IS_DIRECTORY CMAKE_BUILD_TYPE)
  SET(${CMAKE_BUILD_TYPE} Release ... FORCE)
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -Ofast")
ENDIF()

IF (NOT MSVC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Dinteger=int")
ENDIF()

IF(NOT APPLE AND NOT MSVC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
ENDIF()

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_OSX_ARCHITECTURES "${ARCHS_STANDARD}")

### Add custom targets to show all source and header files in Xcode ###

FILE(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/source/*")
ADD_CUSTOM_TARGET( _source SOURCES ${SRC} )

### Halide support
IF (DEFINED HALIDE_DIR)
  INCLUDE("${HALIDE_DIR}/../HalideGenerator.cmake")

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_HALIDE")

  FUNCTION(halide_gen_app name folder)
     ADD_EXECUTABLE("${name}" ${ARGN})
     TARGET_LINK_LIBRARIES("${name}" PRIVATE Halide gflags)
     SET_TARGET_PROPERTIES("${name}" PROPERTIES FOLDER "${folder}")
     IF (MSVC)
        SET_TARGET_PROPERTIES("${name}" PROPERTIES LINK_FLAGS "/STACK:8388608,1048576")
        TARGET_COMPILE_DEFINITIONS("${name}" PRIVATE _CRT_SECURE_NO_WARNINGS)
        TARGET_LINK_LIBRARIES("${name}" PRIVATE Kernel32)
     ENDIF()
  ENDFUNCTION(halide_gen_app)

  LINK_DIRECTORIES(${HALIDE_DIR}/lib)
ENDIF()


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
IF (DEFINED HALIDE_DIR)
   halide_gen_app(CameraIspGen "apps" source/camera_isp/CameraIspGen.cpp)

   ADD_CUSTOM_COMMAND(OUTPUT "${CMAKE_BINARY_DIR}/CameraIspGen8.h" "${CMAKE_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
                      COMMAND CameraIspGen --output_bpp 8
                      DEPENDS CameraIspGen
                      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
                      COMMENT "Generating 8 bit Halide CameraIsp header and object file")

   ADD_CUSTOM_COMMAND(OUTPUT "${CMAKE_BINARY_DIR}/CameraIspGen16.h" "${CMAKE_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
                      COMMAND CameraIspGen --output_bpp 16
                      DEPENDS CameraIspGen
                      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
                      COMMENT "Generating 16 bit Halide CameraIsp header and object file")

  ADD_EXECUTABLE(Raw2Rgb source/camera_isp/Raw2Rgb.cpp "${CMAKE_BINARY_DIR}/CameraIspGen8.h" "${CMAKE_BINARY_DIR}/CameraIspGen16.h")
  ADD_DEPENDENCIES(Raw2Rgb CameraIspGen)

  TARGET_LINK_LIBRARIES(
     Raw2Rgb
     "${CMAKE_CURRENT_BINARY_DIR}/cameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
     "${CMAKE_CURRENT_BINARY_DIR}/cameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
     "${CMAKE_CURRENT_BINARY_DIR}/cameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
     "${CMAKE_CURRENT_BINARY_DIR}/cameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
     LibVrCamera
     LibJSON
     glog
     gflags
     ${OpenCV_LIBS}
     Halide
  )
ELSE()
  ADD_EXECUTABLE(Raw2Rgb source/camera_isp/Raw2Rgb.cpp)
  TARGET_LINK_LIBRARIES(
     Raw2Rgb
     LibVrCamera
     LibJSON
     glog
     gflags
     ${OpenCV_LIBS}
  )
ENDIF()

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

### TestHyperPreview ###

ADD_EXECUTABLE(
  TestHyperPreview
  source/test/TestHyperPreview.cpp
)
TARGET_COMPILE_FEATURES(TestHyperPreview PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestHyperPreview
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

### TestColorCalibration ###

ADD_EXECUTABLE(
  TestColorCalibration
  source/test/TestColorCalibration.cpp
)
TARGET_COMPILE_FEATURES(TestColorCalibration PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestColorCalibration
  LibVrCamera
  LibJSON
  gflags
  glog
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
)
