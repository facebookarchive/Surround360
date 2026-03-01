CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

PROJECT(Surround360Render CXX)
set(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(OpenCV)
FIND_PACKAGE(Ceres REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/util)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/optical_flow)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/camera_isp)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/calibration)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/render)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(/usr/local/include/eigen3)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # required for clang, including AppleClang
  LINK_DIRECTORIES(/usr/local/lib)
endif()

IF (DEFINED HALIDE_DIR)
  INCLUDE_DIRECTORIES(${HALIDE_DIR}/include)
  INCLUDE_DIRECTORIES(${HALIDE_DIR}/src)
ENDIF()

IF(NOT IS_DIRECTORY CMAKE_BUILD_TYPE)
  SET(${CMAKE_BUILD_TYPE} Release ... FORCE)
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -O3 -mmmx -mavx -msse -msse3 -funroll-loops -pipe")
ENDIF()

IF (NOT MSVC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
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
     IF(NOT APPLE AND NOT MSVC)
       TARGET_LINK_LIBRARIES("${name}" PRIVATE Halide gflags dl z tinfo)
     ELSE()
       TARGET_LINK_LIBRARIES("${name}" PRIVATE Halide gflags dl z)
     ENDIF()
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
ENDIF()

IF (DEFINED HALIDE_DIR)
  IF(NOT APPLE AND NOT MSVC)
    TARGET_LINK_LIBRARIES(
       Raw2Rgb
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
       LibVrCamera
       LibJSON
       glog
       gflags
       ${OpenCV_LIBS}
       Halide
       dl
       tinfo
       z)
  ELSE()
      TARGET_LINK_LIBRARIES(
       Raw2Rgb
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
       "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
       LibVrCamera
       LibJSON
       glog
       gflags
       ${OpenCV_LIBS}
       Halide
       dl)
  ENDIF()
ELSE()
  ADD_EXECUTABLE(Raw2Rgb source/camera_isp/Raw2Rgb.cpp)
  TARGET_LINK_LIBRARIES(
     Raw2Rgb
     LibVrCamera
     LibJSON
     glog
     gflags
     dl
     z
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
  folly
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

### Unpacker ###

IF (DEFINED HALIDE_DIR)
  ADD_EXECUTABLE(
    Unpacker
    source/camera_isp/BinaryFootageFile.cpp
    source/camera_isp/Unpacker.cpp
    source/camera_isp/RawConverter.cpp
    "${CMAKE_BINARY_DIR}/CameraIspGen8.h"
    "${CMAKE_BINARY_DIR}/CameraIspGen16.h"
  )

  IF(NOT APPLE AND NOT MSVC)
    TARGET_LINK_LIBRARIES(
      Unpacker
      LibVrCamera
      LibJSON
      glog
      gflags
      ${OpenCV_LIBS}
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      Halide
      dl
      tinfo
      z
      ${OpenCV_LIBS}
      ${PLATFORM_SPECIFIC_LIBS}
    )
  ELSE()
    TARGET_LINK_LIBRARIES(
      Unpacker
      LibVrCamera
      LibJSON
      glog
      gflags
      ${OpenCV_LIBS}
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      Halide
      dl
      z
      ${OpenCV_LIBS}
      ${PLATFORM_SPECIFIC_LIBS}
  )
  ENDIF()
  ADD_DEPENDENCIES(Unpacker CameraIspGen)
ENDIF() # HALIDE_DIR

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
  folly
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
  folly
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

IF (DEFINED HALIDE_DIR)
  ADD_EXECUTABLE(
    TestColorCalibration
    source/test/TestColorCalibration.cpp
    "${CMAKE_BINARY_DIR}/CameraIspGen8.h" "${CMAKE_BINARY_DIR}/CameraIspGen16.h"
  )
  ADD_DEPENDENCIES(TestColorCalibration CameraIspGen)
  TARGET_COMPILE_FEATURES(TestColorCalibration PRIVATE cxx_range_for)

  IF(NOT APPLE AND NOT MSVC)
    TARGET_LINK_LIBRARIES(
      TestColorCalibration
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      LibVrCamera
      LibJSON
      gflags
      glog
      ${OpenCV_LIBS}
      ${CERES_LIBRARIES}
      Halide
      dl
      tinfo
      ${PLATFORM_SPECIFIC_LIBS}
    )
  ELSE()
  TARGET_LINK_LIBRARIES(
    TestColorCalibration
    "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
    LibVrCamera
    LibJSON
    gflags
    glog
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    Halide
    dl
    ${PLATFORM_SPECIFIC_LIBS}
  )
  ENDIF()
ELSE()
  ADD_EXECUTABLE(
    TestColorCalibration
    source/test/TestColorCalibration.cpp)
  TARGET_COMPILE_FEATURES(TestColorCalibration PRIVATE cxx_range_for)
  TARGET_LINK_LIBRARIES(
    TestColorCalibration
    LibVrCamera
    LibJSON
    gflags
    glog
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    ${PLATFORM_SPECIFIC_LIBS}
  )
ENDIF()

### TestVignettingCalibration ###

IF (DEFINED HALIDE_DIR)
  ADD_EXECUTABLE(
    TestVignettingCalibration
    source/test/TestVignettingCalibration.cpp
    "${CMAKE_BINARY_DIR}/CameraIspGen8.h" "${CMAKE_BINARY_DIR}/CameraIspGen16.h"
  )
  ADD_DEPENDENCIES(Raw2Rgb CameraIspGen)
  TARGET_COMPILE_FEATURES(TestVignettingCalibration PRIVATE cxx_range_for)
  IF(NOT APPLE AND NOT MSVC)
    TARGET_LINK_LIBRARIES(
      TestVignettingCalibration
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      LibVrCamera
      LibJSON
      gflags
      glog
      folly
      ${OpenCV_LIBS}
      ${CERES_LIBRARIES}
      Halide
      dl
      tinfo
      ${PLATFORM_SPECIFIC_LIBS}
    )
  ELSE()
    TARGET_LINK_LIBRARIES(
      TestVignettingCalibration
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen8${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGenFast16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/CameraIspGen16${CMAKE_STATIC_LIBRARY_SUFFIX}"
      LibVrCamera
      LibJSON
      gflags
      glog
      folly
      ${OpenCV_LIBS}
      ${CERES_LIBRARIES}
      Halide
      dl
      ${PLATFORM_SPECIFIC_LIBS}
  )
  ENDIF()
ELSE()
  ADD_EXECUTABLE(
    TestVignettingCalibration
    source/test/TestVignettingCalibration.cpp)
  TARGET_COMPILE_FEATURES(TestVignettingCalibration PRIVATE cxx_range_for)
  TARGET_LINK_LIBRARIES(
    TestVignettingCalibration
    LibVrCamera
    LibJSON
    gflags
    glog
    folly
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    ${PLATFORM_SPECIFIC_LIBS}
  )
ENDIF()

### TestVignettingDataAcquisition ###

ADD_EXECUTABLE(
  TestVignettingDataAcquisition
  source/test/TestVignettingDataAcquisition.cpp)
TARGET_COMPILE_FEATURES(TestVignettingDataAcquisition PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  TestVignettingDataAcquisition
  LibVrCamera
  LibJSON
  gflags
  glog
  folly
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${PLATFORM_SPECIFIC_LIBS}
)

### GeoemtricCalibration ###

ADD_EXECUTABLE(
  GeometricCalibration
  source/calibration/GeometricCalibration.cpp
)
TARGET_LINK_LIBRARIES(
  GeometricCalibration
  LibVrCamera
  LibJSON
  gflags
  glog
  folly
  boost_filesystem
  boost_system
  double-conversion
  ${OpenCV_LIBS}
  ${PLATFORM_SPECIFIC_LIBS}
  ${CERES_LIBRARIES}
)
