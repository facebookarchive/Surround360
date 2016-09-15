CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

PROJECT(Surround360CameraCtl CXX)

INCLUDE_DIRECTORIES(/usr/include/flycapture)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source/camera_control)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
   SET(${CMAKE_BUILD_TYPE} Release ... FORCE)
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -Ofast")
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wno-write-strings")

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Add custom targets to show all source and header files in IDEs ###

FILE(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/source/*")
ADD_CUSTOM_TARGET(_source SOURCES ${SRC})

### CameraControl ###

ADD_EXECUTABLE(
  CameraControl
  source/camera_control/PointGrey.cpp
  source/camera_control/CameraControl.cpp
)

TARGET_COMPILE_FEATURES(CameraControl PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  CameraControl
  "-lflycapture${D}"
  "-lusb-1.0"
  "-lpthread"
  "-lgflags"
  "-lswscale"
  "-lavutil"
  "-lavcodec"
  "-lavformat"
  "-lavfilter"
)

### Producer consumer test ###
ADD_EXECUTABLE(
  pc_test
  source/camera_control/pc_test.cpp
)

TARGET_LINK_LIBRARIES(
  pc_test
)
