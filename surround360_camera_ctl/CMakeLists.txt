cmake_minimum_required(VERSION 3.1)
project(Surround360CameraCtl CXX)
include_directories(/usr/include/flycapture)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/camera_control)

IF( NOT DEFINED CMAKE_BUILD_TYPE )
   SET( ${CMAKE_BUILD_TYPE} Release ... FORCE )
ENDIF()

IF ( NOT ${CMAKE_BUILD_TYPE} MATCHES  Debug )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -Ofast" )
ENDIF ()

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wno-write-strings" )

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Add custom targets to show all source and header files in IDEs ###

file( GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/source/*" )
add_custom_target( _source SOURCES ${SRC} )

### CameraControl ###

add_executable(
  CameraControl
  source/camera_control/PointGrey.cpp
  source/camera_control/CameraControl.cpp
)

target_compile_features(CameraControl PRIVATE cxx_range_for)
target_link_libraries(
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
add_executable(
  pc_test
  source/camera_control/pc_test.cpp
)

target_link_libraries(
  pc_test
)
