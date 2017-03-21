CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

PROJECT(Surround360CameraCtlUI CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0)
pkg_check_modules(GLIBMM glibmm-2.4)
pkg_check_modules(GLIB glib-2.0)
pkg_check_modules(GL gl)
pkg_check_modules(GLEW glew)
pkg_check_modules(GLU glu)

INCLUDE_DIRECTORIES(/usr/include/flycapture)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/source)
INCLUDE_DIRECTORIES(${GTKMM_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GLIBMM_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GLIB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GLU_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${USB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(../surround360_render/source/camera_isp)
INCLUDE_DIRECTORIES(../surround360_render/source/util)
INCLUDE_DIRECTORIES(../surround360_render/source)
INCLUDE_DIRECTORIES(../surround360_render)
INCLUDE_DIRECTORIES(${HALIDE_DIR}/include)
INCLUDE_DIRECTORIES(${HALIDE_DIR}/src)

LINK_DIRECTORIES(${GTKMM_LIBRARY_DIRS})
LINK_DIRECTORIES(${GLIBMM_LIBRARY_DIRS})
LINK_DIRECTORIES(${GLIB_LIBRARY_DIRS})
LINK_DIRECTORIES(${GL_LIBRARY_DIRS})
LINK_DIRECTORIES(${GLEW_LIBRARY_DIRS})
LINK_DIRECTORIES(${GLU_LIBRARY_DIRS})
LINK_DIRECTORIES(${USB_LIBRARY_DIRS})
LINK_DIRECTORIES(../surround360_render)
LINK_DIRECTORIES(../surround360_render/lib)
LINK_DIRECTORIES(${HALIDE_DIR}/lib)

ADD_LIBRARY(CameraIspGen8 STATIC IMPORTED)
ADD_LIBRARY(CameraIspGenFast8 STATIC IMPORTED)
ADD_LIBRARY(CameraIspGen16 STATIC IMPORTED)
ADD_LIBRARY(CameraIspGenFast16 STATIC IMPORTED)

SET_TARGET_PROPERTIES(CameraIspGen8 PROPERTIES IMPORTED_LOCATION ../surround360_render/CameraIspGen8.a)
SET_TARGET_PROPERTIES(CameraIspGenFast8 PROPERTIES IMPORTED_LOCATION ../surround360_render/CameraIspGenFast8.a)
SET_TARGET_PROPERTIES(CameraIspGen16 PROPERTIES IMPORTED_LOCATION ../surround360_render/CameraIspGen16.a)
SET_TARGET_PROPERTIES(CameraIspGenFast16 PROPERTIES IMPORTED_LOCATION ../surround360_render/CameraIspGenFast16.a)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
   SET(${CMAKE_BUILD_TYPE} Release ... FORCE)
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -O3")
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -ggdb -mmmx -msse -msse2 -msse3 -msse4 -pthread -Wno-write-strings")

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Add custom targets to show all source and header files in IDEs ###

FILE(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/source/*")
ADD_CUSTOM_TARGET(_source SOURCES ${SRC})

### CameraControl ###

ADD_EXECUTABLE(
  CameraControlUI
  source/CameraController.cpp
  source/CameraDiscoverButton.cpp
  source/CameraListView.cpp
  source/CameraView.cpp
  source/Config.cpp
  source/MainWindow.cpp
  source/PointGrey.cpp
  source/PreviewButton.cpp
  source/PreviewCamComboBox.cpp
  source/RecordButton.cpp
  source/ShutterComboBox.cpp
  source/GainComboBox.cpp
  source/StillButton.cpp
  source/Shaders.cpp
  source/main.cpp
)

TARGET_COMPILE_FEATURES(CameraControlUI PRIVATE cxx_range_for)
TARGET_LINK_LIBRARIES(
  CameraControlUI
  "-lflycapture${D}"
  "-lpthread"
  "-lgtkmm-3.0"
  "-lglibmm-2.4"
  "-lglib-2.0"
  "-latkmm-1.6"
  "-lsigc-2.0"
  "-lglut"
  "-lglog"
  "-lGL"
  "-lLibVrCamera"
  "-lLibJSON"
  "-lopencv_core"
  "-lopencv_imgproc"
  "-lHalide"
  "-lGLEW"
  CameraIspGen8
  CameraIspGenFast8
  CameraIspGen16
  CameraIspGenFast16
 "-ldl"
)
