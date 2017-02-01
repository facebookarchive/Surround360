# Surround 360 Rendering Software

Surround 360 is a hardware and software system for capturing and rendering 3d (stereo) 360 videos and photos, suitable for viewing in VR. This subdirectory of the project is specific to rendering software.

## Requirements

* The Surround 360 rendering software requires either Linux or Mac OS X

* The essential dependencies of the Surround 360 rendering software are:
  * CMake
  * gflags
  * glog
  * OpenCV 3.0+

* Additional optional functionality depends on:
  * ffmpeg
  * Gooey
  * wx

## Installing Dependencies of the Surround 360 Rendering Software

* After building all of the dependencies and the rendering code itself, there are no further installation steps.

* The build system for the Surround 360 code is CMake.

* This software has been tested on Ubuntu 14.04/16.04 LTS (CMake 3.2.2) and OS X 10.11.5 (using CMake 3.5.1).

* If you are building on OS X, it may be more convenient to use brew to install some dependencies.

* Install CMake (method 1):
  see https://cmake.org

* Install CMake (method 2 - Linux only)
```
  sudo apt-get install software-properties-common
  sudo add-apt-repository ppa:george-edison55/cmake-3.x
  sudo apt-get update
  sudo apt-get install cmake && sudo apt-get upgrade cmake
```

* Install homebrew (on OS X):
  http://brew.sh/, or just run this directly:
```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

* Git is used to download and install packages from source (e.g. OpenCV below). Git may come preinstalled (check using git --version). It is not required if using brew in OS X.

* Install Git (method 1 - Linux only):
```
  sudo apt-get install git
```

* Install Subversion (method 1 - Linux only):
```
  sudo apt install subversion
```

* Install Subversion (method 1 - OS X only):
```
  brew install subversion
```

* Install Git (method 2 - OS X only):
```
  brew install git
```

* Install Python (method 1 - Linux only):
```
  sudo apt-get install python
```

* Install Python (method 2 - OS X only):
```
  brew install python
```

* Install gflags (method 1 - Linux only):
```
  sudo apt-get install libgflags2v5 libgflags-dev
```

* Install gflags (method 2 - OS X only):
```
  brew install gflags
```

* Install glog (method 1 - Linux only):
```
  sudo apt-get install libgoogle-glog-dev
```

* Install glog (method 2 - OS X only):
```
  brew install glog
```

* Install folly (method 1):
  see https://github.com/facebook/folly

* Install folly (method 2 - OS X only):
<pre>
  brew install folly
</pre>

* Install Ceres (method 1 - Linux only)
  see http://ceres-solver.org/installation.html
```
  sudo apt-get install libatlas-base-dev
  sudo apt-get install libeigen3-dev
  cd ~
  git clone https://ceres-solver.googlesource.com/ceres-solver
  cd ceres-solver
  mkdir ceres-bin
  cd ceres-bin
  cmake ..
  make -j3
  sudo make install
  sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
```

* Install Ceres (method 2 - OSX only)
```
  brew install --build-from-source ceres-solver
```

* Install OpenCV:
```
  cd ~
  git clone https://github.com/Itseez/opencv.git
  cd opencv
  cmake -DWITH_IPP=OFF
  make
  sudo make install
```

* Install COLMAP
  see https://colmap.github.io/install.html

* Install ffmpeg (method 1):
  see https://trac.ffmpeg.org/wiki/CompilationGuide

* Install ffmpeg (method 2 - OS X only):
```
  brew install ffmpeg
```

* Install Gooey (method 1):
  see https://github.com/chriskiehl/Gooey

* Install Gooey (method 2 - Linux only):
```
  sudo apt-get install python-pip
  sudo pip install --upgrade pip
  sudo pip install Gooey
  sudo apt-get install python-wxgtk2.8
```

If python-wxgtk2.8 not available (e.g. Ubuntu 16.04):
```
  echo "deb http://archive.ubuntu.com/ubuntu wily main universe" | sudo tee /etc/apt/sources.list.d/wily-copies.list
  sudo apt update
  sudo apt install python-wxgtk2.8
  sudo rm /etc/apt/sources.list.d/wily-copies.list
  sudo apt update
```

* Install Gooey (method 3 - OS X only):
```
  pip install --upgrade pip
  sudo pip install Gooey
  brew install wxpython
  brew install wxmac
  brew link wxmac
```

* Install PIL (method 1 - Linux only)
```
  sudo apt-get install python-pil
```

* Install PIL (method 2 - OSX only)
```
  pip install pillow
```

* (to use accelerated ISP) Install Halide
```
  cd ~
  git clone https://github.com/halide/Halide.git
```
  see README file inside Halide directory for installation instructions, including LLVM dependency. If the make step throws errors about unused variables, add the option ```-DWARNINGS_AS_ERRORS=OFF``` to the cmake command

## Compiling the Surround 360 Rendering Software

* After installing all of the dependencies as described above, run:
```
  cd <install_path>/surround360/surround360_render
  cmake -DCMAKE_BUILD_TYPE=Release
  make
```

  (to use accelerated ISP):
```
  cd <install_path>/surround360/surround360_render
  cmake -DCMAKE_BUILD_TYPE=Release -DHALIDE_DIR=$HOME/Halide/cmake_build
  make
```

* To test that compilation is successful, run:
```
  ./bin/TestRenderStereoPanorama --help
```

* We recommend configuring CMake to compile in Release mode because the code will execute faster. However, you can also set it up for debug mode with:
```
  cmake -DCMAKE_BUILD_TYPE=Debug
```

* Sometimes it is useful to compile with XCode instead of CMake (e.g., to use its profiling and debugging tools). To do so:
```
  cd <install_path>/surround360/surround360_render
  mkdir XCodeDebug
  cd XCodeDebug
  cmake -DCMAKE_BUILD_TYPE=Debug -G Xcode ..
```

## How the Surround 360 Rendering Software Works

Check out our blog post about how rendering for Surround 360 works here:
https://code.facebook.com/posts/265413023819735


## Join the Surround 360 community

* Website: https://facebook360.fb.com/facebook-surround-360/

See the CONTRIBUTING file for how to help out.

## License

The Surround 360 rendering code is BSD-licensed, as it appears in LICENSE_render.md under /surround360_render. We also provide an additional patent grant.
