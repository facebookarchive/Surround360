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

* Install OpenCV:
```
  cd ~
  git clone https://github.com/Itseez/opencv.git
  cd opencv
  git checkout tags/3.1.0
  cmake -DWITH_IPP=OFF
  make
  sudo make install
```

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

* (if using accelerated ISP) Install LLVM
```
  cd ~
  svn co https://llvm.org/svn/llvm-project/llvm/branches/release_37 llvm3.7
  svn co https://llvm.org/svn/llvm-project/cfe/branches/release_37 llvm3.7/tools/clang
  cd llvm3.7
  mkdir build
  cd build
  cmake -DLLVM_ENABLE_TERMINFO=OFF -DLLVM_TARGETS_TO_BUILD="X86;ARM;NVPTX;AArch64;Mips;PowerPC" -DLLVM_ENABLE_ASSERTIONS=ON -DCMAKE_BUILD_TYPE=Release ..
  make
  export LLVM_CONFIG=$HOME/llvm3.7/build/bin/llvm-config
  export CLANG=$HOME/llvm3.7/build/bin/clang
```

* (to use accelerated ISP) Install Halide
```
  cd ~
  git clone https://github.com/halide/Halide.git
  cd Halide
  mkdir cmake_build
  cd cmake_build
  export LLVM_ROOT=$HOME/llvm3.7/build
  cmake -DLLVM_BIN=${LLVM_ROOT}/bin -DLLVM_INCLUDE="${LLVM_ROOT}/../include;${LLVM_ROOT}/include" -DLLVM_LIB=${LLVM_ROOT}/lib -DLLVM_VERSION=37 ..
  make
```

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
