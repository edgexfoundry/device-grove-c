#!/bin/sh
set -e -x

BUILD_CSDK=$1

# Dependencies
if [ ! -d deps ]
then
  mkdir deps

  cd /device-grove/deps

  git clone https://github.com/intel-iot-devkit/mraa.git
  cd mraa
  # This version contain fix to identify raspberry-pi 3
  git reset --hard d320776

# patch for raspberryPI
  patch -p1 < /device-grove/scripts/rpi_patch
  mkdir -p build && cd build

# always install in lib folder
  cmake -DBUILDSWIG=OFF -DCMAKE_INSTALL_LIBDIR=lib ../.
  make && make install

# get c-sdk from edgexfoundry and build
if [ "$BUILD_CSDK" = "1" ]
then
  cd /device-grove/deps

  git clone --depth 1 --branch v0.7.0 https://github.com/PJK/libcbor
  sed -e 's/-flto//' -i libcbor/CMakeLists.txt
  cmake -DCMAKE_BUILD_TYPE=Release -DCBOR_CUSTOM_ALLOC=ON libcbor
  make
  make install

  wget https://github.com/edgexfoundry/device-sdk-c/archive/v1.2.2.zip
  unzip v1.2.2.zip
  cd device-sdk-c-1.2.2
  ./scripts/build.sh
  cp -rf include/* /usr/include/
  cp build/release/c/libcsdk.so /usr/lib/
fi
  rm -rf /device-grove/deps
fi
