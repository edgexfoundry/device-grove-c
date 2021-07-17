#!/bin/sh
set -e -x

BUILD_CSDK=$1

CSDK_VER=2.0.0
MRAA_VER=2.2.0

# Dependencies
if [ ! -d deps ]
then
  mkdir deps

  cd /device-grove/deps

  git clone --depth 1 --branch v${MRAA_VER} https://github.com/intel-iot-devkit/mraa.git
  cd mraa

# always install in lib folder
  mkdir -p build && cd build
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

  wget https://github.com/edgexfoundry/device-sdk-c/archive/v${CSDK_VER}.zip
  unzip v${CSDK_VER}.zip
  cd device-sdk-c-${CSDK_VER}
  ./scripts/build.sh
  cp -rf include/* /usr/include/
  cp build/release/c/libcsdk.so /usr/lib/
fi
  rm -rf /device-grove/deps
fi
