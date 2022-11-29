#!/bin/sh
set -e -x

CSDK_VER=2.2.0
MRAA_VER=2.2.0
MRAA_PATCHES=aaa0a5cd4e401bde4fb3691dd4e6c70a5c61e031

# Dependencies
if [ ! -d deps ]
then
  mkdir -p deps/usr
  cd deps

  git clone https://github.com/intel-iot-devkit/mraa.git
  cd mraa
  git checkout v${MRAA_VER}
  git cherry-pick -n ${MRAA_PATCHES}
  mkdir build
  cd build

# always install in lib folder
  cmake -DBUILDSWIG=OFF -DCMAKE_INSTALL_PREFIX=../../usr -DCMAKE_INSTALL_LIBDIR=lib ..
  make && make install
  cd ../..

# get c-sdk from edgexfoundry and build

  wget https://github.com/edgexfoundry/device-sdk-c/archive/v${CSDK_VER}.zip
  unzip v${CSDK_VER}.zip
  cd device-sdk-c-${CSDK_VER}
  ./scripts/build.sh
  cd ..
  tar -z -x -C usr --strip-components=1 -f device-sdk-c-${CSDK_VER}/build/release/csdk-${CSDK_VER}.tar.gz
fi
