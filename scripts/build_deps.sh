#!/bin/sh
set -e -x

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

# get c-sdk from edgexfoundry
  cd /device-grove/deps
  wget https://github.com/edgexfoundry/device-sdk-c/archive/0.7.0.tar.gz
  tar -xzf 0.7.0.tar.gz
  cd device-sdk-c-0.7.0
  ./scripts/build.sh
  cp -rf include/* /usr/include/
  cp build/release/c/libcsdk.so /usr/lib/

  rm -rf /device-grove/deps
fi
