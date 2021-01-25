# Device Grove Service

## About
The EdgeX Device Grove Service is developed to control/communicate Grove sensors connected on Grove PI in an EdgeX deployment

## Supported Boards:
Raspberry PI 3+ - ARM64 bit

## Dependencies:

The Device Grove service based on [device-c-sdk](https://github.com/edgexfoundry/device-sdk-c)
is developed using libmraa - a low level library that communicates with the Raspberry PI board.
The repository can be found on git at [libmraa](https://github.com/intel-iot-devkit/mraa). 

## Build Instruction:

1. Check out device-grove-c available at [device-grove-c](https://github.com/edgexfoundry/device-grove-c)

2. Build a docker image by using the following command
```
sh> cd device-grove-c
sh> make version 
sh> docker build . -t device-grove-c -f ./scripts/Dockerfile.alpine

```
This command shall build the dependencies - libmraa and device-c-sdk library and create the release version of the docker image by the name 'device-grove-c'. This name can be replaced with your preferred name, if necessary.
By default, the configuration and profile file used by the service are available in __'res'__ folder.

## Configuration for docker image
1. Port number specified in the configuration.toml
2. --device=/dev/ < i2c-device > to map host device to the container. For Raspberry PI, it is i2c-1.

**Note:** On Raspberry PI, make sure that i2c_arm=on is set. This enables i2c-1 device, required for communication between Grove PI & Raspberry PI boards.
