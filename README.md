# Device Grove Service
[![Build Status](https://jenkins.edgexfoundry.org/view/EdgeX%20Foundry%20Project/job/edgexfoundry/job/device-grove-c/job/main/badge/icon)](https://jenkins.edgexfoundry.org/view/EdgeX%20Foundry%20Project/job/edgexfoundry/job/device-grove-c/job/main/) [![GitHub Latest Dev Tag)](https://img.shields.io/github/v/tag/edgexfoundry/device-grove-c?include_prereleases&sort=semver&label=latest-dev)](https://github.com/edgexfoundry/device-grove-c/tags) ![GitHub Latest Stable Tag)](https://img.shields.io/github/v/tag/edgexfoundry/device-grove-c?sort=semver&label=latest-stable) [![GitHub License](https://img.shields.io/github/license/edgexfoundry/device-grove-c)](https://choosealicense.com/licenses/apache-2.0/) [![GitHub Pull Requests](https://img.shields.io/github/issues-pr-raw/edgexfoundry/device-grove-c)](https://github.com/edgexfoundry/device-grove-c/pulls) [![GitHub Contributors](https://img.shields.io/github/contributors/edgexfoundry/device-grove-c)](https://github.com/edgexfoundry/device-grove-c/contributors) [![GitHub Committers](https://img.shields.io/badge/team-committers-green)](https://github.com/orgs/edgexfoundry/teams/device-grove-c-committers/members) [![GitHub Commit Activity](https://img.shields.io/github/commit-activity/m/edgexfoundry/device-grove-c)](https://github.com/edgexfoundry/device-grove-c/commits)

## Status

This device service has not been released in EdgeX since 1.3.x "Hanoi". Although the codebase has been updated for EdgeX v2, it is not supported by EdgeX Foundry and as such this repository is archived.

## About
The EdgeX Device Grove Service is developed to control/communicate Grove sensors connected on Grove PI in an EdgeX deployment

## Supported Boards:
Raspberry PI 3, 4 - ARM64 bit

## Dependencies:

The Device Grove service based on [device-c-sdk](https://github.com/edgexfoundry/device-sdk-c)
is developed using libmraa - a low level library that communicates with the Raspberry PI board.
The repository can be found on git at [libmraa](https://github.com/intel-iot-devkit/mraa). 

## Build Instruction:

1. Check out device-grove-c available at [device-grove-c](https://github.com/edgexfoundry/device-grove-c)

2. Build a docker image by using the following command
```
sh> cd device-grove-c
sh> make docker

```
This command shall build the dependencies - libmraa and device-c-sdk library and create the release version of the docker image by the name 'device-grove-c'.
By default, the configuration and profile file used by the service are available in __'res'__ folder.

## Configuration for docker image
1. Port number specified in the configuration.toml
2. --device=/dev/ < i2c-device > to map host device to the container. For Raspberry PI, it is i2c-1.

**Note:** On Raspberry PI, make sure that i2c_arm=on is set. This enables i2c-1 device, required for communication between Grove PI & Raspberry PI boards.

### Building natively

Before building natively the dependencies for the C SDK need to be installed: these are listed at https://github.com/edgexfoundry/device-sdk-c

The `build_deps.sh` script downloads and builds the required libmraa and the EdgeX device SDK. The `build.sh` script is then used to build the device service.

```
sh> cd device-grove-c
sh> ./scripts/build_deps.sh
sh> ./scripts/build.sh
```
