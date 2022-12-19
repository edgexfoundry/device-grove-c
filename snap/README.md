# EdgeX Grove Device Service Snap
[![snap store badge](https://raw.githubusercontent.com/snapcore/snap-store-badges/master/EN/%5BEN%5D-snap-store-black-uneditable.png)](https://snapcraft.io/edgex-device-grove)

This folder contains snap packaging for the EdgeX Grove Device Service Snap

The snap currently supports `arm64` platform.

## Installation

### Installing snapd
The snap can be installed on any system that supports snaps. 
You can see how to install snaps on your system [here](https://snapcraft.io/docs/installing-snapd/6735).

However for full security confinement, the snap should be installed on an
Ubuntu 18.04 LTS or later (Desktop or Server), or a system running Ubuntu Core 18 or later.

### Installing EdgeX Device Grove as a snap
The snap is published in the snap store as [edgex-device-grove](https://snapcraft.io/edgex-device-grove).
You can see the current revisions available for your machine's architecture by running the command:
```bash
snap info edgex-device-grove
```

The snap can be installed using:
```bash
sudo snap install edgex-device-grove
```

## Snap configuration


Snapd doesn't support orchestration between services in different snaps. 
It is therefore possible on a reboot for a device service to come up faster than all of the required services running in the main edgexfoundry snap. 
If this happens, 
the device service may repeatedly fail startup, 
and if it exceeds the systemd default limits, 
then it might be left in a failed state. 
This situation might be more likely on constrained hardware (e.g. RPi).

The default configuration file is in `/var/snap/edgex-device-grove/current/config/edgex-device-grove/res`. 

This device service is started by default. 
Changes to the configuration files require a restart to take effect:
```bash
sudo snap restart edgex-device-grove
```
