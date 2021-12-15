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

The latest beta version of the snap can be installed using:
```bash

sudo snap install edgex-device-grove --beta
```

The latest development version of the snap can be installed using:
```bash
sudo snap install edgex-device-grove --edge
```

## Snap configuration

Device services implement a service dependency check on startup which ensures that 
all of the runtime dependencies of a particular service are met before the service transitions to an active state.

Snapd doesn't support orchestration between services in different snaps. 
It is therefore possible on a reboot for a device service to come up faster than all of the required services running in the main edgexfoundry snap. 
If this happens, 
the device service may repeatedly fail startup, 
and if it exceeds the systemd default limits, 
then it might be left in a failed state. 
This situation might be more likely on constrained hardware (e.g. RPi).

This snap therefore implements a basic retry loop with a maximum `startup-duration` and `startup-interval`. 
If the dependent services are not available, 
the service sleeps for the defined interval and then tries again up to a maximum duration. 
EdgeX services wait for dependencies (e.g. core-data) to become available and will exit after reaching the maximum duration if the dependencies aren't met.

This environment variable sets the total duration in seconds allowed for the services to complete the bootstrap start-up. 
Default is 60 seconds. 
Change the maximum duration:

```bash
sudo snap set edgex-device-grove startup-duration=120
```

This environment variable sets the retry interval in seconds for the services retrying a failed action during the bootstrap start-up. 
Default is 1 second. 
Change the interval between retries:

```bash
sudo snap set edgex-device-grove startup-interval=3
```

The service can then be started as follows. 
The `--enable` option ensures that as well as starting the service now, 
it will be automatically started on boot:
```bash
sudo snap start --enable edgex-device-grove.device-grove
```

### Using a content interface to set device configuration

The `device-config` content interface allows another snap to seed this snap with configuration directories under `$SNAP_DATA/config/device-grove`.

Note that the `device-config` content interface does NOT support seeding of the Secret Store Token because that file is expected at a different path.

Please refer to [edgex-config-provider](https://github.com/canonical/edgex-config-provider), for an example and further instructions.
                                        
### Autostart
By default, the edgex-device-grove disables its service on install, 
as the expectation is that the default profile configuration files will be customized, 
and thus this behavior allows the profile `configuration.toml` files in [`$SNAP_DATA`](https://snapcraft.io/docs/environment-variables) 
to be modified before the service is first started.

This behavior can be overridden by setting the `autostart` configuration setting to `true`. 
This is useful when configuration and/or device profiles are being provided via configuration or gadget snap content interface.

**Note** - this option is typically set from a gadget snap.

### Rich Configuration
While it's possible on Ubuntu Core to provide additional profiles via gadget 
snap content interface, quite often only minor changes to existing profiles are required. 

These changes can be accomplished via support for EdgeX environment variable 
configuration overrides via the snap's configure hook.
If the service has already been started, setting one of these overrides currently requires the
service to be restarted via the command-line or snapd's REST API. 
If the overrides are provided via the snap configuration defaults capability of a gadget snap, 
the overrides will be picked up when the services are first started.

The following syntax is used to specify service-specific configuration overrides:

```
env.<stanza>.<config option>
```
For instance, to setup an override of the service's Port use:
```bash
sudo snap set edgex-device-grove env.service.port=2112
```
And restart the service:
```bash
sudo snap restart edgex-device-grove.device-grove
```

**Note** - at this time changes to configuration values in the [Writable] section are not supported.
For details on the mapping of configuration options to Config options, please refer to "Service Environment Configuration Overrides".

## Service Environment Configuration Overrides
**Note** - all of the configuration options below must be specified with the prefix: 'env.'
```
[Service]
service.health-check-interval   // Service.HealthCheckInterval
service.host                    // Service.Host
service.server-bind-addr        // Service.ServerBindAddr
service.port                    // Service.Port
service.max-result-count        // Service.MaxResultCount
service.max-request-size        // Service.MaxRequestSize
service.startup-msg             // Service.StartupMsg
service.request-timeout         // Service.RequestTimeout

[SecretStore]
secret-store.secrets-file               // SecretStore.SecretsFile
secret-store.disable-scrub-secrets-file // SecretStore.DisableScrubSecretsFile

[Clients.core-data]
clients.core-data.port          // Clients.core-data.Port

[Clients.core-metadata]
clients.core-metadata.port      // Clients.core-metadata.Port

[Device]
device.update-last-connected    // Device.UpdateLastConnected
device.use-message-bus          // Device.UseMessageBus
```
