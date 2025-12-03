# UPS HAT Controller

Standalone C++ application for monitoring [Waveshare UPS HAT (E)](https://www.waveshare.com/wiki/UPS_HAT_(E)) and performing safe system shutdown on power loss or low battery voltage.

![UPS HAT (E)](https://www.waveshare.com/w/upload/thumb/7/73/UPS-HAT-E-1.jpg/600px-UPS-HAT-E-1.jpg)

## Overview

This project provides a lightweight, systemd-integrated solution for monitoring the Waveshare UPS HAT (E) expansion board on Raspberry Pi systems. It communicates with the UPS HAT via I2C and automatically shuts down the system when power is lost or battery voltage drops below safe thresholds.

For detailed hardware specifications and register documentation, refer to:
- [UPS HAT (E) Product Page](https://www.waveshare.com/wiki/UPS_HAT_(E))
- [UPS HAT (E) Register Manual](https://www.waveshare.com/wiki/UPS_HAT_(E)_Register)

## Features

- I2C communication with Waveshare UPS HAT (E) (I2C address: 0x2D)
- Real-time monitoring of battery status, charging state, and cell voltages
- Automatic shutdown on power loss (configurable delay)
- Low voltage protection with configurable thresholds
- Thermal monitoring (CPU temperature and cooling state)
- Comprehensive logging via syslog/journald
- Systemd service integration
- No ROS2 dependencies - pure C++ standalone application

## Requirements

- Raspberry Pi with I2C enabled
- Waveshare UPS HAT (E) hardware
- CMake 3.10 or higher
- C++17 compatible compiler (GCC/Clang)
- Linux system with systemd

## Building

```bash
git clone <repository-url>
cd ups-hat-controller
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
```

## Configuration

Configuration can be provided via:

1. **Configuration file**: `/etc/ups-hat-controller/ups-hat-controller.conf`
2. **Environment variables**: Set in `/etc/ups-hat-controller/ups-hat-controller.env` or systemd service
3. **Command line**: `--config <path>` option

### Configuration Parameters

- `I2C_BUS`: I2C bus number (default: 1)
- `I2C_ADDR`: I2C device address in decimal (default: 45 = 0x2d)
- `PUBLISH_RATE_HZ`: Monitoring rate in Hz (default: 1.0)
- `SHUTDOWN_DELAY_SEC`: Delay before shutdown on power loss in seconds (default: 60)
- `LOW_VOLTAGE_THRESHOLD`: Low voltage threshold in millivolts (default: 3150)
- `LOW_VOLTAGE_THRESHOLD_COUNT`: Number of consecutive low voltage detections before shutdown (default: 30)
- `ENABLE_SYSLOG`: Enable syslog logging (default: true)

## Installation

After building:

```bash
sudo make install
```

This will install:
- Executable: `/usr/local/bin/ups_hat_controller`
- Systemd service: `/etc/systemd/system/ups-hat-controller.service`
- Config file: `/etc/ups-hat-controller/ups-hat-controller.conf`

## Systemd Service

Enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable ups-hat-controller.service
sudo systemctl start ups-hat-controller.service
```

Check status:

```bash
sudo systemctl status ups-hat-controller.service
```

View logs:

```bash
sudo journalctl -u ups-hat-controller.service -f
```

## Permissions

The service needs access to I2C devices. Ensure the user running the service (or root) has access to `/dev/i2c-*` devices. You may need to add the user to the `i2c` group:

```bash
sudo usermod -aG i2c <username>
```

## Logging

Logs are written to systemd journal (journald) via syslog. View logs using:

```bash
sudo journalctl -u ups-hat-controller.service -f
```

The application logs status changes only (power transitions, charge state changes) to reduce log verbosity.

## Shutdown Behavior

The controller will initiate system shutdown when:

1. **Power loss**: Mains power is lost and not restored within `SHUTDOWN_DELAY_SEC` seconds
2. **Low voltage**: Any cell voltage drops below `LOW_VOLTAGE_THRESHOLD` mV while battery current is below 50 mA, detected for `LOW_VOLTAGE_THRESHOLD_COUNT` consecutive cycles

Shutdown is performed by:
1. Sending shutdown command to UPS HAT (register 0x01 = 0x55)
2. Executing `systemctl poweroff`

## Charge States

The application monitors charge states according to the [UPS HAT (E) Register documentation](https://www.waveshare.com/wiki/UPS_HAT_(E)_Register):

- 0: Standby
- 1: Trickle Charge
- 2: Constant Current Charge
- 3: Constant Voltage Charge
- 4: Charging Pending
- 5: Full State
- 6: Charge Timeout

## Troubleshooting

### I2C device not found

Check I2C device availability:
```bash
ls -l /dev/i2c-*
i2cdetect -y 1
```

The UPS HAT should appear at address 0x2D.

### Permission denied

Ensure user is in `i2c` group or run as root:
```bash
groups
sudo usermod -aG i2c $USER
```

### Service fails to start

Check service logs:
```bash
sudo journalctl -u ups-hat-controller.service -n 50
```

Check configuration:
```bash
sudo /usr/local/bin/ups_hat_controller --help
```

### Enable I2C on Raspberry Pi

If I2C is not enabled:
```bash
sudo raspi-config
# Navigate to: Interfacing Options -> I2C -> Enable
sudo reboot
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## References

- [Waveshare UPS HAT (E) Product Page](https://www.waveshare.com/wiki/UPS_HAT_(E))
- [Waveshare UPS HAT (E) Register Manual](https://www.waveshare.com/wiki/UPS_HAT_(E)_Register)
