# README

This is the README for the Self-Balancing Robot.

## Building the firmware and serial reader

From the repository root, change to the PlatformIO project directory:

```bash
cd electronics/code/platformio_projects/self_balancing_robot
```

Build both the robot firmware and the PC serial reader:

```bash
make
```

The serial reader is written to `.pio/read_serial`.

To build only one component:

```bash
make firmware
make read_serial
```

The firmware can also be built directly with PlatformIO:

```bash
~/.platformio/penv/bin/pio run
```

## Programming the ESP32 board

Plug the USB cable into the **right** USB slot on the ESP32-S3 WROOM Freenove board to program it and to send/receive serial communication, as shown in [this video](https://www.youtube.com/watch?v=VZDCkARFCPk&ab_channel=Freenove).

From the PlatformIO project directory, build and upload the firmware with:

```bash
make upload
```

This uses `/dev/ttyACM0`, as configured in `platformio.ini`.

The equivalent direct PlatformIO command is:

```bash
~/.platformio/penv/bin/pio run --target upload
```

To open PlatformIO's serial monitor after uploading:

```bash
~/.platformio/penv/bin/pio device monitor
```

Press `Ctrl+C` to exit the monitor.

## To receive serial communication

In the terminal:

```bash
tio /dev/ttyACM0
```
Check that this is the correct comm port (see `platformio.ini`).

`Ctrl+t q` to quit.

Build and run the application that reads and displays data from the ESP32:

```bash
make run-reader
```

Close `tio` or PlatformIO's serial monitor first because only one application can use `/dev/ttyACM0` at a time. Press `Ctrl+C` to stop the reader.
