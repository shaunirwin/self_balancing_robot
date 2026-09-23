# README

This is the README for the Self-Balancing Robot.

## Building the firmware

From the repository root, change to the PlatformIO project directory and run the build:

```bash
cd electronics/code/platformio_projects/self_balancing_robot
~/.platformio/penv/bin/pio run
```

To clean the previous build artifacts and rebuild the firmware:

```bash
~/.platformio/penv/bin/pio run --target clean
~/.platformio/penv/bin/pio run
```

If `pio` is already available on your `PATH`, you can use the shorter command:

```bash
pio run
```

## Programming the ESP32 board

Plug the USB cable into the **right** USB slot on the ESP32-S3 WROOM Freenove board to program it and to send/receive serial communication, as shown in (this)[https://www.youtube.com/watch?v=VZDCkARFCPk&ab_channel=Freenove] video.

## To receive serial communication

In the terminal:

```bash
tio /dev/ttyACM0
```
Check that this is the correct comm port (see `platformio.ini`).

`Ctrl+t q` to quit.

Run the script to read and siplay data from the ESP32:

```bash
cd src
g++ -std=c++2a -o read_serial -I ../../../esp_idf_projects/hello_world/main/include read_serial.cpp
./read_serial
```
