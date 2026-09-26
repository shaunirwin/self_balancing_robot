# README

This is the README for the Self-Balancing Robot.

## Building the firmware and serial reader

From the repository root, change to the PlatformIO project directory:

```bash
cd firmware/platformio_projects/self_balancing_robot
```

The firmware includes `src/secrets.h` for Wi-Fi credentials. On a fresh
checkout, create it from the tracked template:

```bash
cp src/secrets_template.h src/secrets.h
```

Edit `src/secrets.h` and replace `REPLACE_WITH_YOUR_SSID` and
`REPLACE_WITH_YOUR_PASSWORD` with your network name and password. The project
ignores `secrets.h` so credentials stay out of commits. Keep the template's
variable names unchanged; `src/main.cpp` uses them to connect to Wi-Fi.

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

## HTTP control and telemetry

The robot connects to the configured Wi-Fi network and serves HTTP on port 80. The examples below use the current robot address:

```bash
ROBOT=http://192.168.178.55
```

The root endpoint is a simple connectivity check:

```bash
curl "$ROBOT/"
```

Read the live controller state and safety flags:

```bash
curl -s "$ROBOT/status" | python3 -m json.tool
```

Important fields include `CONTROL_MODE`, `ESTIMATES_VALID`, `AUTO_ARM_ALLOWED`, `pitch_angle_current`, and the current PWM limits.

### Updating controller values

`/set-value` is a `POST` endpoint requiring `key` and `value` form fields. Changes are queued for the control task. Angle values are in radians; PWM values are integers from 0 to 255.

Examples:

```bash
# Select or disarm the controller mode
curl -X POST --data-urlencode 'key=CONTROL_MODE' \
  --data-urlencode 'value=MANUAL' "$ROBOT/set-value"

curl -X POST --data-urlencode 'key=CONTROL_MODE' \
  --data-urlencode 'value=AUTO' "$ROBOT/set-value"

# PID parameters and setpoint
curl -X POST --data-urlencode 'key=PID_Kp' \
  --data-urlencode 'value=18' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=PID_Ki' \
  --data-urlencode 'value=0' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=PID_Kd' \
  --data-urlencode 'value=0.4' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=PID_setpoint' \
  --data-urlencode 'value=0' "$ROBOT/set-value"

# Motor limits and pitch safety thresholds
curl -X POST --data-urlencode 'key=MOTOR_DUTY_CYCLE_MIN' \
  --data-urlencode 'value=35' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=MOTOR_DUTY_CYCLE_MAX' \
  --data-urlencode 'value=160' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=PITCH_ANGLE_ERROR_MAX' \
  --data-urlencode 'value=0.174533' "$ROBOT/set-value"  # 10 degrees
curl -X POST --data-urlencode 'key=PITCH_ANGLE_ERROR_MIN' \
  --data-urlencode 'value=0.003491' "$ROBOT/set-value"  # 0.2 degrees
```

Manual motor commands are available for bench testing with the wheels raised:

```bash
curl -X POST --data-urlencode 'key=MOTOR_1_DIR_MANUAL' \
  --data-urlencode 'value=FORWARD' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=MOTOR_2_DIR_MANUAL' \
  --data-urlencode 'value=FORWARD' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=MOTOR_1_DUTY_CYCLE_MANUAL' \
  --data-urlencode 'value=35' "$ROBOT/set-value"
curl -X POST --data-urlencode 'key=MOTOR_2_DUTY_CYCLE_MANUAL' \
  --data-urlencode 'value=35' "$ROBOT/set-value"
```

Emergency stop:

```bash
curl -X POST --data-urlencode 'key=EMERGENCY_STOP' \
  --data-urlencode 'value=1' "$ROBOT/set-value"
```

The network emergency stop is a software request and is not a substitute for a physical motor-power cutoff.

### Recording telemetry in RAM

The firmware stores up to 20 seconds of 100 Hz telemetry in a fixed 64 KB buffer. Each recording contains pitch, gyro rate, PID output, encoder positions, loop timing, PWM, directions, mode, and estimate-validity flags.

Start a recording while the robot is still in MANUAL:

```bash
curl -X POST "$ROBOT/recording/start"
```

Check its progress:

```bash
curl -s "$ROBOT/recording/status" | python3 -m json.tool
```

Stop it explicitly:

```bash
curl -X POST "$ROBOT/recording/stop"
```

The recording also stops when the buffer fills or AUTO disarms. Once the status reports `"state": "READY"`, download and decode it:

```bash
curl -o balance-recording.bin "$ROBOT/recording/download"
python3 ../../../software/python/decode_recording.py balance-recording.bin
```

The decoder writes `balance-recording.csv`, which can be plotted or inspected with standard tools. Only one completed recording is retained; starting a new recording overwrites the previous one. Recordings are held in volatile RAM and are lost if the ESP32 resets or loses power.

The old `/logs` endpoint is deprecated and returns an error; use the `/recording/*` endpoints instead.

For compatibility, `START_LOGGING` and `STOP_LOGGING` are also accepted as `/set-value` keys, but the dedicated recording endpoints are preferred.
