# Recording format

`recording.proto` is the shared schema for robot recordings. An `SBRPB1` file
starts with the eight bytes `53 42 52 50 42 31 00 00` (`SBRPB1\0\0`). It is
followed by one varint-length-delimited `RecordingHeader`, then ordered
varint-length-delimited `RecordingBatch` messages until `record_count` samples
have been read. Each batch contains 1–16 samples, and its `first_index` must
equal the number of preceding samples. Files with trailing or missing data are
invalid. No wall-clock time is claimed; sample timestamps are monotonic and
relative to the recording start. Readers limit the header frame to 1024 bytes
and each batch frame to 2048 bytes. The file extension is `.sbrpb`.

The magic and `format_version` identify the framing and semantics. Additive
schema changes may use new field numbers without changing the major file
version. Never reuse a removed field number; mark it `reserved`. Change the
magic and major version if framing or existing field meanings change.

New recordings include `settings_at_start`, captured when the control task
processes Start. Older `SBRPB1` recordings omit it; readers should show
"metadata unavailable" for those and for `SBRLOG1` files. The original
header fields are retained for older protobuf readers. The snapshot holds PID
gains, output limits and integral threshold; pitch setpoint and safety limits;
complementary filter gyro weight; motor speed calibration slopes and intercepts,
direction inversion and coast settings; PWM frequency in Hz and resolution in
bits; wheel diameter in meters and encoder pulses per revolution; and IMU
offsets in radians per second (gyro Y) and radians (accelerometer pitch).
`imu_calibration_valid` is false if calibration had not finished at Start; in
that case the offset numbers are zero placeholders. `firmware_revision` is the
build Git revision, or `unknown` outside a Git checkout.

`settings_changed_during_recording` becomes true when an accepted live tuning
command actually changes a captured tunable value. It stays true if the value
is later restored. It does not track control mode or motor outputs, which are
stored per sample, or the completion of IMU calibration after Start. Thus a
false value means no captured tunable was changed by a command during this
recording; it does not turn the snapshot into a history of settings.

The robot keeps its 32-byte `SBRLOG1` records in RAM and encodes `SBRPB1` only
while downloading. `SBRLOG1` files remain readable by the dashboard and Python
decoder.

The firmware uses nanopb 0.4.9, Python uses generated `recording_pb2.py`, and
the Vite dashboard imports this `.proto` as text for protobuf.js. From the
repository root, run a PlatformIO build once to install nanopb, then regenerate
the checked-in C and Python files after editing the schema:

```sh
cd firmware/platformio_projects/self_balancing_robot
~/.platformio/penv/bin/pio run
cd ../../..
~/.platformio/penv/bin/python firmware/platformio_projects/self_balancing_robot/.pio/libdeps/esp32-s3-devkitc-1/Nanopb/generator/nanopb_generator.py -I proto -D firmware/platformio_projects/self_balancing_robot/src proto/recording.proto
~/.platformio/penv/bin/python -m grpc_tools.protoc -I proto --python_out=software/python proto/recording.proto
```

The dashboard reads the schema directly, so it has no generated file. Other
languages can generate readers from `recording.proto`; their file reader must
also implement the short framing described above.
