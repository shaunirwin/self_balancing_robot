#!/usr/bin/env python3
"""Convert an SBRPB1 or legacy SBRLOG1 control recording to CSV."""

import argparse
import csv
import math
from pathlib import Path
import struct
import sys

from google.protobuf.message import DecodeError
from recording_reader import MAGIC as PROTO_MAGIC, parse_recording


HEADER = struct.Struct("<8sHHHHIIffffffBBHIII")
RECORD = struct.Struct("<IfffiiIBBBB")
MODE_NAMES = {0: "AUTO", 1: "MANUAL", 2: "FUNCTION"}


def decode_protobuf(data: bytes, output_path: Path) -> int:
    header, samples = parse_recording(data)
    fieldnames = [
        "elapsed_s", "pitch_rad", "pitch_deg", "gyro_rad_s",
        "gyro_deg_s", "pid_output", "motor1_encoder_pulses",
        "motor2_encoder_pulses", "control_interval_us", "motor1_pwm",
        "motor2_pwm", "motor1_dir_pin", "motor2_dir_pin",
        "motor1_forward_command", "motor2_forward_command", "mode",
        "estimates_valid",
    ]
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow({
                "elapsed_s": f"{sample.elapsed_us / 1_000_000.0:.6f}",
                "pitch_rad": f"{sample.pitch_rad:.8f}",
                "pitch_deg": f"{math.degrees(sample.pitch_rad):.5f}",
                "gyro_rad_s": f"{sample.gyro_rad_s:.8f}",
                "gyro_deg_s": f"{math.degrees(sample.gyro_rad_s):.5f}",
                "pid_output": f"{sample.pid_output:.8f}",
                "motor1_encoder_pulses": sample.motor1_encoder_pulses,
                "motor2_encoder_pulses": sample.motor2_encoder_pulses,
                "control_interval_us": sample.control_interval_us,
                "motor1_pwm": sample.motor1_pwm,
                "motor2_pwm": sample.motor2_pwm,
                "motor1_dir_pin": int(sample.motor1_dir_pin),
                "motor2_dir_pin": int(sample.motor2_dir_pin),
                "motor1_forward_command": int(sample.motor1_forward_command),
                "motor2_forward_command": int(sample.motor2_forward_command),
                "mode": MODE_NAMES.get(sample.mode, f"UNKNOWN_{sample.mode}"),
                "estimates_valid": int(sample.estimates_valid),
            })
    print(f"Wrote {len(samples)} records ({len(samples) / header.sample_rate_hz:.2f} s) to {output_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="downloaded .bin recording")
    parser.add_argument("output", nargs="?", type=Path,
                        help="output CSV path (default: input name with .csv)")
    args = parser.parse_args()

    output_path = args.output or args.input.with_suffix(".csv")
    data = args.input.read_bytes()
    if data.startswith(PROTO_MAGIC):
        return decode_protobuf(data, output_path)
    if len(data) < HEADER.size:
        raise ValueError("file is too short to contain an SBRLOG1 header")

    (
        magic, version, header_size, record_size, sample_rate_hz,
        record_count, capacity, kp, ki, kd, setpoint_rad,
        pitch_error_min_rad, pitch_error_max_rad, duty_min, duty_max,
        header_flags, _reserved0, _reserved1, _reserved2,
    ) = HEADER.unpack_from(data)

    if magic.rstrip(b"\0") != b"SBRLOG1":
        raise ValueError(f"unexpected file magic: {magic!r}")
    if version != 1:
        raise ValueError(f"unsupported recording version: {version}")
    if header_size != HEADER.size or record_size != RECORD.size:
        raise ValueError(
            f"unsupported layout: header={header_size}, record={record_size}")

    expected_size = header_size + record_count * record_size
    if len(data) < expected_size:
        raise ValueError(
            f"truncated recording: expected {expected_size} bytes, got {len(data)}")

    fieldnames = [
        "elapsed_s", "pitch_rad", "pitch_deg", "gyro_rad_s",
        "gyro_deg_s", "pid_output", "motor1_encoder_pulses",
        "motor2_encoder_pulses", "control_interval_us", "motor1_pwm",
        "motor2_pwm", "motor1_dir_pin", "motor2_dir_pin",
        "motor1_forward_command", "motor2_forward_command", "mode",
        "estimates_valid",
    ]

    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for index in range(record_count):
            offset = header_size + index * record_size
            (
                elapsed_us, pitch_rad, gyro_rad_s, pid_output,
                motor1_encoder_pulses, motor2_encoder_pulses,
                control_interval_us, motor1_pwm, motor2_pwm, flags, _reserved,
            ) = RECORD.unpack_from(data, offset)

            mode_value = (flags >> 2) & 0x03
            writer.writerow({
                "elapsed_s": f"{elapsed_us / 1_000_000.0:.6f}",
                "pitch_rad": f"{pitch_rad:.8f}",
                "pitch_deg": f"{math.degrees(pitch_rad):.5f}",
                "gyro_rad_s": f"{gyro_rad_s:.8f}",
                "gyro_deg_s": f"{math.degrees(gyro_rad_s):.5f}",
                "pid_output": f"{pid_output:.8f}",
                "motor1_encoder_pulses": motor1_encoder_pulses,
                "motor2_encoder_pulses": motor2_encoder_pulses,
                "control_interval_us": control_interval_us,
                "motor1_pwm": motor1_pwm,
                "motor2_pwm": motor2_pwm,
                "motor1_dir_pin": (flags >> 0) & 1,
                "motor2_dir_pin": (flags >> 1) & 1,
                "motor1_forward_command": (flags >> 5) & 1,
                "motor2_forward_command": (flags >> 6) & 1,
                "mode": MODE_NAMES.get(mode_value, f"UNKNOWN_{mode_value}"),
                "estimates_valid": (flags >> 4) & 1,
            })

    print(
        f"Wrote {record_count} records ({record_count / sample_rate_hz:.2f} s) "
        f"to {output_path}\n"
        f"PID: Kp={kp:g}, Ki={ki:g}, Kd={kd:g}; "
        f"setpoint={math.degrees(setpoint_rad):.3f} deg\n"
        f"Limits: PWM {duty_min}..{duty_max}; pitch error "
        f"{math.degrees(pitch_error_min_rad):.3f}.."
        f"{math.degrees(pitch_error_max_rad):.3f} deg; "
        f"capacity={capacity} records"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, struct.error, DecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
