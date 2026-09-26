"""Format checks using the generated Python protobuf classes."""

import unittest
import struct
import tempfile
from pathlib import Path
from unittest.mock import patch

import decode_recording
from recording_pb2 import RecordingBatch, RecordingHeader
from recording_reader import MAGIC, parse_recording


def _frame(message) -> bytes:
    payload = message.SerializeToString()
    size = len(payload)
    prefix = bytearray()
    while size > 127:
        prefix.append((size & 0x7f) | 0x80)
        size >>= 7
    prefix.append(size)
    return bytes(prefix) + payload


def example_recording() -> bytes:
    header = RecordingHeader(
        format_version=1, sample_rate_hz=100, record_count=2, capacity=2000,
        pid_kp=18.0, pid_kd=0.4, pitch_error_max_rad=0.17,
        duty_cycle_min=35, duty_cycle_max=160,
    )
    batch = RecordingBatch(first_index=0)
    batch.samples.add(
        elapsed_us=0, pitch_rad=0.1, gyro_rad_s=-0.2,
        pid_output=12.5, motor1_encoder_pulses=-123,
        motor2_encoder_pulses=456, control_interval_us=10000,
        motor1_pwm=35, motor2_pwm=36, motor1_dir_pin=True,
        estimates_valid=True, motor1_forward_command=True,
        motor2_forward_command=True,
    )
    batch.samples.add(
        elapsed_us=10000, pitch_rad=0.2, gyro_rad_s=-0.1,
        pid_output=0, motor1_encoder_pulses=-124,
        motor2_encoder_pulses=457, control_interval_us=10001,
        motor1_pwm=0, motor2_pwm=0, mode=1, estimates_valid=True,
    )
    return MAGIC + _frame(header) + _frame(batch)


class RecordingReaderTest(unittest.TestCase):
    def test_legacy_binary_decoder(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "old.bin"
            target = Path(directory) / "old.csv"
            source.write_bytes(struct.pack(
                "<8sHHHHIIffffffBBHIII", b"SBRLOG1", 1, 64, 32, 100,
                1, 2000, 18, 0, 0.4, 0, 0.01, 0.17, 35, 160,
                0, 0, 0, 0,
            ) + struct.pack("<IfffiiIBBBB", 0, 0, 0, 0, 0, 0,
                            10000, 0, 0, 0, 0))
            with patch("sys.argv", ["decode_recording.py", str(source), str(target)]):
                self.assertEqual(decode_recording.main(), 0)
            self.assertIn("elapsed_s", target.read_text())

    def test_settings_snapshot_and_older_header(self):
        old_header, _ = parse_recording(example_recording())
        self.assertFalse(old_header.HasField("settings_at_start"))
        self.assertFalse(old_header.settings_changed_during_recording)

        header = RecordingHeader(format_version=1, sample_rate_hz=100,
                                 record_count=0, capacity=2000,
                                 firmware_revision="abc123")
        settings = header.settings_at_start
        settings.pid_kp = 18.0
        settings.pid_output_min = -1.0
        settings.pwm_frequency_hz = 30000
        settings.wheel_diameter_m = 0.0618
        settings.imu_calibration_valid = False
        data = MAGIC + _frame(header)
        self.assertLessEqual(len(header.SerializeToString()), 1024)
        parsed, _ = parse_recording(data)
        self.assertTrue(parsed.HasField("settings_at_start"))
        self.assertFalse(parsed.settings_changed_during_recording)
        self.assertFalse(parsed.settings_at_start.imu_calibration_valid)
        self.assertEqual(parsed.settings_at_start.imu_gyro_y_offset_rad_s, 0)

        header.settings_changed_during_recording = True
        header.settings_at_start.imu_calibration_valid = True
        header.settings_at_start.imu_gyro_y_offset_rad_s = 0.03
        changed, _ = parse_recording(MAGIC + _frame(header))
        self.assertTrue(changed.settings_changed_during_recording)
        self.assertTrue(changed.settings_at_start.imu_calibration_valid)
        self.assertAlmostEqual(changed.settings_at_start.imu_gyro_y_offset_rad_s, 0.03)

    def test_values(self):
        header, samples = parse_recording(example_recording())
        self.assertEqual((header.sample_rate_hz, header.record_count), (100, 2))
        self.assertEqual(samples[0].motor1_encoder_pulses, -123)
        self.assertEqual(samples[1].elapsed_us, 10000)
        self.assertTrue(samples[0].motor1_forward_command)
        self.assertEqual(samples[1].mode, 1)

    def test_truncated_and_trailing(self):
        recording = example_recording()
        with self.assertRaises(ValueError):
            parse_recording(recording[:-1])
        with self.assertRaises(ValueError):
            parse_recording(recording + b"x")

    def test_version_and_batch_order(self):
        recording = example_recording()
        bad_header = RecordingHeader(format_version=2, sample_rate_hz=100,
                                     record_count=2, capacity=2000)
        first_frame_size = recording[len(MAGIC)]
        first_frame_end = len(MAGIC) + 1 + first_frame_size
        with self.assertRaises(ValueError):
            parse_recording(MAGIC + _frame(bad_header) + recording[first_frame_end:])

        bad_batch = RecordingBatch(first_index=1)
        bad_batch.samples.add(elapsed_us=0)
        with self.assertRaises(ValueError):
            parse_recording(recording[:first_frame_end] + _frame(bad_batch))

    def test_full_capacity(self):
        header = RecordingHeader(format_version=1, sample_rate_hz=100,
                                 record_count=2000, capacity=2000)
        data = bytearray(MAGIC + _frame(header))
        for first in range(0, 2000, 16):
            batch = RecordingBatch(first_index=first)
            for index in range(first, first + 16):
                batch.samples.add(elapsed_us=index * 10000,
                                  control_interval_us=10000,
                                  motor1_encoder_pulses=-index)
            data.extend(_frame(batch))
        parsed_header, samples = parse_recording(bytes(data))
        self.assertEqual(parsed_header.record_count, 2000)
        self.assertEqual(samples[-1].elapsed_us, 19990000)


if __name__ == "__main__":
    unittest.main()
