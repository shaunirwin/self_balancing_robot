"""Read an SBRPB1 recording using the shared protobuf schema."""

from pathlib import Path

from recording_pb2 import RecordingBatch, RecordingHeader, RecordingSample


MAGIC = b"SBRPB1\0\0"
MAX_HEADER_BYTES = 1024
MAX_BATCH_BYTES = 2048


def _frame(data: bytes, offset: int, max_size: int) -> tuple[bytes, int]:
    size = 0
    shift = 0
    for _ in range(5):
        if offset >= len(data):
            raise ValueError("truncated protobuf frame length")
        byte = data[offset]
        offset += 1
        size |= (byte & 0x7f) << shift
        if byte < 0x80:
            if size == 0 or size > max_size:
                raise ValueError(f"invalid protobuf frame length: {size}")
            end = offset + size
            if end > len(data):
                raise ValueError("truncated protobuf frame")
            return data[offset:end], end
        shift += 7
    raise ValueError("protobuf frame length is too long")


def parse_recording(data: bytes) -> tuple[RecordingHeader, list[RecordingSample]]:
    if not data.startswith(MAGIC):
        raise ValueError("unexpected recording magic")
    header_data, offset = _frame(data, len(MAGIC), MAX_HEADER_BYTES)
    header = RecordingHeader.FromString(header_data)
    if header.format_version != 1:
        raise ValueError(f"unsupported recording version: {header.format_version}")
    if not 0 < header.sample_rate_hz <= 10000 or header.record_count > header.capacity:
        raise ValueError("invalid recording metadata")

    samples = []
    while len(samples) < header.record_count:
        batch_data, offset = _frame(data, offset, MAX_BATCH_BYTES)
        batch = RecordingBatch.FromString(batch_data)
        if batch.first_index != len(samples) or not 1 <= len(batch.samples) <= 16:
            raise ValueError("invalid recording batch sequence")
        if len(samples) + len(batch.samples) > header.record_count:
            raise ValueError("recording has too many samples")
        samples.extend(batch.samples)
    if offset != len(data):
        raise ValueError("recording has trailing data")
    return header, samples


def read_recording(path: str | Path) -> tuple[RecordingHeader, list[RecordingSample]]:
    return parse_recording(Path(path).read_bytes())
