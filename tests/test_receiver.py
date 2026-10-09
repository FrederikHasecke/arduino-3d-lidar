"""Tests for the serial packet parser and point-cloud conversion (no hardware or OpenGL needed)."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python-receiver-code"))

from offline_visualization import MAX_RANGE_MM, convert_to_cartesian, drop_invalid_points  # noqa: E402
from receive_data import (  # noqa: E402
    END_MARKER,
    START_MARKER,
    SYNC_PAYLOAD,
    FrameAccumulator,
    SerialPacketReader,
    decode_packet,
)


def data_packet(readings: list[tuple[int, int]]) -> bytes:
    """Encode eight (angle_deg, distance_mm) pairs the same way the firmware does."""
    payload = b"".join(
        angle.to_bytes(2, "big") + distance.to_bytes(2, "big") for angle, distance in readings
    )
    return START_MARKER + payload + END_MARKER


SYNC_PACKET = START_MARKER + SYNC_PAYLOAD + END_MARKER


class FakeSerial:
    """Serves a fixed byte stream in small chunks to exercise marker resynchronisation."""

    def __init__(self, data: bytes, chunk_size: int = 5) -> None:
        self._data = data
        self._chunk_size = chunk_size
        self.in_waiting = 0

    def read(self, size: int) -> bytes:
        chunk, self._data = self._data[: self._chunk_size], self._data[self._chunk_size :]
        return chunk


def test_decode_packet_returns_distance_angle_channel():
    readings = [(10 + i, 500 + 100 * i) for i in range(8)]
    packet = decode_packet(data_packet(readings))

    assert not packet.is_sync
    assert packet.points.tolist() == [[d, a, i] for i, (a, d) in enumerate(readings)]


def test_decode_packet_recognises_sync():
    assert decode_packet(SYNC_PACKET).is_sync


def test_decode_packet_rejects_bad_markers():
    with pytest.raises(ValueError):
        decode_packet(b"\x00" * 36)


@pytest.mark.parametrize("chunk_size", [1, 2, 5, 36, 100])
def test_reader_skips_garbage_and_split_markers(chunk_size):
    first = data_packet([(1, 1000)] * 8)
    second = data_packet([(359, 2999)] * 8)
    # Leading noise, a lone start-marker byte and a truncated packet must all be skipped.
    stream = b"\x00\x13\xfe" + START_MARKER + b"\x01\x02" + first + SYNC_PACKET + second
    reader = SerialPacketReader(FakeSerial(stream, chunk_size))

    packets = [reader.read_packet() for _ in range(3)]

    assert [p.is_sync for p in packets] == [False, True, False]
    assert packets[0].points[:, 0].tolist() == [1000] * 8
    assert packets[2].points[:, 1].tolist() == [359] * 8


def test_accumulator_emits_one_frame_per_rotation():
    accumulator = FrameAccumulator()
    assert accumulator.add_packet(decode_packet(SYNC_PACKET)) is None

    for _ in range(3):
        assert accumulator.add_packet(decode_packet(data_packet([(0, 100)] * 8))) is None

    frame = accumulator.add_packet(decode_packet(SYNC_PACKET))
    assert frame.shape == (24, 3)
    assert accumulator.flush() is None


def test_drop_invalid_points_removes_zero_and_clamped_readings():
    raw = np.array([[0, 0, 0], [1500, 10, 1], [MAX_RANGE_MM, 20, 2]], dtype=np.int32)
    assert drop_invalid_points(raw).tolist() == [[1500, 10, 1]]


def test_convert_to_cartesian_preserves_range():
    raw = np.array([[1000, angle, channel] for angle in (0, 90, 270) for channel in range(8)])
    xyz = convert_to_cartesian(raw)

    assert xyz.shape == (len(raw), 3)
    np.testing.assert_allclose(np.linalg.norm(xyz, axis=1), 1.0, rtol=1e-5)


def test_bundled_sample_loads():
    sample_dir = Path(__file__).resolve().parents[1] / "data" / "sequences" / "01" / "lidar_points"
    frames = sorted(sample_dir.glob("*.npy"))
    assert frames

    raw = np.load(frames[0])
    assert raw.ndim == 2 and raw.shape[1] == 3
    assert convert_to_cartesian(drop_invalid_points(raw)).shape[1] == 3
