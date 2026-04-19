"""Capture LiDAR frames from the Arduino/ESP32 serial stream and save them as `.npy`."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from threading import Event

import numpy as np


START_MARKER = b"\xFE\xFD"
END_MARKER = b"\xFC\xFB"
SYNC_PAYLOAD = b"\xFF" * 32
PACKET_SIZE = 36
PAYLOAD_SIZE = 32
CHANNEL_COUNT = 8
DEFAULT_BAUD = 115200


@dataclass(slots=True)
class DecodedPacket:
    is_sync: bool
    points: np.ndarray | None = None


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_output_dir() -> Path:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return repo_root() / "data" / "sequences" / timestamp / "lidar_points"


def resolve_output_dir(path_arg: Path | None) -> Path:
    if path_arg is None:
        return default_output_dir()

    candidate = path_arg.expanduser()
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    return candidate.resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive LiDAR packets over serial and save each rotation as a .npy frame."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/ttyUSB0.")
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate. Defaults to {DEFAULT_BAUD}.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory to save timestamped .npy frames into.",
    )
    parser.add_argument(
        "--frame-limit",
        type=int,
        help="Optional number of completed rotations to capture before exiting.",
    )
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=0.25,
        help="Serial read timeout in seconds. Defaults to 0.25.",
    )
    parser.add_argument(
        "--save-partial-on-exit",
        action="store_true",
        help="Save the current incomplete frame when interrupted.",
    )
    return parser.parse_args()


def import_serial():
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required for serial capture. Install the project dependencies first."
        ) from exc

    return serial, list_ports


def available_ports_text() -> str:
    _, list_ports = import_serial()
    ports = sorted(list_ports.comports(), key=lambda port: port.device)
    if not ports:
        return "No serial ports detected."

    return "\n".join(f"- {port.device}: {port.description}" for port in ports)


def open_serial_connection(port: str, baud: int, timeout: float):
    serial, _ = import_serial()
    try:
        return serial.Serial(port=port, baudrate=baud, timeout=timeout)
    except Exception as exc:
        raise RuntimeError(
            f"Failed to open serial port {port!r}. Available ports:\n{available_ports_text()}"
        ) from exc


def decode_packet(packet: bytes) -> DecodedPacket:
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"Expected a {PACKET_SIZE}-byte packet, got {len(packet)} bytes.")
    if packet[:2] != START_MARKER or packet[-2:] != END_MARKER:
        raise ValueError("Packet markers are invalid.")

    payload = packet[2:-2]
    if payload == SYNC_PAYLOAD:
        return DecodedPacket(is_sync=True)

    points = np.empty((CHANNEL_COUNT, 3), dtype=np.int32)
    for channel_idx in range(CHANNEL_COUNT):
        offset = channel_idx * 4
        angle_deg = (payload[offset] << 8) | payload[offset + 1]
        distance_mm = (payload[offset + 2] << 8) | payload[offset + 3]
        points[channel_idx] = (distance_mm, angle_deg, channel_idx)

    return DecodedPacket(is_sync=False, points=points)


class SerialPacketReader:
    def __init__(self, serial_connection) -> None:
        self._serial_connection = serial_connection
        self._buffer = bytearray()

    def _read_more(self) -> bytes:
        waiting = getattr(self._serial_connection, "in_waiting", 0)
        chunk_size = max(PACKET_SIZE, int(waiting) if waiting else 1)
        return self._serial_connection.read(chunk_size)

    def read_packet(self, stop_event: Event | None = None) -> DecodedPacket | None:
        while stop_event is None or not stop_event.is_set():
            start_index = self._buffer.find(START_MARKER)
            if start_index == -1:
                chunk = self._read_more()
                if not chunk:
                    continue

                if self._buffer and self._buffer[-1:] == START_MARKER[:1] and chunk[:1] == START_MARKER[1:2]:
                    self._buffer[:] = START_MARKER
                    self._buffer.extend(chunk[1:])
                else:
                    self._buffer.clear()
                    if chunk[-1:] == START_MARKER[:1]:
                        self._buffer.extend(chunk[-1:])
                    if start_index == -1:
                        embedded_start = chunk.find(START_MARKER)
                        if embedded_start != -1:
                            self._buffer[:] = chunk[embedded_start:]
                continue

            if start_index > 0:
                del self._buffer[:start_index]

            while len(self._buffer) < PACKET_SIZE:
                chunk = self._read_more()
                if not chunk:
                    break
                self._buffer.extend(chunk)

            if len(self._buffer) < PACKET_SIZE:
                continue

            candidate = bytes(self._buffer[:PACKET_SIZE])
            if candidate[-2:] == END_MARKER:
                del self._buffer[:PACKET_SIZE]
                return decode_packet(candidate)

            del self._buffer[0]

        return None


class FrameAccumulator:
    def __init__(self) -> None:
        self._packet_points: list[np.ndarray] = []

    def add_packet(self, packet: DecodedPacket) -> np.ndarray | None:
        if packet.is_sync:
            if not self._packet_points:
                return None

            frame = np.vstack(self._packet_points).astype(np.int32, copy=False)
            self._packet_points.clear()
            return frame

        if packet.points is None:
            raise ValueError("Normal packets must contain decoded points.")

        self._packet_points.append(packet.points)
        return None

    def flush(self) -> np.ndarray | None:
        if not self._packet_points:
            return None

        frame = np.vstack(self._packet_points).astype(np.int32, copy=False)
        self._packet_points.clear()
        return frame


def timestamped_frame_path(output_dir: Path) -> Path:
    return output_dir / f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f')}.npy"


def save_frame(frame: np.ndarray, output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    frame_path = timestamped_frame_path(output_dir)
    np.save(frame_path, frame.astype(np.int32, copy=False))
    return frame_path


def capture_frames(
    port: str,
    baud: int,
    output_dir: Path,
    serial_timeout: float,
    frame_limit: int | None,
    save_partial_on_exit: bool,
) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    saved_frames = 0
    accumulator = FrameAccumulator()

    with open_serial_connection(port=port, baud=baud, timeout=serial_timeout) as serial_connection:
        packet_reader = SerialPacketReader(serial_connection)
        print(f"Listening on {port} at {baud} baud.")
        print(f"Saving completed rotations into: {output_dir}")

        try:
            while frame_limit is None or saved_frames < frame_limit:
                packet = packet_reader.read_packet()
                if packet is None:
                    continue

                frame = accumulator.add_packet(packet)
                if frame is None:
                    continue

                frame_path = save_frame(frame, output_dir)
                saved_frames += 1
                print(f"Saved frame {saved_frames}: {frame_path.name} ({len(frame)} points)")
        except KeyboardInterrupt:
            print("Capture interrupted by user.")
            if save_partial_on_exit:
                partial_frame = accumulator.flush()
                if partial_frame is not None:
                    frame_path = save_frame(partial_frame, output_dir)
                    saved_frames += 1
                    print(
                        f"Saved partial frame {saved_frames}: {frame_path.name} "
                        f"({len(partial_frame)} points)"
                    )

    return saved_frames


def main() -> None:
    args = parse_args()
    output_dir = resolve_output_dir(args.output_dir)
    saved_frames = capture_frames(
        port=args.port,
        baud=args.baud,
        output_dir=output_dir,
        serial_timeout=args.serial_timeout,
        frame_limit=args.frame_limit,
        save_partial_on_exit=args.save_partial_on_exit,
    )
    print(f"Capture finished with {saved_frames} saved frame(s).")


if __name__ == "__main__":
    main()
