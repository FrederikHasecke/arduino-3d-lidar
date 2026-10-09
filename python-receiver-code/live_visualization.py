"""Live point-cloud visualization for the Arduino/ESP32 LiDAR serial stream."""

from __future__ import annotations

import argparse
import queue
import threading
from pathlib import Path

import numpy as np

from offline_visualization import (
    MAX_RANGE_MM,
    configure_origin,
    convert_to_cartesian,
    drop_invalid_points,
    fit_view_to_bounds,
    load_vispy_modules,
    point_colors,
)
from receive_data import FrameAccumulator, SerialPacketReader, open_serial_connection, resolve_output_dir, save_frame


DEFAULT_BAUD = 115200
DEFAULT_REFRESH_HZ = 30.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live-visualize LiDAR frames from the Arduino/ESP32 serial stream."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/ttyUSB0.")
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate. Defaults to {DEFAULT_BAUD}.",
    )
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=0.25,
        help="Serial read timeout in seconds. Defaults to 0.25.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=8.0,
        help="Point size in screen pixels. Defaults to 8.0.",
    )
    parser.add_argument(
        "--keep-invalid",
        action="store_true",
        help=f"Also draw points with no valid return (0 mm or clamped to {MAX_RANGE_MM} mm).",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save every completed frame to disk while visualizing.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory to save timestamped .npy frames into when --save is used.",
    )
    return parser.parse_args()


class SerialFrameWorker(threading.Thread):
    def __init__(
        self,
        serial_connection,
        frame_queue: queue.SimpleQueue[tuple[np.ndarray, Path | None]],
        output_dir: Path | None,
    ) -> None:
        super().__init__(daemon=True)
        self._serial_connection = serial_connection
        self._frame_queue = frame_queue
        self._output_dir = output_dir
        self._stop_event = threading.Event()
        self._accumulator = FrameAccumulator()
        self.error: Exception | None = None

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            packet_reader = SerialPacketReader(self._serial_connection)
            while not self._stop_event.is_set():
                packet = packet_reader.read_packet(stop_event=self._stop_event)
                if packet is None:
                    continue

                frame = self._accumulator.add_packet(packet)
                if frame is None:
                    continue

                frame_path = None
                if self._output_dir is not None:
                    frame_path = save_frame(frame, self._output_dir)
                self._frame_queue.put((frame, frame_path))
        except Exception as exc:  # pragma: no cover - runtime error reporting path
            self.error = exc
        finally:
            self._serial_connection.close()


class LivePointCloudViewer:
    def __init__(
        self,
        worker: SerialFrameWorker,
        frame_queue: queue.SimpleQueue[tuple[np.ndarray, Path | None]],
        point_size: float,
        keep_invalid: bool = False,
    ) -> None:
        if point_size <= 0:
            raise ValueError("--point-size must be greater than 0.")

        app, scene = load_vispy_modules()
        self._app = app
        self._worker = worker
        self._frame_queue = frame_queue
        self._point_size = point_size
        self._keep_invalid = keep_invalid
        self._frame_count = 0
        self._latest_saved_path: Path | None = None
        self._bounds_min: np.ndarray | None = None
        self._bounds_max: np.ndarray | None = None

        self.canvas = scene.SceneCanvas(
            title="Arduino 3D LiDAR Live Viewer",
            keys="interactive",
            bgcolor="#081018",
            size=(1280, 720),
            show=True,
        )
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.cameras.TurntableCamera(
            fov=45.0,
            azimuth=35.0,
            elevation=20.0,
            center=(0.0, 0.0, 0.0),
            scale_factor=5.0,
        )

        self.markers = scene.visuals.Markers(parent=self.view.scene)
        self.markers.set_gl_state(
            depth_test=True,
            blend=True,
            blend_func=("src_alpha", "one_minus_src_alpha"),
        )
        self.origin_marker = scene.visuals.Markers(parent=self.view.scene)
        self.origin_marker.set_gl_state(
            depth_test=True,
            blend=True,
            blend_func=("src_alpha", "one_minus_src_alpha"),
        )
        self.origin_axes = scene.visuals.Line(
            parent=self.view.scene,
            connect="segments",
            width=2.0,
            method="gl",
        )
        configure_origin(self.origin_marker, self.origin_axes, axis_length=0.2, point_size=point_size)

        self.timer = app.Timer(interval=1.0 / DEFAULT_REFRESH_HZ, connect=self._on_timer, start=True)
        self.canvas.events.close.connect(self._on_close)

    def _apply_camera_bounds(self) -> None:
        if self._bounds_min is None or self._bounds_max is None:
            return
        fit_view_to_bounds(
            self.view,
            self.origin_marker,
            self.origin_axes,
            self._bounds_min,
            self._bounds_max,
            self._point_size,
        )

    def _show_frame(self, raw_frame: np.ndarray, saved_path: Path | None) -> None:
        if not self._keep_invalid:
            raw_frame = drop_invalid_points(raw_frame)
        point_cloud = convert_to_cartesian(raw_frame)
        self.markers.set_data(
            pos=point_cloud,
            face_color=point_colors(point_cloud),
            edge_width=0.0,
            size=self._point_size,
            symbol="disc",
        )

        if len(point_cloud):
            frame_min = point_cloud.min(axis=0)
            frame_max = point_cloud.max(axis=0)
            if self._bounds_min is None or self._bounds_max is None:
                self._bounds_min = frame_min
                self._bounds_max = frame_max
            else:
                self._bounds_min = np.minimum(self._bounds_min, frame_min)
                self._bounds_max = np.maximum(self._bounds_max, frame_max)
            self._apply_camera_bounds()

        self._frame_count += 1
        self._latest_saved_path = saved_path
        title = f"Arduino 3D LiDAR Live Viewer ({self._frame_count} frame(s), {len(raw_frame)} points)"
        if saved_path is not None:
            title = f"{title} - saved {saved_path.name}"
        self.canvas.title = title

    def _on_timer(self, event) -> None:
        latest_frame: tuple[np.ndarray, Path | None] | None = None
        while True:
            try:
                latest_frame = self._frame_queue.get_nowait()
            except queue.Empty:
                break

        if latest_frame is not None:
            self._show_frame(*latest_frame)

        if self._worker.error is not None:
            self.timer.stop()
            raise RuntimeError("Serial frame worker stopped unexpectedly.") from self._worker.error

    def _on_close(self, event) -> None:
        if self.timer.running:
            self.timer.stop()
        self._worker.stop()
        self._worker.join(timeout=2.0)

    def run(self) -> None:
        self._worker.start()
        self._app.run()


def main() -> None:
    args = parse_args()
    output_dir = resolve_output_dir(args.output_dir) if args.save else None
    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        print(f"Saving live frames into: {output_dir}")

    serial_connection = open_serial_connection(
        port=args.port,
        baud=args.baud,
        timeout=args.serial_timeout,
    )
    print(f"Listening on {args.port} at {args.baud} baud.")

    frame_queue: queue.SimpleQueue[tuple[np.ndarray, Path | None]] = queue.SimpleQueue()
    worker = SerialFrameWorker(serial_connection, frame_queue, output_dir)
    viewer = LivePointCloudViewer(
        worker, frame_queue, point_size=args.point_size, keep_invalid=args.keep_invalid
    )
    viewer.run()


if __name__ == "__main__":
    main()
