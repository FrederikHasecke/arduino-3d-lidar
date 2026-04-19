"""Offline point-cloud playback for recorded `.npy` LiDAR frames."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


DEFAULT_SAMPLE_PATH = Path("data/sequences/01/lidar_points")
ANGLE_OFFSET = np.array([0, 90, -135, -45, 135, 45, -90, 180], dtype=np.float32)
CHANNEL_ANGLE_LUT = np.linspace(-15, 15, 8, dtype=np.float32)[::-1]
LOW_Z_COLOR = np.array([0.20, 0.90, 1.00], dtype=np.float32)
HIGH_Z_COLOR = np.array([0.80, 0.65, 0.55], dtype=np.float32)
POINT_ALPHA = np.float32(0.95)
ORIGIN_MARKER_COLOR = np.array([[1.0, 0.82, 0.18, 1.0]], dtype=np.float32)
ORIGIN_LINE_COLORS = np.array(
    [
        [1.0, 0.35, 0.35, 1.0],
        [1.0, 0.35, 0.35, 1.0],
        [0.35, 1.0, 0.55, 1.0],
        [0.35, 1.0, 0.55, 1.0],
        [0.45, 0.7, 1.0, 1.0],
        [0.45, 0.7, 1.0, 1.0],
    ],
    dtype=np.float32,
)


def spherical_to_cartesian(pc_sphere: np.ndarray) -> np.ndarray:
    """Project a point cloud from spherical coordinates to cartesian coordinates."""
    pc_cartesian = np.zeros((pc_sphere.shape[0], 3), dtype=np.float32)
    pc_cartesian[:, 0] = pc_sphere[:, 0] * np.cos(pc_sphere[:, 2]) * np.cos(pc_sphere[:, 1])
    pc_cartesian[:, 1] = pc_sphere[:, 0] * np.cos(pc_sphere[:, 2]) * np.sin(pc_sphere[:, 1])
    pc_cartesian[:, 2] = pc_sphere[:, 0] * np.sin(pc_sphere[:, 2])
    return pc_cartesian


def convert_to_cartesian(pc: np.ndarray) -> np.ndarray:
    """Convert raw `(distance_mm, rotation_deg, channel_idx)` points to XYZ meters."""
    if pc.ndim != 2 or pc.shape[1] != 3:
        raise ValueError(f"Expected a point cloud with shape (n, 3), got {pc.shape}.")

    channel_indices = pc[:, 2].astype(int)
    if np.any(channel_indices < 0) or np.any(channel_indices >= len(CHANNEL_ANGLE_LUT)):
        raise ValueError("Point cloud contains channel indices outside the expected range 0-7.")

    pc_sphere = np.zeros((pc.shape[0], 3), dtype=np.float32)
    pc_sphere[:, 0] = pc[:, 0].astype(np.float32) / 1000.0
    pc_sphere[:, 1] = np.radians((pc[:, 1] + ANGLE_OFFSET[channel_indices] - 90) % 360)
    pc_sphere[:, 2] = np.radians(CHANNEL_ANGLE_LUT[channel_indices])
    return spherical_to_cartesian(pc_sphere)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize LiDAR point cloud frames from a .npy file or directory."
    )
    parser.add_argument(
        "--path",
        type=Path,
        help="Optional path to a .npy file or a directory containing .npy frames.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=2.0,
        help="Playback speed in frames per second. Defaults to 2.0.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=8.0,
        help="Point size in screen pixels. Defaults to 8.0.",
    )
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_input_path(path_arg: Path | None) -> tuple[Path, bool]:
    if path_arg is None:
        return repo_root() / DEFAULT_SAMPLE_PATH, True

    candidate = path_arg.expanduser()
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    return candidate.resolve(), False


def collect_frame_paths(input_path: Path) -> list[Path]:
    if not input_path.exists():
        raise FileNotFoundError(f"Input path does not exist: {input_path}")

    if input_path.is_file():
        if input_path.suffix.lower() != ".npy":
            raise ValueError(f"Expected a .npy file, got: {input_path}")
        return [input_path]

    frame_paths = sorted(path for path in input_path.iterdir() if path.suffix.lower() == ".npy")
    if not frame_paths:
        raise FileNotFoundError(f"No .npy files found in: {input_path}")
    return frame_paths


def load_point_clouds(frame_paths: list[Path]) -> list[np.ndarray]:
    point_clouds = []
    for frame_path in frame_paths:
        raw_point_cloud = np.load(frame_path)
        point_clouds.append(convert_to_cartesian(raw_point_cloud))
    return point_clouds


def print_default_sample_disclaimer(input_path: Path, frame_count: int) -> None:
    print(
        "Disclaimer: no --path was provided, so the viewer is running the bundled basic input "
        "pointcloud sample. Expect a rough, low-fidelity point cloud rather than production-grade data."
    )
    print(f"Using sample data from: {input_path}")
    print(f"Loaded {frame_count} frame(s).")


def load_vispy_modules():
    try:
        import vispy

        vispy.use(app="glfw")
        from vispy import app, scene
    except Exception as exc:
        raise RuntimeError(
            "Failed to initialize the VisPy GLFW viewer. Install the project dependencies and "
            "make sure your graphics drivers expose a working OpenGL context."
        ) from exc

    return app, scene


def point_colors(point_cloud: np.ndarray) -> np.ndarray:
    if len(point_cloud) == 0:
        return np.empty((0, 4), dtype=np.float32)

    z_values = point_cloud[:, 2]
    z_range = np.ptp(z_values)
    if z_range == 0:
        normalized = np.full_like(z_values, 0.5, dtype=np.float32)
    else:
        normalized = (z_values - z_values.min()) / z_range

    rgb = LOW_Z_COLOR + (HIGH_Z_COLOR - LOW_Z_COLOR) * normalized[:, None]
    colors = np.empty((len(point_cloud), 4), dtype=np.float32)
    colors[:, :3] = rgb
    colors[:, 3] = POINT_ALPHA
    return colors


def point_cloud_bounds(point_clouds: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    non_empty = [point_cloud for point_cloud in point_clouds if len(point_cloud)]
    if not non_empty:
        raise ValueError("The selected recording does not contain any points to visualize.")

    stacked = np.vstack(non_empty)
    return stacked.min(axis=0), stacked.max(axis=0)


def origin_axis_segments(axis_length: float) -> np.ndarray:
    return np.array(
        [
            [-axis_length, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            [0.0, -axis_length, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, -axis_length],
            [0.0, 0.0, axis_length],
        ],
        dtype=np.float32,
    )


class OfflinePointCloudViewer:
    def __init__(self, point_clouds: list[np.ndarray], fps: float, point_size: float) -> None:
        if fps <= 0:
            raise ValueError("--fps must be greater than 0.")
        if point_size <= 0:
            raise ValueError("--point-size must be greater than 0.")

        app, scene = load_vispy_modules()
        self._app = app
        self.point_clouds = point_clouds
        self.frame_count = len(point_clouds)
        self.frame_index = 0
        self.point_size = point_size

        self.canvas = scene.SceneCanvas(
            title="Arduino 3D LiDAR Offline Viewer",
            keys="interactive",
            bgcolor="#081018",
            size=(1280, 720),
            show=True,
        )
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.cameras.TurntableCamera(fov=45.0, azimuth=35.0, elevation=20.0)

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
        self._configure_camera()
        self._show_frame(0)

        self.timer = app.Timer(interval=1.0 / fps, connect=self._on_timer)
        if self.frame_count > 1:
            self.timer.start()

        self.canvas.events.close.connect(self._on_close)

    def _configure_camera(self) -> None:
        min_corner, max_corner = point_cloud_bounds(self.point_clouds)
        center = (min_corner + max_corner) / 2.0
        size = np.maximum(max_corner - min_corner, 0.1)
        padding = np.maximum(size * 0.15, 0.1)
        axis_length = float(max(np.max(size) * 0.08, 0.08))

        self.view.camera.center = tuple(center.tolist())
        self.view.camera.scale_factor = float(np.max(size + padding) * 1.6)
        self.view.camera.set_range(
            x=(float(min_corner[0] - padding[0]), float(max_corner[0] + padding[0])),
            y=(float(min_corner[1] - padding[1]), float(max_corner[1] + padding[1])),
            z=(float(min_corner[2] - padding[2]), float(max_corner[2] + padding[2])),
        )
        self.origin_marker.set_data(
            pos=np.zeros((1, 3), dtype=np.float32),
            face_color=ORIGIN_MARKER_COLOR,
            edge_color=np.array([[0.04, 0.04, 0.04, 1.0]], dtype=np.float32),
            edge_width=1.5,
            size=max(self.point_size * 1.75, 12.0),
            symbol="star",
        )
        self.origin_axes.set_data(pos=origin_axis_segments(axis_length), color=ORIGIN_LINE_COLORS)

    def _show_frame(self, frame_index: int) -> None:
        point_cloud = self.point_clouds[frame_index]
        self.markers.set_data(
            pos=point_cloud,
            face_color=point_colors(point_cloud),
            edge_width=0.0,
            size=self.point_size,
            symbol="disc",
        )
        self.canvas.title = (
            f"Arduino 3D LiDAR Offline Viewer ({frame_index + 1}/{self.frame_count})"
        )

    def _on_timer(self, event) -> None:
        self.frame_index = (self.frame_index + 1) % self.frame_count
        self._show_frame(self.frame_index)

    def _on_close(self, event) -> None:
        if self.timer.running:
            self.timer.stop()

    def run(self) -> None:
        self._app.run()


def playback_point_clouds(point_clouds: list[np.ndarray], fps: float, point_size: float) -> None:
    viewer = OfflinePointCloudViewer(point_clouds, fps, point_size)
    viewer.run()


def main() -> None:
    args = parse_args()
    input_path, using_default_sample = resolve_input_path(args.path)
    frame_paths = collect_frame_paths(input_path)
    point_clouds = load_point_clouds(frame_paths)

    if using_default_sample:
        print_default_sample_disclaimer(input_path, len(frame_paths))
    else:
        print(f"Loaded {len(frame_paths)} frame(s) from: {input_path}")

    print("Starting VisPy GLFW viewer. Close the window when you are done inspecting the sample.")
    playback_point_clouds(point_clouds, args.fps, args.point_size)


if __name__ == "__main__":
    main()




