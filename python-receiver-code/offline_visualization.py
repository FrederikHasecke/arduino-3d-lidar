"""Offline point-cloud playback for recorded `.npy` LiDAR frames."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import open3d as o3d


DEFAULT_SAMPLE_PATH = Path("data/sequences/01/lidar_points")
ANGLE_OFFSET = np.array([0, 90, -135, -45, 135, 45, -90, 180], dtype=np.float32)
CHANNEL_ANGLE_LUT = np.linspace(-15, 15, 8, dtype=np.float32)[::-1]


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


def playback_point_clouds(point_clouds: list[np.ndarray], fps: float) -> None:
    if fps <= 0:
        raise ValueError("--fps must be greater than 0.")

    frame_delay = 1.0 / fps
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Arduino 3D LiDAR Offline Viewer")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_clouds[0])
    vis.add_geometry(pcd)

    render_option = vis.get_render_option()
    render_option.point_size = 3.0

    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)
    ctr.set_front([0.0, 0.0, -1.0])
    ctr.set_lookat([0.0, 0.0, 0.0])

    for frame in point_clouds:
        pcd.points = o3d.utility.Vector3dVector(frame)
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(frame_delay)

    vis.destroy_window()


def main() -> None:
    args = parse_args()
    input_path, using_default_sample = resolve_input_path(args.path)
    frame_paths = collect_frame_paths(input_path)
    point_clouds = load_point_clouds(frame_paths)

    if using_default_sample:
        print_default_sample_disclaimer(input_path, len(frame_paths))
    else:
        print(f"Loaded {len(frame_paths)} frame(s) from: {input_path}")

    playback_point_clouds(point_clouds, args.fps)


if __name__ == "__main__":
    main()




