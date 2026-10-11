"""2D scan registration for stitching recorded revolutions into one map.

The scanner is assumed to move on a flat floor, so every revolution gets a planar pose
(x, y, yaw) and heights are left untouched.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

# Measured median lag (deg) of each channel's reading behind the last one (channel 7) within
# a packet: the firmware reads the eight sensors one after another.
CHANNEL_READ_LAG_DEG = np.array([-11, -9, -6, -4, -3, -2, -1, 0], dtype=np.float64)
DEFAULT_YAW_SEEDS_DEG = (-30, -20, -10, 0, 10, 20, 30)


def retime_azimuths(raw: np.ndarray) -> np.ndarray:
    """Replace the encoder azimuths of one revolution by evenly spaced, time-based ones.

    The firmware adds 24 deg per encoder tooth, and spurious ticks show up in the recordings as
    sudden +24 deg jumps that rotate the rest of the revolution. The sync packet that starts a
    revolution is reliable and packets arrive at a steady rate, so packet k of n is placed at
    360 * (k + 0.5) / n, plus each channel's read lag within the packet.
    """
    out = raw.copy()
    packets = max(1, len(raw) // 8)
    packet_index = np.arange(len(raw)) // 8
    channels = raw[:, 2].astype(int)
    azimuth = 360.0 * (packet_index + 0.5) / packets + CHANNEL_READ_LAG_DEG[channels]
    out[:, 1] = np.round(azimuth).astype(raw.dtype)
    return out


def make_pose_2d(x: float, y: float, yaw: float) -> np.ndarray:
    """Build a 3x3 homogeneous 2D transform from a translation and a yaw angle (radians)."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def apply_pose_2d(pose: np.ndarray, xy: np.ndarray) -> np.ndarray:
    return xy @ pose[:2, :2].T + pose[:2, 2]


def rotate_pose_2d(pose: np.ndarray, yaw: float) -> np.ndarray:
    """Turn a pose in place about its own position by `yaw` radians."""
    turned = pose.copy()
    turned[:2, :2] = make_pose_2d(0.0, 0.0, yaw)[:2, :2] @ pose[:2, :2]
    return turned


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """Keep one point (the centroid) per occupied voxel."""
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), points.shape[1]))
    np.add.at(sums, inverse.ravel(), points)
    return sums / counts[:, None]


def estimate_normals_2d(points: np.ndarray, k: int = 6) -> np.ndarray:
    """Unit normal of each point from a PCA of its `k` nearest neighbours."""
    k = min(k, len(points))
    _, idx = cKDTree(points).query(points, k=k)
    neighbours = points[idx] - points[idx].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", neighbours, neighbours)
    _, vectors = np.linalg.eigh(cov)
    return vectors[:, :, 0]


class ScanMap:
    """Target point set for registration, with its search tree and normals."""

    def __init__(self, points: np.ndarray, voxel: float = 0.03) -> None:
        self.points = voxel_downsample(points, voxel)
        self.tree = cKDTree(self.points)
        self.normals = estimate_normals_2d(self.points)


def alignment_score(src_xy: np.ndarray, target: ScanMap, pose: np.ndarray, clip: float = 0.1) -> float:
    """Mean nearest-neighbour distance (m), clipped so that non-overlapping points count equally."""
    dist, _ = target.tree.query(apply_pose_2d(pose, src_xy))
    return float(np.mean(np.minimum(dist, clip)))


def icp_point_to_line_2d(
    src_xy: np.ndarray,
    target: ScanMap,
    init: np.ndarray | None = None,
    iters: int = 30,
    max_dist: float = 0.15,
    huber: float = 0.03,
) -> tuple[np.ndarray, float]:
    """Align `src_xy` to `target` by minimising point-to-line distances (Gauss-Newton, Huber).

    Point-to-line suits wall-like scans much better than point-to-point: points may slide along
    a wall without penalty, so sparse samples of the same wall do not pull the fit sideways.
    Returns the 3x3 pose mapping source points into the target frame and `alignment_score`.
    """
    pose = np.eye(3) if init is None else init.copy()
    for _ in range(iters):
        moved = apply_pose_2d(pose, src_xy)
        dist, idx = target.tree.query(moved)
        keep = dist < max_dist
        if keep.sum() < 8:
            break
        p, q, n = moved[keep], target.points[idx[keep]], target.normals[idx[keep]]
        error = np.einsum("ij,ij->i", n, p - q)
        jacobian = np.column_stack([n[:, 0], n[:, 1], n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1]])
        weights = huber / np.maximum(np.abs(error), huber)
        step = np.linalg.solve(
            jacobian.T @ (jacobian * weights[:, None]) + 1e-6 * np.eye(3), -jacobian.T @ (weights * error)
        )
        pose = make_pose_2d(*step) @ pose
        if np.abs(step).max() < 1e-5:
            break
    return pose, alignment_score(src_xy, target, pose)


def best_of_seeds(
    src_xy: np.ndarray, target: ScanMap, seeds: list[np.ndarray], yaw_seeds_deg=DEFAULT_YAW_SEEDS_DEG
) -> tuple[np.ndarray, float]:
    """Run ICP from every seed turned by every yaw offset and keep the best-scoring result."""
    results = [
        icp_point_to_line_2d(src_xy, target, rotate_pose_2d(seed, np.radians(yaw)))
        for seed in seeds
        for yaw in yaw_seeds_deg
    ]
    return min(results, key=lambda result: result[1])


def pose_2d_to_3d(pose: np.ndarray) -> np.ndarray:
    """Lift a planar (x, y, yaw) pose to a 4x4 transform with z unchanged."""
    pose_3d = np.eye(4)
    pose_3d[:2, :2] = pose[:2, :2]
    pose_3d[:2, 3] = pose[:2, 2]
    return pose_3d


def register_sequence(
    scans_xy: list[np.ndarray],
    voxel: float = 0.03,
    yaw_seeds_deg=DEFAULT_YAW_SEEDS_DEG,
    refine_passes: int = 2,
    min_points: int = 10,
) -> tuple[list[np.ndarray], list[float]]:
    """Estimate a planar pose for every scan and return 4x4 poses (scan -> map) and scores.

    1. Each scan is aligned to the map of all scans before it. ICP starts from the previous pose
       and from a constant-velocity guess, each turned by several yaw offsets, because the
       coarse scans have many local minima.
    2. Each refinement pass re-aligns every scan against the map of all other scans, which
       spreads the remaining drift over the whole sequence instead of letting it pile up.
    """
    poses = [np.eye(3)]
    scores = [0.0]
    world = [scans_xy[0]]
    for xy in scans_xy[1:]:
        seeds = [poses[-1]]
        if len(poses) >= 2:
            seeds.append(poses[-1] @ np.linalg.inv(poses[-2]) @ poses[-1])
        if len(xy) < min_points:
            pose, score = seeds[-1], float("nan")
        else:
            pose, score = best_of_seeds(xy, ScanMap(np.vstack(world), voxel), seeds, yaw_seeds_deg)
        poses.append(pose)
        scores.append(score)
        world.append(apply_pose_2d(pose, xy))

    for _ in range(refine_passes):
        for i, xy in enumerate(scans_xy):
            if i == 0 or len(xy) < min_points:
                continue
            others = ScanMap(np.vstack(world[:i] + world[i + 1 :]), voxel)
            poses[i], scores[i] = icp_point_to_line_2d(xy, others, poses[i], iters=15)
            world[i] = apply_pose_2d(poses[i], xy)
    return [pose_2d_to_3d(pose) for pose in poses], scores


def accumulate(point_clouds: list[np.ndarray], poses: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Transform every frame into the map frame and stack them, with the source frame index per point."""
    points = [pc @ pose[:3, :3].T + pose[:3, 3] for pc, pose in zip(point_clouds, poses)]
    frame_ids = np.concatenate([np.full(len(pc), i) for i, pc in enumerate(point_clouds)])
    return np.vstack(points), frame_ids


def frame_support(points: np.ndarray, frame_ids: np.ndarray, radius: float = 0.04) -> np.ndarray:
    """For each point, count how many *other* frames have a point within `radius`.

    Real surfaces are seen again and again as the scanner moves; one-off readings are noise.
    """
    neighbours = cKDTree(points).query_ball_point(points, r=radius)
    return np.array(
        [len(np.unique(frame_ids[idx])) - 1 for idx in neighbours],
        dtype=np.int64,
    )
